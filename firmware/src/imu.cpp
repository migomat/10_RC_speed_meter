#include "imu.h"

#include <Arduino.h>
#include "config.h"

// Rejestry LSM6DS3TR-C
#define REG_FIFO_CTRL3   0x08
#define REG_FIFO_CTRL5   0x0A
#define REG_WHO_AM_I     0x0F
#define REG_CTRL1_XL     0x10
#define REG_CTRL2_G      0x11
#define REG_CTRL3_C      0x12
#define REG_CTRL4_C      0x13
#define REG_CTRL6_C      0x15
#define REG_CTRL8_XL     0x17
#define REG_FIFO_STATUS1 0x3A
#define REG_FIFO_STATUS3 0x3C
#define REG_FIFO_DATA    0x3E
#define WHO_AM_I_VALUE   0x6A

#if IMU_ACCEL_RANGE_G == 2
#define XL_FS 0x0
#define XL_SCALE (0.061e-3f)
#elif IMU_ACCEL_RANGE_G == 4
#define XL_FS 0x2
#define XL_SCALE (0.122e-3f)
#elif IMU_ACCEL_RANGE_G == 8
#define XL_FS 0x3
#define XL_SCALE (0.244e-3f)
#elif IMU_ACCEL_RANGE_G == 16
#define XL_FS 0x1
#define XL_SCALE (0.488e-3f)
#else
#error "IMU_ACCEL_RANGE_G: 2/4/8/16"
#endif

#if IMU_GYRO_RANGE_DPS == 250
#define G_FS 0x0
#define G_SCALE (8.75e-3f)
#elif IMU_GYRO_RANGE_DPS == 500
#define G_FS 0x1
#define G_SCALE (17.5e-3f)
#elif IMU_GYRO_RANGE_DPS == 1000
#define G_FS 0x2
#define G_SCALE (35.0e-3f)
#elif IMU_GYRO_RANGE_DPS == 2000
#define G_FS 0x3
#define G_SCALE (70.0e-3f)
#else
#error "IMU_GYRO_RANGE_DPS: 250/500/1000/2000"
#endif

#if IMU_ODR_HZ == 833
#define ODR_CODE 0x7
#elif IMU_ODR_HZ == 1666
#define ODR_CODE 0x8
#elif IMU_ODR_HZ == 416
#define ODR_CODE 0x6
#else
#error "IMU_ODR_HZ: 416/833/1666"
#endif

#define IMU_PERIOD_US   (1000000u / IMU_ODR_HZ)
#define HIST_LEN        32
#define SETS_PER_READ   8     // 8 × 12 B = 96 B na transakcję (EasyDMA)
#define MAX_SETS_PER_POLL 24  // ogranicza czas blokowania pętli (~7 ms)

static bool     s_ok;
static ImuRaw   s_hist[HIST_LEN];
static uint32_t s_hist_n;          // liczba zapisanych (narastająco)
static uint32_t s_overruns;
static uint32_t s_misaligned;
static bool     s_overrun_flag;
static uint32_t s_last_t;

// ---------------------------------------------------------------- I2C
// Własna obsługa TWIM1 (piny Wire1) zamiast biblioteki Wire: Wire robi
// powtórzony start programowo (SUSPEND po LASTTX). Gdy SoftDevice opóźni CPU
// w tym oknie, TWIM czyta dodatkowe słowo z FIFO i zestawy się przesuwają
// (zmierzone: ~1 % serii przy włączonym BLE, 0 bez BLE). Tu cała transakcja
// zapis-rejestru → odczyt → STOP idzie sprzętowo (SHORTS), bez udziału CPU.
#define TWIM NRF_TWIM1
static uint8_t s_tx[2];

static bool twim_wait(void) {
  uint32_t t0 = millis();
  while (!TWIM->EVENTS_STOPPED) {
    if (TWIM->EVENTS_ERROR) {
      TWIM->EVENTS_ERROR = 0;
      TWIM->TASKS_STOP = 1;
      uint32_t t1 = millis();
      while (!TWIM->EVENTS_STOPPED && millis() - t1 < 5) {
      }
      TWIM->EVENTS_STOPPED = 0;
      TWIM->ERRORSRC = TWIM->ERRORSRC;
      return false;
    }
    if (millis() - t0 > 10) {
      TWIM->TASKS_STOP = 1;
      return false;
    }
  }
  TWIM->EVENTS_STOPPED = 0;
  return true;
}

static void twim_begin(void) {
  uint32_t scl = g_ADigitalPinMap[PIN_WIRE1_SCL], sda = g_ADigitalPinMap[PIN_WIRE1_SDA];
  const uint32_t cnf = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                       (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                       (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                       (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos);
  NRF_GPIO_Type* port_scl = scl >= 32 ? NRF_P1 : NRF_P0;
  NRF_GPIO_Type* port_sda = sda >= 32 ? NRF_P1 : NRF_P0;
  port_scl->PIN_CNF[scl & 31] = cnf;
  port_sda->PIN_CNF[sda & 31] = cnf;
  TWIM->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
  TWIM->PSEL.SCL = scl;
  TWIM->PSEL.SDA = sda;
  TWIM->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K400;  // maks. dla nRF52840 TWIM
  TWIM->ADDRESS = IMU_I2C_ADDR;
  TWIM->INTEN = 0;
  TWIM->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;
}

static bool wr(uint8_t reg, uint8_t val) {
  s_tx[0] = reg;
  s_tx[1] = val;
  TWIM->SHORTS = TWIM_SHORTS_LASTTX_STOP_Msk;
  TWIM->TXD.PTR = (uint32_t)s_tx;
  TWIM->TXD.MAXCNT = 2;
  TWIM->EVENTS_STOPPED = 0;
  TWIM->EVENTS_ERROR = 0;
  TWIM->TASKS_STARTTX = 1;
  return twim_wait();
}

static bool rdn(uint8_t reg, uint8_t* buf, uint16_t n) {
  s_tx[0] = reg;
  TWIM->SHORTS = TWIM_SHORTS_LASTTX_STARTRX_Msk | TWIM_SHORTS_LASTRX_STOP_Msk;
  TWIM->TXD.PTR = (uint32_t)s_tx;
  TWIM->TXD.MAXCNT = 1;
  TWIM->RXD.PTR = (uint32_t)buf;
  TWIM->RXD.MAXCNT = n;
  TWIM->EVENTS_STOPPED = 0;
  TWIM->EVENTS_ERROR = 0;
  TWIM->TASKS_STARTTX = 1;
  return twim_wait() && TWIM->RXD.AMOUNT == n;
}

bool imu_begin(void) {
  // Zasilanie IMU z pinu P1.08 – wymaga trybu wysokiej wydajności (H0H1),
  // tak jak w bibliotece Seeed_Arduino_LSM6DS3.
  NRF_P1->PIN_CNF[8] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                       (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                       (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);
  NRF_P1->OUTSET = 1u << 8;
  delay(30);  // rozruch LSM6DS3 (inicjalizacja, poza pętlą główną)
  twim_begin();

  uint8_t who = 0;
  if (!rdn(REG_WHO_AM_I, &who, 1) || who != WHO_AM_I_VALUE) {
    s_ok = false;
    return false;
  }
  wr(REG_CTRL3_C, 0x01);  // SW_RESET
  delay(20);
  bool ok = true;
  ok &= wr(REG_CTRL3_C, 0x44);  // BDU, IF_INC
  // ODR, zakres, filtr analogowy anty-aliasing 400 Hz (BW0_XL = 1)
  ok &= wr(REG_CTRL1_XL, (ODR_CODE << 4) | (XL_FS << 2) | 0x01);
  ok &= wr(REG_CTRL2_G, (ODR_CODE << 4) | (G_FS << 2));
  // Cyfrowy LPF2 akcelerometru ODR/9 ≈ 92 Hz (przed decymacją do 500 Hz)
  ok &= wr(REG_CTRL8_XL, 0x80 | (0x2 << 5));
  // LPF1 żyroskopu, FTYPE = 10 (~155 Hz przy 833 Hz)
  ok &= wr(REG_CTRL4_C, 0x02);
  ok &= wr(REG_CTRL6_C, 0x02);
  // FIFO: bez decymacji żyro i akcelerometru, tryb ciągły z ODR IMU
  ok &= wr(REG_FIFO_CTRL5, 0x00);
  ok &= wr(REG_FIFO_CTRL3, 0x09);
  ok &= wr(REG_FIFO_CTRL5, (ODR_CODE << 3) | 0x6);
  s_ok = ok;
  return ok;
}

bool imu_ok(void) { return s_ok; }

int imu_poll(uint32_t now_us) {
  if (!s_ok) return 0;
  uint8_t st[4];
  if (!rdn(REG_FIFO_STATUS1, st, 4)) return 0;
  uint32_t words = st[0] | ((uint32_t)(st[1] & 0x07) << 8);
  bool over = st[1] & 0x40;
  bool full = st[1] & 0x20;
  uint16_t pattern = st[2] | ((uint16_t)(st[3] & 0x03) << 8);
  if (full && words == 0) words = 2048;  // DIFF_FIFO ma 11 bitów
  if (over) {
    s_overruns++;
    s_overrun_flag = true;
  }
  // Wyrównanie do początku zestawu (gx gy gz ax ay az)
  while (pattern != 0 && words > 0) {
    uint8_t b[2];
    if (!rdn(REG_FIFO_DATA, b, 2)) return 0;
    words--;
    pattern = (pattern + 1) % 6;
  }
  // Ostatni zestaw w FIFO może być w trakcie zapisu – czytanie do samego
  // końca dawało czasem powtórzone/przesunięte słowa. Zostawiamy 1 zestaw.
  uint32_t sets = words / 6;
  if (sets <= 1) return 0;
  sets -= 1;
  uint32_t total = sets + 1;
  if (sets > MAX_SETS_PER_POLL) sets = MAX_SETS_PER_POLL;

  int got = 0;
  while (got < (int)sets) {
    int n = sets - got;
    if (n > SETS_PER_READ) n = SETS_PER_READ;
    uint8_t buf[SETS_PER_READ * 12];
    if (!rdn(REG_FIFO_DATA, buf, n * 12)) break;
    // Kontrola wyrównania: po pełnych zestawach wzorzec musi wskazywać gx (0).
    // Zestawy z przesuniętej serii są odrzucane, kolejne wywołanie wyrówna FIFO.
    uint8_t pt[2];
    if (!rdn(REG_FIFO_STATUS3, pt, 2) || (pt[0] | ((pt[1] & 0x03) << 8)) != 0) {
      s_misaligned++;
      break;
    }
    for (int k = 0; k < n; k++) {
      const int16_t* v = (const int16_t*)(buf + k * 12);
      // Czas: ostatnia próbka w FIFO ≈ teraz, wcześniejsze co okres ODR
      uint32_t idx_from_end = total - 1 - (got + k);
      uint32_t t = now_us - idx_from_end * IMU_PERIOD_US;
      if (s_hist_n && (int32_t)(t - s_last_t) < (int32_t)(IMU_PERIOD_US / 2)) t = s_last_t + IMU_PERIOD_US / 2;
      ImuRaw& r = s_hist[s_hist_n % HIST_LEN];
      r.t_us = t;
      r.gx = v[0]; r.gy = v[1]; r.gz = v[2];
      r.ax = v[3]; r.ay = v[4]; r.az = v[5];
      s_last_t = t;
      s_hist_n++;
    }
    got += n;
  }
  return got;
}

bool imu_get_at(uint32_t t_us, ImuRaw* out) {
  if (s_hist_n == 0) return false;
  uint32_t n = s_hist_n < HIST_LEN ? s_hist_n : HIST_LEN;
  for (uint32_t i = 0; i < n; i++) {
    const ImuRaw& r = s_hist[(s_hist_n - 1 - i) % HIST_LEN];
    if ((int32_t)(r.t_us - t_us) <= (int32_t)(IMU_PERIOD_US / 2)) {
      *out = r;
      return (int32_t)(t_us - r.t_us) <= (int32_t)(IMU_PERIOD_US * 2);
    }
  }
  *out = s_hist[(s_hist_n - n) % HIST_LEN];
  return false;
}

uint32_t imu_overrun_count(void) { return s_overruns; }
uint32_t imu_misaligned_count(void) { return s_misaligned; }
bool imu_take_overrun_flag(void) {
  bool f = s_overrun_flag;
  s_overrun_flag = false;
  return f;
}
uint32_t imu_sample_count(void) { return s_hist_n; }
float imu_accel_scale_g(void) { return XL_SCALE; }
float imu_gyro_scale_dps(void) { return G_SCALE; }
