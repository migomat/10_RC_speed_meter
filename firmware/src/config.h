// X4F Launch Logger – wszystkie parametry kompilacji.
// Jednostki w nazwach albo w komentarzach. Wartości oznaczone
// „DO ZMIERZENIA” trzeba potwierdzić na prawdziwym aucie.
#pragma once

#include <stdint.h>

#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    0
#define FW_VERSION_PATCH    0
#define FW_VERSION_STR      "1.0.0"

// ---------------------------------------------------------------- hall
// Tryb czujnika halla: 1 (tylko linia A) albo 3 (A/B/C).
// Można nadpisać z linii poleceń: -DHALL_LINES=3
#ifndef HALL_LINES
#define HALL_LINES          1
#endif
#if HALL_LINES != 1 && HALL_LINES != 3
#error "HALL_LINES musi być 1 albo 3"
#endif

#define MOTOR_POLE_PAIRS    1       // DO ZMIERZENIA / potwierdzenia
#define EDGES_PER_REV       (2 * HALL_LINES * MOTOR_POLE_PAIRS)
#define HALL_MIN_EDGE_US    50      // [us] krótsze impulsy = zakłócenie
// [us] brak zbocza → prędkość 0. Najniższe mierzalne obroty:
// 60e6 / (EDGES_PER_REV · timeout). Tryb 1 przy 50 ms nie zmierzyłby 500 rpm
// (zbocze co 60 ms), dlatego 80 ms (min. 375 rpm); tryb 3: 50 ms (200 rpm).
#if HALL_LINES == 1
#define HALL_TIMEOUT_US     80000
#else
#define HALL_TIMEOUT_US     50000
#endif
#define MAX_MOTOR_RPM       40000   // [rpm] do wymiarowania bufora zboczy

// Napięcie podciągania linii halla w ESC – DO ZMIERZENIA multimetrem.
// 5 → dzielnik 47k/68k, 3.3 → tylko 10k szeregowo. Firmware tego nie
// używa, wartość tylko informacyjnie (wpisywana do nagłówka startu ×10).
#define HALL_PULLUP_V_X10   50

// ---------------------------------------------------------------- piny
#define PIN_THROTTLE        D0
#define PIN_HALL_A          D1
#define PIN_HALL_B          D2      // tylko HALL_LINES == 3
#define PIN_HALL_C          D3      // tylko HALL_LINES == 3

// ---------------------------------------------------------------- gaz
#define THROTTLE_MIN_US             800    // [us] impulsy spoza zakresu odrzucane
#define THROTTLE_MAX_US             2200   // [us]
#define THROTTLE_TIMEOUT_MS         100    // [ms] brak impulsu → brak sygnału
#define THROTTLE_NEUTRAL_LEARN_MS   2000   // [ms] okno uczenia neutrum
#define THROTTLE_NEUTRAL_SPREAD_US  10     // [us] maks. rozrzut w oknie
#define THROTTLE_FULL_SCALE_US      500    // [us] neutrum → pełny gaz (100 %)
#define THROTTLE_FORWARD_IS_HIGHER  1      // 1: „do przodu” = dłuższy impuls (zależy od reverse w aparaturze)

// ---------------------------------------------------------------- logger
#define SAMPLE_RATE_HZ      500
#define SAMPLE_PERIOD_US    (1000000 / SAMPLE_RATE_HZ)
#define PRETRIGGER_MS       500
#define POSTTRIGGER_MS      3000
#define TRIGGER_THROTTLE_PCT 20     // [% zakresu do przodu od neutrum]

#define PRE_SAMPLES         (PRETRIGGER_MS * SAMPLE_RATE_HZ / 1000)
#define POST_SAMPLES        (POSTTRIGGER_MS * SAMPLE_RATE_HZ / 1000)
#define MAX_SAMPLES         (PRE_SAMPLES + POST_SAMPLES)

#define LOG_RAW_EDGES       1       // zapis surowych znaczników czasu zboczy
// Bufor zboczy: maks. obroty × zbocza/obrót × długość okna (+10 % zapasu)
#define MAX_EDGES           ((uint32_t)((uint64_t)MAX_MOTOR_RPM * EDGES_PER_REV * \
                              (PRETRIGGER_MS + POSTTRIGGER_MS) * 11 / (60ULL * 1000 * 10)))

// ---------------------------------------------------------------- IMU
#define IMU_ACCEL_RANGE_G   4       // ±4 g
#define IMU_GYRO_RANGE_DPS  500     // ±500 dps
#define IMU_ODR_HZ          833
#define IMU_I2C_ADDR        0x6A    // LSM6DS3TR-C na Wire1 (SA0 = 0)
#define IMU_I2C_HZ          400000

// ---------------------------------------------------------------- flash
#define FLASH_SLOT_SIZE     (128 * 1024)  // wielokrotność bloku kasowania 64 kB
#define FLASH_NUM_SLOTS     16            // 16 × 128 kB = 2 MB
#define FLASH_ERASE_BLOCK   (64 * 1024)
#define FLASH_SLOT_HDR_SIZE 256           // nagłówek slotu = 1 strona
#define GET_BLOCK_MAX       4096          // [B] maks. rozmiar bloku przy pobieraniu
#define SAVE_CHUNK_BYTES    1024          // [B] zapis na pętlę główną w SAVING

// ---------------------------------------------------------------- prędkościomierz
// Te same wartości domyślne co tools/config.yaml i ustawienia aplikacji.
#define FINAL_DRIVE_RATIO       5.0f    // DO UZUPEŁNIENIA
#define FRONT_TIRE_DIAMETER_MM  64.0f   // DO UZUPEŁNIENIA, kalibrowane
// Mapowanie osi płytki na osie auta: indeks osi płytki (0=x,1=y,2=z) i znak
#define IMU_AXIS_FORWARD        0
#define IMU_SIGN_FORWARD        (+1)
#define IMU_AXIS_LEFT           1
#define IMU_SIGN_LEFT           (+1)
#define IMU_AXIS_UP             2
#define IMU_SIGN_UP             (+1)
#define IMU_YAW_OFFSET_DEG      0.0f    // [deg] kąt montażu płytki w poziomie
#define ZUPT_MS                 200     // [ms] czas bezruchu do zerowania
#define ZUPT_GYRO_DPS           4.0f    // [dps] maks. |ω| (po odjęciu biasu) w bezruchu
#define ZUPT_ACC_G              0.04f   // [g] maks. ||a| − 1 g| w bezruchu
#define SLIP_ACC_THR            3.0f    // [m/s²] |a_front − a_long| = brak buksowania
#define SLIP_V_THR              0.5f    // [m/s] |v_front − v| = brak buksowania
#define GRIP_HOLD_MS            100     // [ms] jak długo warunek przyspieszeń → korekta
#define WHEEL_CORR_TAU_S        0.5f    // [s] stała czasowa ściągania do v_front
#define BRAKE_THROTTLE_PCT      (-10)   // [%] poniżej = strefa hamowania
#define EST_AFRONT_TAU_S        0.03f   // [s] wygładzanie a_front / a_long w estymatorze
#define LIVE_RATE_HZ            20

// ---------------------------------------------------------------- BLE
#define BLE_NAME            "X4F-LOG"
#define BLE_MTU             247

// ---------------------------------------------------------------- format
#define RUN_FORMAT_VERSION  1
#define PROTOCOL_VERSION    1

// ---------------------------------------------------------------- debug
// Komendy SIM_* (syntetyczne zbocza/gaz) dostępne tylko przez USB.
#define ENABLE_SIM_COMMANDS 1
