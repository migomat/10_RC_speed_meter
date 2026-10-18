#include "ble_link.h"

#include <Arduino.h>
#include <bluefruit.h>
#include "config.h"
#include "crc32.h"
#include "flash_store.h"
#include "hall.h"
#include "imu.h"
#include "logger.h"
#include "run_format.h"
#include "sim.h"
#include "throttle.h"
#include "timebase.h"

LiveData g_live;

// ------------------------------------------------------------ NUS (własna
// instancja zamiast BLEUart: każda notyfikacja musi zaczynać się bajtem typu,
// a BLEUart::write() dzieli dane po swojemu).
static BLEService        s_nus(BLEUART_UUID_SERVICE);
static BLECharacteristic s_txd(BLEUART_UUID_CHR_TXD);
static BLECharacteristic s_rxd(BLEUART_UUID_CHR_RXD);
static volatile uint16_t s_conn = BLE_CONN_HANDLE_INVALID;

#define RX_RING 512
static volatile uint8_t  s_rx[RX_RING];
static volatile uint16_t s_rx_head, s_rx_tail;
static bool s_vmax_reset;
static volatile uint16_t s_mtu = 23;

static void rxd_cb(uint16_t conn, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn; (void)chr;
  for (uint16_t i = 0; i < len; i++) {
    uint16_t nh = (s_rx_head + 1) % RX_RING;
    if (nh == s_rx_tail) break;  // przepełnienie – reszta odrzucona
    s_rx[s_rx_head] = data[i];
    s_rx_head = nh;
  }
}

// BLEConnection::getMtu() potrafi zwrócić MTU zgłoszone przez klienta (np. 517
// z Chrome/BlueZ), a nie wynegocjowane min(klient, BLE_MTU). Bez ograniczenia
// pakiety były za długie (NRF_ERROR_DATA_SIZE) i przepełniały bufor.
static uint16_t clamp_mtu(uint16_t m) {
  if (m > BLE_MTU) m = BLE_MTU;
  if (m < 23) m = 23;
  return m;
}

static void connect_cb(uint16_t conn) {
  s_conn = conn;
  BLEConnection* c = Bluefruit.Connection(conn);
  if (c) {
    s_mtu = clamp_mtu(c->getMtu());
    c->requestPHY(BLE_GAP_PHY_2MBPS);
    c->requestDataLengthUpdate();
    c->requestMtuExchange(BLE_MTU);
    c->requestConnectionParameter(6);  // 7,5 ms
  }
}

static void disconnect_cb(uint16_t conn, uint8_t reason) {
  (void)conn; (void)reason;
  s_conn = BLE_CONN_HANDLE_INVALID;
  s_mtu = 23;
}

bool ble_begin(void) {
  Bluefruit.configPrphConn(BLE_MTU, 100, 8, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
  if (!Bluefruit.begin(1, 0)) return false;
  Bluefruit.setName(BLE_NAME);
  Bluefruit.setTxPower(4);
  Bluefruit.autoConnLed(false);
  Bluefruit.Periph.setConnectCallback(connect_cb);
  Bluefruit.Periph.setDisconnectCallback(disconnect_cb);
  Bluefruit.Periph.setConnIntervalMS(7, 15);

  s_nus.begin();
  s_txd.setProperties(CHR_PROPS_NOTIFY);
  s_txd.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  s_txd.setMaxLen(BLE_MTU - 3);
  s_txd.begin();
  s_rxd.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  s_rxd.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  s_rxd.setMaxLen(BLE_MTU - 3);
  s_rxd.setWriteCallback(rxd_cb);
  s_rxd.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(s_nus);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
  return true;
}

bool ble_connected(void) { return s_conn != BLE_CONN_HANDLE_INVALID; }

bool vmax_reset_requested(void) {
  bool r = s_vmax_reset;
  s_vmax_reset = false;
  return r;
}

// ------------------------------------------------------------ łącza
struct Link {
  bool is_ble;
  bool live;
  char line[160];
  uint16_t n;
};
static Link s_usb = {false, false, {0}, 0};
static Link s_ble = {true, false, {0}, 0};

static uint32_t s_tx_pkts, s_tx_busy_waits, s_tx_errors, s_tx_last_err, s_tx_last_len;

// Notyfikacje wprost przez SoftDevice (sd_ble_gatts_hvx). BLECharacteristic::
// notify() czeka na semafor w obiekcie BLEConnection, który przy rozłączeniu
// jest zwalniany – wyścig kończył się zawieszeniem. Tu przy braku bufora
// (NRF_ERROR_RESOURCES) oddajemy procesor i sprawdzamy, czy połączenie trwa.
static void ble_send_typed(uint8_t type, const uint8_t* data, uint32_t len) {
  if (!ble_connected() || !s_txd.notifyEnabled(s_conn)) return;
  uint16_t payload = clamp_mtu(s_mtu) - 3 - 1;
  static uint8_t pkt[BLE_MTU];  // statycznie – stos zadania loop ma tylko 4 KB
  uint16_t handle = s_txd.handles().value_handle;
  while (len) {
    uint16_t n = len < payload ? len : payload;
    pkt[0] = type;
    memcpy(pkt + 1, data, n);
    uint16_t plen = n + 1;
    ble_gatts_hvx_params_t hvx = {};
    hvx.handle = handle;
    hvx.type = BLE_GATT_HVX_NOTIFICATION;
    hvx.p_len = &plen;
    hvx.p_data = pkt;
    uint32_t t0 = millis();
    for (;;) {
      uint16_t conn = s_conn;
      if (conn == BLE_CONN_HANDLE_INVALID) return;
      plen = n + 1;  // SoftDevice nadpisuje *p_len
      uint32_t err = sd_ble_gatts_hvx(conn, &hvx);
      if (err == NRF_SUCCESS) break;
      if (err != NRF_ERROR_RESOURCES || millis() - t0 > 1000) {
        s_tx_errors++;
        s_tx_last_err = err;
        s_tx_last_len = (uint32_t)(n + 1) | ((uint32_t)s_mtu << 16);
        return;
      }
      s_tx_busy_waits++;
      delay(1);  // kolejka HVN pełna – czekamy na wysłanie (zadanie BLE)
    }
    s_tx_pkts++;
    data += n;
    len -= n;
  }
}

static void send_text(Link& l, const char* s) {
  size_t n = strlen(s);
  if (l.is_ble) {
    ble_send_typed(FRAME_TEXT, (const uint8_t*)s, n);
    ble_send_typed(FRAME_TEXT, (const uint8_t*)"\n", 1);
  } else if (Serial) {
    Serial.write(s, n);
    Serial.write('\n');
  }
}

static void send_binary(Link& l, const uint8_t* d, uint32_t n) {
  if (l.is_ble) {
    ble_send_typed(FRAME_BINARY, d, n);
  } else if (Serial) {
    Serial.write((uint8_t)FRAME_BINARY);
    Serial.write(d, n);
  }
}

static void send_err(Link& l, const char* err) {
  char b[64];
  snprintf(b, sizeof(b), "{\"err\":\"%s\"}", err);
  send_text(l, b);
}

// ------------------------------------------------------------ komendy
static char s_out[1600];

static void cmd_info(Link& l) {
  snprintf(s_out, sizeof(s_out),
           "{\"fw\":\"%s\",\"format\":%d,\"proto\":%d,\"hall_lines\":%d,\"pole_pairs\":%d,"
           "\"edges_per_rev\":%d,\"slot_size\":%lu,\"slots\":%d,\"sample_rate\":%d,"
           "\"get_block_max\":%d,\"max_samples\":%d,\"max_edges\":%lu,\"jedec\":\"%06lX\","
           "\"name\":\"%s\"}",
           FW_VERSION_STR, RUN_FORMAT_VERSION, PROTOCOL_VERSION, HALL_LINES, MOTOR_POLE_PAIRS,
           EDGES_PER_REV, (unsigned long)FLASH_SLOT_SIZE, FLASH_NUM_SLOTS, SAMPLE_RATE_HZ,
           GET_BLOCK_MAX, MAX_SAMPLES, (unsigned long)EDGE_LOG_LEN,
           (unsigned long)flash_jedec_id(), BLE_NAME);
  send_text(l, s_out);
}

static void cmd_status(Link& l) {
  uint32_t now = timebase_now();
  snprintf(s_out, sizeof(s_out),
           "{\"state\":\"%s\",\"neutral_us\":%u,\"neutral_learned\":%s,\"throttle_ok\":%s,"
           "\"throttle_us\":%u,\"runs\":%d,\"free_slots\":%d,\"overwritten_unread\":%lu,"
           "\"time_set\":%s,\"ram_run\":%s,\"imu_ok\":%s,\"flash_ok\":%s,\"imu_overruns\":%lu,"
           "\"imu_samples\":%lu,\"imu_misaligned\":%lu,\"hall_edges\":%lu,\"hall_glitches\":%lu,\"uptime_ms\":%lu,"
           "\"boot\":%lu,\"last_save_ms\":%lu,\"last_crc_ms\":%lu,\"connected\":%s,\"sim\":%s,"
           "\"mtu\":%u,\"ble_pkts\":%lu,\"ble_waits\":%lu,\"ble_err\":%lu,"
           "\"reset_reason\":%lu,\"stack_free\":%lu,\"tick_overflows\":%lu,\"ble_last_err\":%lu,"
           "\"ble_last_len\":%lu,\"erased_slots\":%d,\"erasing\":%s}",
           logger_state_name(), throttle_neutral_us(), throttle_neutral_learned() ? "true" : "false",
           throttle_signal_ok(now) ? "true" : "false", throttle_tick_width(now),
           flash_count_runs() + (logger_ram_run_available() ? 1 : 0), flash_count_free(),
           (unsigned long)flash_overwritten_unread(), logger_time_set() ? "true" : "false",
           logger_ram_run_available() ? "true" : "false", imu_ok() ? "true" : "false",
           flash_ok() ? "true" : "false", (unsigned long)imu_overrun_count(),
           (unsigned long)imu_sample_count(), (unsigned long)imu_misaligned_count(),
           (unsigned long)hall_edge_count(),
           (unsigned long)hall_glitch_count(), (unsigned long)millis(),
           (unsigned long)flash_boot_count(), (unsigned long)logger_last_save_ms(),
           (unsigned long)logger_last_crc_ms(), ble_connected() ? "true" : "false",
           sim_active() ? "true" : "false", s_mtu, (unsigned long)s_tx_pkts,
           (unsigned long)s_tx_busy_waits, (unsigned long)s_tx_errors,
           (unsigned long)readResetReason(), (unsigned long)g_stack_free_min,
           (unsigned long)g_tick_overflows, (unsigned long)s_tx_last_err,
           (unsigned long)s_tx_last_len, flash_count_erased(), flash_busy() ? "true" : "false");
  send_text(l, s_out);
}

static void cmd_list(Link& l) {
  int idx[FLASH_NUM_SLOTS];
  int n = flash_sorted(idx, FLASH_NUM_SLOTS);
  size_t p = snprintf(s_out, sizeof(s_out), "{\"runs\":[");
  bool first = true;
  if (logger_ram_run_available()) {
    const RunHeader* h = logger_ram_run_header();
    p += snprintf(s_out + p, sizeof(s_out) - p,
                  "{\"seq\":%u,\"len\":%lu,\"crc\":%lu,\"unix\":%lu,\"boot\":%lu,\"uptime_ms\":%lu,"
                  "\"ram\":true}",
                  RAM_RUN_SEQ, (unsigned long)logger_ram_run_len(),
                  (unsigned long)logger_ram_run_crc(), (unsigned long)h->unix_s,
                  (unsigned long)h->boot_count, (unsigned long)h->uptime_ms);
    first = false;
  }
  for (int i = 0; i < n && p < sizeof(s_out) - 120; i++) {
    const SlotInfo* s = flash_slot(idx[i]);
    p += snprintf(s_out + p, sizeof(s_out) - p,
                  "%s{\"seq\":%lu,\"len\":%lu,\"crc\":%lu,\"unix\":%lu,\"boot\":%lu,\"uptime_ms\":%lu}",
                  first ? "" : ",", (unsigned long)s->seq, (unsigned long)s->len,
                  (unsigned long)s->crc, (unsigned long)s->unix_s, (unsigned long)s->boot_count,
                  (unsigned long)s->uptime_ms);
    first = false;
  }
  snprintf(s_out + p, sizeof(s_out) - p, "]}");
  send_text(l, s_out);
}

static void cmd_get(Link& l, uint32_t seq, uint32_t off, uint32_t len) {
  static uint8_t frame[sizeof(BlockHeader) + GET_BLOCK_MAX] __attribute__((aligned(4)));
  if (logger_busy()) return send_err(l, "BUSY");
  if (len == 0 || len > GET_BLOCK_MAX) return send_err(l, "BAD_LEN");
  uint8_t* data = frame + sizeof(BlockHeader);
  bool ok;
  uint32_t total;
  if (seq == RAM_RUN_SEQ && logger_ram_run_available()) {
    total = logger_ram_run_len();
    if (off >= total) return send_err(l, "BAD_OFFSET");
    if (off + len > total) len = total - off;
    ok = logger_read_ram_run(off, data, len);
  } else {
    int slot = flash_find(seq);
    if (slot < 0) return send_err(l, "NOT_FOUND");
    total = flash_slot(slot)->len;
    if (off >= total) return send_err(l, "BAD_OFFSET");
    if (off + len > total) len = total - off;
    ok = flash_read(slot, off, data, len);
  }
  if (!ok) return send_err(l, "READ");
  BlockHeader bh = {BLOCK_MAGIC, seq, off, len, crc32(data, len)};
  memcpy(frame, &bh, sizeof(bh));
  send_binary(l, frame, sizeof(BlockHeader) + len);
}

static void cmd_del(Link& l, uint32_t seq) {
  if (logger_busy()) return send_err(l, "BUSY");
  bool ok = false;
  if (seq == RAM_RUN_SEQ && logger_ram_run_available()) {
    logger_discard_ram_run();
    ok = true;
  } else {
    ok = flash_mark_downloaded(seq);
  }
  if (!ok) {
    snprintf(s_out, sizeof(s_out), "{\"err\":\"NOT_FOUND\",\"seq\":%lu}", (unsigned long)seq);
    return send_text(l, s_out);
  }
  snprintf(s_out, sizeof(s_out), "OK DEL %lu", (unsigned long)seq);
  send_text(l, s_out);
}

static void send_live(Link& l) {
  snprintf(s_out, sizeof(s_out),
           "{\"live\":1,\"v\":%.3f,\"v_max\":%.3f,\"v_front\":%.3f,\"slip\":%d,\"rpm\":%.0f,"
           "\"thr\":%u,\"thr_pct\":%.1f,\"ax\":%.3f,\"zupt\":%d,\"state\":\"%s\"}",
           g_live.v, g_live.v_max, g_live.v_front, g_live.spinning ? 1 : 0, g_live.rpm,
           g_live.throttle_us, g_live.throttle_pct, g_live.ax_g, g_live.zupt ? 1 : 0,
           logger_state_name());
  send_text(l, s_out);
}

static void handle_line(Link& l, char* line) {
  char* argv[6];
  int argc = 0;
  for (char* tok = strtok(line, " \t"); tok && argc < 6; tok = strtok(nullptr, " \t")) argv[argc++] = tok;
  if (argc == 0) return;
  const char* c = argv[0];
  auto U = [&](int i) -> uint32_t { return i < argc ? strtoul(argv[i], nullptr, 10) : 0; };

  if (!strcmp(c, "INFO")) return cmd_info(l);
  if (!strcmp(c, "STATUS")) return cmd_status(l);
  if (!strcmp(c, "LIST")) return cmd_list(l);
  if (!strcmp(c, "TIME") && argc >= 2) {
    logger_set_time(U(1));
    return send_text(l, "OK");
  }
  if (!strcmp(c, "GET") && argc >= 4) return cmd_get(l, U(1), U(2), U(3));
  if (!strcmp(c, "DEL") && argc >= 2) return cmd_del(l, U(1));
  if (!strcmp(c, "ARM")) {
    logger_arm();
    return send_text(l, "OK");
  }
  if (!strcmp(c, "LIVE") && argc >= 2) {
    l.live = U(1) != 0;
    return send_text(l, "OK");
  }
  if (!strcmp(c, "VMAX") && argc >= 2 && !strcmp(argv[1], "RESET")) {
    s_vmax_reset = true;
    return send_text(l, "OK");
  }
#if ENABLE_SIM_COMMANDS
  if (!strcmp(c, "SIM") && argc >= 2 && !l.is_ble) {
    const char* s = argv[1];
    if (!strcmp(s, "RPM")) sim_set_rpm(atof(argc > 2 ? argv[2] : "0"));
    else if (!strcmp(s, "RAMP")) sim_ramp(atof(argc > 2 ? argv[2] : "0"), U(3));
    else if (!strcmp(s, "THR")) sim_set_throttle((uint16_t)U(2));
    else if (!strcmp(s, "TRIG")) logger_force_trigger();
    else if (!strcmp(s, "GLITCH")) sim_glitch(argc > 2 ? U(2) : 10);
    else if (!strcmp(s, "OFF")) sim_off();
    else if (!strcmp(s, "RESETAT")) g_flash_test_reset_at = U(2);
    else return send_err(l, "BAD_SIM");
    logger_set_sim_flag(true);
    return send_text(l, "OK");
  }
  if (!strcmp(c, "REBOOT") && !l.is_ble) {
    send_text(l, "OK");
    Serial.flush();
    delay(50);
    NVIC_SystemReset();
  }
  if (!strcmp(c, "DBG")) {
    ImuRaw r;
    imu_get_at(timebase_now(), &r);
    snprintf(s_out, sizeof(s_out),
             "{\"ax\":%d,\"ay\":%d,\"az\":%d,\"gx\":%d,\"gy\":%d,\"gz\":%d,\"v\":%.3f,"
             "\"v_front\":%.3f,\"rpm\":%.1f,\"sim_rpm\":%.1f,\"thr\":%u,\"zupt\":%d,"
             "\"thr_pulses\":%lu,\"thr_rejects\":%lu,\"t_us\":%lu}",
             r.ax, r.ay, r.az, r.gx, r.gy, r.gz, g_live.v, g_live.v_front, g_live.rpm, sim_rpm(),
             g_live.throttle_us, g_live.zupt ? 1 : 0, (unsigned long)throttle_pulse_count(),
             (unsigned long)throttle_reject_count(), (unsigned long)timebase_now());
    return send_text(l, s_out);
  }
#endif
  send_err(l, "UNKNOWN");
}

static void feed(Link& l, char ch) {
  if (ch == '\r') return;
  if (ch == '\n') {
    l.line[l.n] = 0;
    handle_line(l, l.line);
    l.n = 0;
    return;
  }
  if (l.n < sizeof(l.line) - 1) l.line[l.n++] = ch;
}

void links_poll(uint32_t now_ms) {
  int budget = 256;
  while (Serial.available() && budget--) feed(s_usb, (char)Serial.read());
  while (s_rx_tail != s_rx_head) {
    char ch = (char)s_rx[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1) % RX_RING;
    feed(s_ble, ch);
  }
  if (!ble_connected()) {
    s_ble.live = false;
  } else {
    // MTU po wymianie (zadanie BLE aktualizuje obiekt połączenia)
    BLEConnection* c = Bluefruit.Connection(s_conn);
    if (c) s_mtu = clamp_mtu(c->getMtu());
  }

  static uint32_t last_live;
  if ((s_usb.live || s_ble.live) && now_ms - last_live >= 1000 / LIVE_RATE_HZ) {
    last_live = now_ms;
    if (s_usb.live) send_live(s_usb);
    if (s_ble.live) send_live(s_ble);
  }
}
