// Nagrywanie startu w RAM (pre-trigger w buforze kołowym) i zapis do flash.
//   IDLE → (gaz > próg) → RECORDING → (POSTTRIGGER_MS) → SAVING → IDLE
// Błąd zapisu flash → ERROR; start zostaje w RAM jako seq RAM_RUN_SEQ.
#pragma once

#include <stdint.h>
#include "run_format.h"

enum LogState : uint8_t { LS_IDLE, LS_RECORDING, LS_SAVING, LS_ERROR };

void        logger_begin(void);
void        logger_push(const Sample& s, float throttle_pct, bool throttle_valid);
void        logger_poll(void);            // postęp SAVING (pętla główna)
LogState    logger_state(void);
const char* logger_state_name(void);
bool        logger_busy(void);            // RECORDING albo SAVING
bool        logger_armed(void);           // postój zaliczony – wyzwolenie możliwe
void        logger_arm(void);             // ERROR → IDLE
void        logger_force_trigger(void);   // SIM / testy
void        logger_set_sim_flag(bool on);
void        logger_set_error(void);       // np. brak IMU przy starcie

// Start trzymany tylko w RAM (po błędzie zapisu)
bool        logger_ram_run_available(void);
uint32_t    logger_ram_run_len(void);
uint32_t    logger_ram_run_crc(void);
const RunHeader* logger_ram_run_header(void);
bool        logger_read_ram_run(uint32_t offset, void* buf, uint32_t len);
void        logger_discard_ram_run(void);

uint32_t    logger_last_save_ms(void);    // czas ostatniego SAVING [ms]
uint32_t    logger_last_crc_ms(void);
uint32_t    logger_runs_recorded(void);

// Czas rzeczywisty (komenda TIME)
void        logger_set_time(uint32_t unix_s);
bool        logger_time_set(void);
uint32_t    logger_unix_now(void);

// CRC startu: CRC32 całości z polem RunHeader.crc32 traktowanym jako 0.
uint32_t    run_crc_update_chunk(uint32_t crc, uint32_t offset, const uint8_t* data, uint32_t len);
