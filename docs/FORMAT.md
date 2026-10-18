# Format binarny startu (wersja 1)

Wspólny dla flash, BLE i plików `.bin`. Little-endian. Źródło prawdy:
`firmware/src/run_format.h`; dekodery: `app/js/format.js`, `tools/rc_launch/parse.py`.
Zmiana formatu = podbicie `RUN_FORMAT_VERSION` we wszystkich trzech miejscach.

```
[RunHeader 128 B][Sample × n_samples (28 B)][EdgeWord × n_edges (4 B)]
```

## RunHeader (128 B)

| off | typ | pole | opis |
|---|---|---|---|
| 0 | u32 | magic | `0x52463458` („X4FR”) |
| 4 | u16 | format_version | 1 |
| 6 | u16 | header_size | 128 |
| 8 | u32 | seq | numer startu (rosnący; 0 = start tylko w RAM) |
| 12 | u32 | unix_s | czas wyzwolenia, 0 = nieznany |
| 16 | u32 | boot_count | numer rozruchu Xiao |
| 20 | u32 | uptime_ms | czas od włączenia w chwili wyzwolenia |
| 24 | u8 | hall_lines | 1 albo 3 |
| 25 | u8 | motor_pole_pairs | |
| 26 | u8 | edges_per_rev | 2·hall_lines·pole_pairs |
| 27 | u8 | throttle_fwd_higher | 1: „do przodu” = dłuższy impuls |
| 28 | u16 | sample_rate_hz | 500 |
| 30 | u16 | imu_odr_hz | 833 |
| 32 | u16 | accel_range_g | 4 |
| 34 | u16 | gyro_range_dps | 500 |
| 36 | f32 | accel_scale_g | g/LSB (0,122e-3) |
| 40 | f32 | gyro_scale_dps | dps/LSB (17,5e-3) |
| 44 | u16 | throttle_neutral_us | 0 = nienauczone |
| 46 | u16 | throttle_full_us | µs od neutrum do 100 % |
| 48 | u32 | trigger_index | indeks próbki wyzwolenia |
| 52 | u32 | n_samples | |
| 56 | u32 | n_edges | |
| 60 | u32 | flags | patrz niżej |
| 64 | u32 | t0_us | t_us pierwszej próbki (baza czasu zboczy) |
| 68 | u32 | trigger_t_us | t_us próbki wyzwolenia |
| 72 | u16 | sample_size | 28 |
| 74 | u16 | edge_size | 4 |
| 76 | u32 | fw_version | major<<16 \| minor<<8 \| patch |
| 80 | u16 | hall_min_edge_us | |
| 82 | u16 | pretrigger_ms | |
| 84 | u16 | posttrigger_ms | |
| 86 | u8 | trigger_pct | |
| 87 | u8 | hall_pullup_v_x10 | informacyjnie (konfiguracja) |
| 88 | u32 | hall_timeout_us | |
| 92 | 32 B | reserved | zera |
| 124 | u32 | crc32 | CRC32 IEEE całego startu z tym polem liczonym jako 0 |

Flagi startu: `0x01` zbocza obcięte (bufor pełny), `0x02` czas ustawiony,
`0x04` neutrum nauczone, `0x08` błąd IMU, `0x10` start tylko w RAM (błąd flash),
`0x20` dane syntetyczne (komendy SIM / wektory testowe).

## Sample (28 B, 500 Hz)

```c
uint32_t t_us;          // licznik 1 MHz (TIMER4), zawija się co ~71,6 min – odejmować jako uint32
uint32_t hall_edges;    // licznik zboczy (narastający)
uint32_t period_rev_us; // okres pełnego obrotu silnika, 0 = brak/postój/pierwsze EDGES_PER_REV zboczy
uint16_t throttle_us;   // 0 = brak sygnału (> 100 ms)
int16_t  ax, ay, az;    // surowe LSB, osie płytki
int16_t  gx, gy, gz;
uint8_t  flags;         // 0x01 hall_valid, 0x02 throttle_valid, 0x04 kierunek A→B→C,
                        // 0x08 kierunek znany, 0x10 przepełnienie FIFO IMU,
                        // 0x20 brak świeżej próbki IMU, 0x40 próbka wyzwolenia
uint8_t  reserved;
```

## EdgeWord (4 B)

Bity 0–28: czas zbocza względem `t0_us` w µs (modulo 2²⁹, okno nagrania ≪ 536 s),
bit 29: poziom linii po zboczu, bity 30–31: linia (0 = A, 1 = B, 2 = C).

## Nagłówek slotu we flash (tylko na urządzeniu)

Slot 128 kB; strona 0 = nagłówek, każde pole w osobnym słowie 32-bit
(programowane niezależnie, przejścia stanów tylko 1→0):
`+0 magic "X4FS"`, `+4 seq`, `+8 stan` (bajt niski: FF wolny, FE zapis, FC kompletny,
F8 pobrany), `+12 długość`, `+16 CRC` (= crc32 startu). Dane startu od offsetu 256.
