# Protokół X4F-LOG (wersja 1)

Transport BLE: Nordic UART Service (`6e400001-…`), RX `6e400002` (zapis, telefon → Xiao),
TX `6e400003` (notyfikacje). Nazwa rozgłoszeniowa `X4F-LOG`. MTU do 247 (payload 244).
Te same komendy działają przez USB Serial (115200, CDC).

## Ramkowanie Xiao → host

* **BLE**: pierwszy bajt każdej notyfikacji to typ: `0x01` fragment tekstu (linie
  zakończone `\n`), `0x02` fragment ramki binarnej. Ramka binarna może zajmować wiele
  notyfikacji; jej długość wynika z nagłówka bloku.
* **USB**: tekst wysyłany wprost (linie `\n`); ramka binarna poprzedzona jednym bajtem
  `0x02`. Linie tekstu nigdy nie zaczynają się od `0x02`.

Ramka binarna (odpowiedź na `GET`):

```
u32 magic 0x42463458 ("X4FB") | u32 seq | u32 offset | u32 len | u32 crc32(dane) | dane[len]
```

## Komendy (tekst zakończony `\n`)

| Komenda | Odpowiedź |
|---|---|
| `INFO` | `{"fw","format","proto","hall_lines","pole_pairs","edges_per_rev","slot_size","slots","sample_rate","get_block_max","max_samples","max_edges","jedec","name"}` |
| `STATUS` | `{"state","neutral_us","neutral_learned","throttle_ok","throttle_us","runs","free_slots","overwritten_unread","time_set","ram_run","imu_ok","flash_ok","imu_overruns",…,"ble_dropped","hall_pins","armed"}` – `hall_pins`: bieżący stan pinów halla (bity A\|B<<1\|C<<2, diagnostyka okablowania), `armed`: postój zaliczony, wyzwolenie możliwe |
| `TIME <unix_s>` | `OK` |
| `LIST` | `{"runs":[{"seq","len","crc","unix","boot","uptime_ms"[,"ram":true]}…]}` od najstarszego |
| `GET <seq> <offset> <len>` | ramka binarna; `len ≤ 4096` |
| `DEL <seq>` | `OK DEL <seq>` albo `{"err":"NOT_FOUND","seq":…}` |
| `ARM` | `OK` – powrót z ERROR do IDLE |
| `LIVE 1` / `LIVE 0` | `OK`; potem 20 Hz: `{"live":1,"v","v_max","v_front","slip","rpm","thr","thr_pct","ax","zupt","state"}` |
| `VMAX RESET` | `OK` |

Tylko USB (testy na biurku): `SIM RPM <rpm>`, `SIM RAMP <rpm> <ms>`, `SIM THR <us>`,
`SIM TRIG`, `SIM GLITCH <us>`, `SIM OFF`, `SIM RESETAT <offset>` (reset w trakcie
zapisu – test zaniku zasilania), `REBOOT`, `DBG`.

Błędy: `{"err":"…"}`. W trakcie RECORDING/SAVING `GET` i `DEL` zwracają `{"err":"BUSY"}`.

## Pobieranie

1. `TIME`, `STATUS`, `LIST`.
2. Dla każdego startu: `GET` kolejnych bloków (CRC bloku, do 3 ponowień), na końcu
   CRC całości (z polem `crc32` nagłówka liczonym jako 0) porównane z `crc` z `LIST`.
3. Zapis u odbiorcy (IndexedDB `oncomplete` / plik na dysku), dopiero potem `DEL`.
4. Rozłączenie niczego nie kasuje; pobieranie wznawia się od ostatniego poprawnego offsetu.

`seq 0` = start, który został tylko w RAM po błędzie zapisu flash (do kolejnego startu).
