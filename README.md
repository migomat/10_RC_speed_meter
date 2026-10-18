# X4F Launch Logger – faza 1

Logger startów samochodu RC Xray X4F (napęd na przód) na XIAO nRF52840 Sense.
Urządzenie tylko podsłuchuje czujnik halla silnika i sygnał gazu, nagrywa IMU
i zapisuje starty we flash. Pobieranie odbywa się przez BLE do aplikacji PWA na Androidzie.
Specyfikacja: `CLAUDE.md`. Okablowanie: `docs/WIRING.md`. Formaty: `docs/FORMAT.md`, `docs/PROTOCOL.md`.

```
firmware/      szkic Arduino (kod w firmware/src), rdzeń Seeed nRF52 (non-mbed)
app/           PWA (czysty JS, uPlot lokalnie), testy: node --test app/test/
tools/         rc_launch (Python): download/parse/analyze/plot/compare, testy: pytest
test_vectors/  wspólne wektory testowe (generator: rc_launch.make_test_vectors)
hall_sim/      symulator halla/gazu na ESP32 (opcjonalny)
docs/          format binarny i protokół
```

## Szybki start

```bash
# narzędzia (raz)
arduino-cli config add board_manager.additional_urls https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
arduino-cli core install Seeeduino:nrf52          # testowane: 1.1.13
arduino-cli lib install "Adafruit SPIFlash"       # testowane: 5.1.1
uv tool install adafruit-nrfutil                  # upload przez bootloader
uv venv .venv && uv pip install --python .venv -r <(echo bleak numpy scipy matplotlib pyyaml pyserial pytest)

# firmware (tryb HALL_LINES=1; tryb 3: dodać oba --build-property)
arduino-cli compile -b Seeeduino:nrf52:xiaonRF52840Sense -u -p /dev/ttyACM0 firmware
#   --build-property "compiler.cpp.extra_flags=-DHALL_LINES=3" --build-property "compiler.c.extra_flags=-DHALL_LINES=3"

# testy
cd tools && ../.venv/bin/python -m pytest -q          # analiza + estymator C vs Python
node --test app/test/                                 # analysis.js vs wektory
.venv/bin/python tools/fw_test.py                     # testy na płytce (USB, komendy SIM)

# pobieranie z PC
cd tools && ../.venv/bin/python -m rc_launch.download          # BLE, kasuje po zapisie
../.venv/bin/python -m rc_launch.download --usb --keep         # USB, bez kasowania
../.venv/bin/python -m rc_launch.plot runs/*.bin --range 1
../.venv/bin/python -m rc_launch.compare runs/*.bin
```

Jeśli płytka przestanie odpowiadać na USB, dwukrotne wciśnięcie RESET uruchamia
bootloader. Firmware wgrywa się wtedy normalnie.

## Aplikacja (PWA) – wdrożenie na GitHub Pages

Web Bluetooth wymaga HTTPS, a localhost jest traktowany jako bezpieczny. Workflow
`.github/workflows/pages.yml` publikuje katalog `app/` przy każdym pushu na `main`:
w ustawieniach repozytorium wybierz *Settings → Pages → Source: GitHub Actions*.
Na telefonie otwórz `https://<użytkownik>.github.io/<repo>/` w Chrome, a następnie
*menu → Dodaj do ekranu głównego*. Po pierwszym otwarciu aplikacja działa offline
dzięki service workerowi. Po każdej zmianie plików aplikacji podbij `CACHE` w `app/sw.js`.

Test lokalny: `python3 -m http.server 8765 --directory app`, potem
`http://localhost:8765/#demo`, czyli tryb demo z wirtualnym Xiao.

## Ustalenia sprzętowe (sprawdzone w źródłach i na płytce)

| Temat | Wniosek |
|---|---|
| `micros()` | `DWT->CYCCNT/64`, gdy DWT włączony, inaczej tick RTOS (1 ms). Nie używać: zawija się po ~67 s i stoi w uśpieniu CPU. Zamiast tego **TIMER4** 32-bit 1 MHz. |
| TIMERy | TIMER0 – SoftDevice; TIMER2 – rdzeń czyta `CC[0]` (wersja bootloadera) w `init()`; TIMER1/3/4 wolne. Wybrany TIMER4 (6 rejestrów CC: 3× hall, gaz, takt 500 Hz, „teraz”). |
| Zegar | Po włączeniu SoftDevice wymuszany HFXO (`sd_clock_hfclk_request`), bo HFINT ma błąd ±1–2 %. |
| Przechwytywanie zboczy | GPIOTE (kanał przydzielony przez `attachInterrupt`) → PPI (kanały 0–3, przez `sd_ppi_*`) → `TIMER4.CAPTURE`. Znacznik czasu bez jittera przerwań; ISR tylko odczytuje CC i poziom pinu. |
| IMU | LSM6DS3TR-C na **Wire1** (P0.07 SDA, P0.27 SCL), adres 0x6A, WHO_AM_I = 0x6A. Zasilanie z pinu P1.08 (`PIN_LSM6DS3TR_C_POWER`) – **wymaga trybu H0H1**. Biblioteka Seeed na rdzeniu non-mbed wybiera zły bus (`TARGET_SEEED_…` nie jest zdefiniowane) – konfiguracja rejestrami. |
| I2C | Biblioteka `Wire` robi powtórzony start programowo. Przy aktywnym SoftDevice ~1 % odczytów FIFO było przesuniętych o 1–3 słowa (zmierzone: 0 błędów bez BLE, ~9/s z BLE). Własny sterownik TWIM1 z `SHORTS` LASTTX→STARTRX→STOP: 0 błędów. |
| Flash | Puya **P25Q16H** (JEDEC 85 60 15), 2 MB, QSPI. Nie ma go na domyślnej liście `Adafruit_SPIFlash` z tym rdzeniem – deskryptor przekazywany jawnie. Na wszelki wypadek komenda 0xAB (wyjście z deep power-down) przy starcie. Kasowanie bloku 64 kB: **~17 ms**, komenda wraca po ~49 µs (nieblokujące, odpytywanie WIP). Zapis 1 kB ≈ 7 ms. Ponowne programowanie słowa (1→0) działa. |
| BLE | `BLEConnection::getMtu()` zwraca MTU zgłoszone przez klienta (517), a nie wynegocjowane – ograniczane do 247. Notyfikacje przez `sd_ble_gatts_hvx` (semafor w `BLECharacteristic::notify` ścigał się z rozłączeniem). |
| Stos | Zadanie `loop` ma tylko 4 KB (`LOOP_STACK_SZ`), a hook przepełnienia przy `CFG_DEBUG=0` nic nie robi. Duże bufory są statyczne, zapas stosu raportowany w `STATUS.stack_free` (zmierzone ≥ 2,3 KB). HardFault → reset zamiast zawieszenia. |
| LED | RGB aktywne niskim (mimo `LED_STATE_ON 1` w variant.h). |

## Pomiary (płytka bez auta, SIM przez USB, laptop BT 4.2)

| Wielkość | Wynik | Cel |
|---|---|---|
| IMU | 838 Hz, overrun = 0 przez 60 s, misaligned = 0; szum |a| 1,1 mg, ω 0,06–0,09 dps | 833 Hz, 0 |
| Czas SAVING – tryb 1 (1750 próbek + ~1500 zboczy, 55 kB) | 480–590 ms (CRC 18–20 ms) | < 1 s |
| Czas SAVING – tryb 3 (1750 próbek + ~4500 zboczy, 67 kB) | 756 ms (CRC 25 ms) | < 1 s |
| Pobieranie USB | 77 kB/s | – |
| Pobieranie BLE (bleak, BlueZ, BT 4.2) | 28 kB/s, MTU 247 | ≥ 20 kB/s |
| LIVE | 20 Hz | 20 Hz |
| RAM (statyczny) | tryb 1: 101 kB, tryb 3: 142 kB z 232 kB dostępnych dla aplikacji | – |

Budżet RAM: próbki 1750 × 28 B = 49 kB; log zboczy wymiarowany z `MAX_MOTOR_RPM`
= 40 000 rpm × okno 3,5 s × 1,1: tryb 1 5133 × 4 B = 20,5 kB, tryb 3 15 400 × 4 B
= 61,6 kB; takty 256 × 16 B; SoftDevice S140 zajmuje RAM od 0x20000000 do 0x20006000.

## Status kryteriów odbioru

| # | Kryterium | Status |
|---|---|---|
| 1 | Szkielet, oba tryby kompilują się | ✅ (oba tryby wgrane i przetestowane na płytce) |
| 2 | IMU 833 Hz bez gubienia, \|a\| ≈ 1 g | ✅ (60 s, 0 overrun) |
| 3 | Gaz ±2 µs, uczenie neutrum | ✅ algorytm (SIM); ⏳ zgodność z symulatorem ESP32 – wymaga okablowania |
| 4 | Hall ±0,5 % 500–30 000 rpm, szpilki, powrót do 0 | ✅ algorytm w trybach 1 i 3, kierunek A→B→C (SIM); ⏳ tor sprzętowy GPIOTE/PPI – wymaga hall_sim/auta |
| 5 | Logger: pre-trigger, liczba próbek, LED | ✅ (1750 próbek, trigger 250, odstęp 2000 µs) |
| 6 | Flash: 20 startów → 16 + overwritten 4; zanik zasilania; DEL (także DEL → natychmiastowy GET, seria pobierz+DEL); SAVING < 1 s | ✅ (zanik zasilania symulowany resetem w trakcie zapisu – `SIM RESETAT`; test z odłączeniem LiPo do wykonania) |
| 7 | LIST/GET/DEL przez USB i BLE, wznawianie, przepustowość | ✅ |
| 8 | Wektory testowe, testy analyze.py | ✅ 25 testów Pythona (w tym estymator C vs Python), 13 testów JS; płytka: 38/38 (`fw_test.py`) |
| 9 | Aplikacja – demo, analysis.js przechodzi wektory | ✅ (pobieranie, DEL po zapisie, tagi, wykresy, porównanie, zerwanie połączenia) |
| 10 | Aplikacja z prawdziwym Xiao na Androidzie, offline | ⏳ wymaga telefonu i hostingu HTTPS |
| 11–13 | Testy z autem, kalibracja, prędkościomierz na torze | ⏳ wymagają auta |

## Odstępstwa i interpretacje specyfikacji

* **`HALL_TIMEOUT_US`**: w trybie 1 przy 50 ms nie da się zmierzyć 500 rpm (zbocze co 60 ms),
  dlatego tryb 1 ma 80 ms (min. 375 rpm), tryb 3 – 50 ms.
* **Wstępne kasowanie slotu**: gdy pamięć jest pełna, najstarszy slot kasowany jest w chwili
  *wyzwolenia* (w trakcie RECORDING, ~35 ms), a nie z wyprzedzeniem w IDLE – inaczej przy 16 slotach
  w pamięci zostawałoby 15 startów i test „20 → 16, overwritten = 4” byłby niemożliwy. SAVING nigdy
  nie czeka na kasowanie. Wolne sloty po starcie są w tle sprawdzane, czy są puste.
* **CRC**: jedno CRC32 (IEEE) na start – z polem `crc32` nagłówka liczonym jako 0; to samo w `LIST`,
  nagłówku slotu i nagłówku startu.
* **Własna usługa NUS** (te same UUID) zamiast `BLEUart` – każda notyfikacja musi zaczynać się bajtem typu.
* **Start analizy** `i0` = wcześniejsze z (próg gazu, pierwsze zbocze halla) – ESC rusza silnik
  poniżej progu wyzwolenia. `esc_delay_s` może być ujemne.
* **Pochylenie**: prędkość pochylenia = −ω wokół osi „left” (wynika z `imu_axes`); `a_long =
  G·(a_fwd − a_ref − sin Δpitch)`, gdzie `a_ref` (bias + składowa grawitacji w spoczynku) jest
  średnią z okna spoczynku.
* **Estymator** (`v_disp`): reguła „lub warunek przyspieszeń trwa > 100 ms” sprawia, że przy kołach
  w powietrzu kręcących się ze *stałą* prędkością wskazanie po ~0,1 s dochodzi do prędkości kół.
  Na `v_imu` i poślizg to nie wpływa. W trakcie buksowania (koła przyspieszają) wskazanie nie skacze.
* Komendy `SIM …`, `REBOOT`, `DBG` – tylko USB, do testów na biurku.

## Do zrobienia przy aucie

1. Zmierzyć `HALL_PULLUP_V` multimetrem i rozkład pinów kabla sensora; potwierdzić `MOTOR_POLE_PAIRS`.
2. `THROTTLE_FORWARD_IS_HIGHER` wg reverse w aparaturze.
3. Kryteria 3–4 z `hall_sim` (ESP32 bez dzielnika), 10 z telefonem, 11–13 z autem.
4. FDR i średnica opony w ustawieniach aplikacji / `tools/config.yaml`, `IMU_YAW_OFFSET_DEG`.
