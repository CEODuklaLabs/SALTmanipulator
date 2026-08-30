# SALT Manipulator

Řízení manipulátoru pro cyklus **pec → chlazení → home**. Raspberry Pi (Python/Flask)
+ Arduino (stepper driver přes sériovou linku) + I²C I/O desky Sequent Microsystems.

## Technologický cyklus

Osy: **R** = rotace [°], **Z** = svislý pojezd [mm] (0 mm = nahoře).
Stanoviště rotace: **−120° pec**, **0° chlazení**, **+120° home**.

Výchozí (home) stav: Z nahoře, R = +120°. Po STARTu (jednorázově):

| # | Krok | Akce |
|---|------|------|
| 1 | `HOMING_Z` / `HOMING_R` | Z na horní senzor, pak R na home senzor (+120°) |
| 2 | `ROT_TO_FURNACE` | R → −120° |
| 3 | `Z_DOWN_FURNACE` | Z → `Z_FURNACE_MM` (400 mm) |
| 4 | `DWELL_FURNACE` | výdrž `recipe.furnace_time_s` |
| 5 | `Z_UP_FROM_FURNACE` | Z → 0 mm |
| 6 | `ROT_TO_COOLING` | R → 0° |
| 7 | `Z_DOWN_COOLING` | Z → `Z_COOLING_MM` (< 400 mm) |
| 8 | `DWELL_COOLING` | výdrž `recipe.cooling_time_s` |
| 9 | `Z_UP_FROM_COOLING` | Z → 0 mm |
| 10 | `ROT_TO_HOME` | R → +120° |
| 11 | `COMPLETE` → `IDLE` | konec, nutný nový START |

**Interlock:** rotace se pohne jen když je Z prokazatelně nahoře
(`|Z_pos| ≤ Z_UP_TOLERANCE_STEPS` a poloha známá po homingu). Jinak `FAULT`.

**STOP** = abort, motory zastaví na místě (`STOPPED`), obsluha řeší ručně (jog / home).
**RESET** ze stavu `STOPPED / COMPLETE / ESTOP / FAULT` → `IDLE`.
Chyba motoru (timeout pohybu, `ERR` z Arduina) → `FAULT`.

Každý START cyklu **vždy** začíná homingem (`HOMING_Z → HOMING_R`).
Fyzická tlačítka jsou softwarově **debouncovaná** (`INPUT_DEBOUNCE_SCANS`).

## Napájení motorů a RESET

Napájení motorů (stykače) je po spuštění a po každém nouzovém stopu **vypnuté**.
Obsluha musí ve webu stisknout **RESET** → control loop dá krátký pulz na **relé 7**
(`MOTOR_PWR_PULSE_S`, ~1 s), který sepne stykače, a procedura přejde do `IDLE`.
RESET je odmítnut, dokud je aktivní nouzový stop (HV-in1). Po RESETu proběhne
inicializace: rameno na home senzorech → pozice se přijme; jinak (a `AUTO_HOME_ON_STARTUP`)
automatický homing (`INIT_Z → INIT_R`). START / homing jsou zablokované, dokud není
napájení ON.

**Nouzový stop** (HV modul, in1, aktivní HIGH – stroj OK = LOW): hardware odpojí
napájení motorů, software na náběžnou hranu přejde do `ESTOP`, zruší panel-jog a
napájení. Total-stop v rozvaděči navíc detekuje UPS monitor (ztráta AC) a udělá totéž.

## Volba receptury (P1–P6)

Panel má tlačítka **P1–P6**. Před startem musí obsluha zvolit jednu recepturu – ve webu
zůstane zvýrazněná a drží se i po `STOPPED` / `COMPLETE` (další cyklus jde spustit hned).
Bez zvolené receptury je START zablokovaný. Volba jde jen v `IDLE`.
Hodnoty (výdrž pec / chlazení) jsou v `config.RECIPES`.
*(Fyzické kontrolky P1–P6 zatím nejsou zapojené – relé 1–6 jsou rezerva.)*

## Ovládání – LOCAL / REMOTE

- **LOCAL** (výchozí): řídí fyzický panel – `START`, `STOP`, `P1–P6`.
- **REMOTE**: po připojení web aplikace (SSE) se automaticky převezme řízení a fyzická
  tlačítka (kromě **STOP**, které funguje vždy) přestanou reagovat.
- Zpět na LOCAL jen ručně – tlačítko **„Předat na panel"** ve webu. Žádný auto-release.

**Ruční jog z panelu** je *standardně vypnutý* a nezávislý na LOCAL/REMOTE. Zapíná se
z webu tlačítkem **„Panel jog"** (`/api/cmd/panel_jog {enabled}`). Poté fungují jog
tlačítka na panelu (jen v `IDLE` a při zapnutém napájení). Vypne se z webu nebo při `ESTOP`.

## I/O moduly (Sequent Microsystems) – přímý I²C přes `smbus`

Konfigurace v `config.INPUT_BOARDS` / `config.OUTPUT_BOARDS` (adresa, počet kanálů,
registry, `invert`). Kanály jsou číslované **globálně** přes moduly daného směru.

| I²C | modul | glob. kanály | registry | obsah |
|---|---|---|---|---|
| `0x20` | LV vstup #1 | 1–16 | čtení 0x00, 0x01 | jog (1–4), P1–P6 (14–9), STOP (15), START (16) |
| `0x21` | LV vstup #2 | 17–32 | čtení 0x00, 0x01 | *(plán)* InPos/Alarm os, home senzory |
| `0x23` | HV vstup 230 V | 33–48 | čtení 0x00, 0x01 | nouzový stop (33) |
| `0x27` | HV výstup 8× relé | 1–8 | zápis 0x01 | **rel 7** = pulz napájení motorů, **rel 8** = stav (1=CHYBA→červená, 0=OK→zelená) |
| `0x22` | LV výstup 16× | 9–24 | zápis 0x01, 0x02 | rezerva |

Vstup: byte se čte z `regs` (regs[0] = kanály 1–8, regs[1] = 9–16).
Výstup: `hardware.py` drží **stínovou kopii bytu** na každý registr a při `set_output()`
mění jen dotčený bit → jednotlivé relé, ne celý port najednou. `invert` obrací logiku
(nastav, pokud deska čte/spíná obráceně – ověř na stroji).

Zpětné signály z řízení os: `CH_ROT_INPOS` / `CH_ROT_ALARM` (a `CH_VERT_*`) – InPosition
(aktivní HIGH) = dokončení pohybu osy, Alarm (aktivní HIGH) → chyba motoru. Dokud jsou
`None`, používá se jen sériové `DONE` z Arduina.

**Home senzory** (`CH_HOME_ROT`, `CH_HOME_VERT`) zatím `None`. Kvůli tomu je
`config.REQUIRE_HOMING = False` – **homing se přeskakuje** (cyklus i RESET berou
aktuální polohu os jako 0). Vhodné pro bench test s motory mimo stroj. Až budou home
senzory zapojené: doplň čísla kanálů (LV vstup #2, 17–32) a přepni `REQUIRE_HOMING = True`.

## Spuštění

### Ručně (venv)

```bash
cd ~/SALTmanipulator
bash setup.sh                 # jednorázově: .venv + pip závislosti
                              #   (bash setup.sh --hardware = navíc povolí I2C)
source .venv/bin/activate
python webapp.py              # → http://<IP-Raspberry>:5000
```

Předpoklady na Pi:
- povolené I2C (`sudo raspi-config` → Interface Options → I2C), uživatel ve skupinách
  `i2c` a `dialout` (`sudo usermod -aG i2c,dialout $USER`, pak odhlásit/přihlásit),
- Arduino na `/dev/ttyACM0` (jinak uprav `ARDUINO_PORT` v `config.py`),
- `smbus` – buď z venv (`smbus2` z requirements se použije automaticky), nebo
  `sudo apt install python3-smbus`.

### Automaticky po startu (systemd)

```bash
sudo cp deploy/salt-manipulator.service /etc/systemd/system/
sudo nano /etc/systemd/system/salt-manipulator.service   # zkontroluj User= a cesty
sudo systemctl daemon-reload
sudo systemctl enable --now salt-manipulator
journalctl -u salt-manipulator -f                        # log
```

## Konfigurace

Vše v [`config.py`](config.py). Nutná **kalibrace na stroji**:

- `ROT_STEPS_PER_DEG`, `VERT_STEPS_PER_MM` – převod jednotek na kroky (klidně záporné
  podle směru osy).
- `ROT_HOME_ANGLE_DEG` = 120 (úhel u homing senzoru R), `VERT_HOME_MM` = 0.
- `Z_COOLING_MM` – přesná hloubka chlazení (zatím orientačně 300).
- `RECIPES` – hodnoty receptur P1–P6 `(výdrž_pec_s, výdrž_chlazení_s)`.
- `CH_RECIPE_BTN` / `CH_RECIPE_LED` – I/O kanály tlačítek a kontrolek P1–P6
  (uprav podle skutečného zapojení).

## Protokol Arduina (`ADRU/code.cpp`, firmware je fixní)

TX: `R+1000` / `Z-500` (rel. pohyb), `RS5000` (rychlost), `RA1000` (akcelerace),
`RX` (stop), `RE` / `RD` (enable / disable). Osa `R` nebo `Z`.
RX: `READY`, `DONE R` / `DONE Z`, `STOP R` / `STOP Z`, `ERR …`.

Homing dělá Python: velký záporný pohyb + `RX` po sepnutí home senzoru + backoff.

## Soubory

| Soubor | Obsah |
|--------|-------|
| `webapp.py` | Flask dashboard, control loop 200 Hz, režim LOCAL/REMOTE, LED |
| `procedure.py` | stavový automat cyklu + `Recipe` |
| `motor.py` | `SerialBridge` + `ArduinoMotorController` (homing, jog, watchdog) |
| `hardware.py` | I²C I/O desky Sequent Microsystems |
| `ups.py` | monitor UPS Suptronics X1205 (ESTOP + shutdown při výpadku AC) |
| `config.py` | veškerá konfigurace |
