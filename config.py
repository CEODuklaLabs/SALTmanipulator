# ═══════════════════════════════════════════════════════════════════════════
#  I/O moduly (Sequent Microsystems) – přímý přístup přes smbus
# ═══════════════════════════════════════════════════════════════════════════
# Vstup:  byte se čte z registrů 'regs' (reg[0] = kanály 1–8, reg[1] = 9–16).
# Výstup: drží se stínová kopie bytu, mění se jen dotčený bit, zapíše celý byte
#         do 'regs' (reg[0] = výstupy 1–8, reg[1] = 9–16).
# 'invert': True pokud deska vrací/očekává invertovanou logiku (ověř na stroji!).
#
# Vstupní i výstupní moduly se řadí za sebou → globální číslování kanálů:
#   1. modul → 1..N,  2. modul → N+1..,  atd.
I2C_BUS = 1

INPUT_BOARDS = [
    {"name": "LV1", "addr": 0x20, "channels": 16, "regs": [0x00, 0x01], "invert": False},  # → 1..16
    {"name": "LV2", "addr": 0x21, "channels": 16, "regs": [0x00, 0x01], "invert": False},  # → 17..32
    {"name": "HV",  "addr": 0x23, "channels": 16, "regs": [0x00, 0x01], "invert": False},  # → 33..48
]
OUTPUT_BOARDS = [
    {"name": "HV8",  "addr": 0x27, "channels": 8,  "regs": [0x01],       "invert": False},  # → 1..8
    {"name": "LV16", "addr": 0x22, "channels": 16, "regs": [0x01, 0x02], "invert": False},  # → 9..24
]

# ── Vstupy – LV modul #1  (0x20, globální kanály 1..16) ─────────────────────
CH_JOG_VERT_DOWN = 1     # jog dolů
CH_JOG_VERT_UP   = 2     # jog nahoru
CH_JOG_ROT_REV   = 3     # jog doleva
CH_JOG_ROT_FWD   = 4     # jog doprava
#   kanály 5–8 rezerva
CH_RECIPE_BTN    = {1: 14, 2: 13, 3: 12, 4: 11, 5: 10, 6: 9}   # P1..P6
CH_STOP_BTN      = 15
CH_RUN_BTN       = 16    # Start cyklu

# ── Vstupy – LV modul #2  (0x21, globální kanály 17..32) ───────────────────
# Až budou signály z řadičů nastavené, odkomentuj / doplň čísla kanálů.
# LV2 pozice 3 → kanál 19, pozice 4 → kanál 20 atd.
CH_VERT_INPOS = None   # InPosition z řízení pojezdu (plán: LV2 poz. 3 → 19)  aktivní HIGH
CH_VERT_ALARM = None   # Alarm z řízení pojezdu      (plán: LV2 poz. 4 → 20)  aktivní HIGH
CH_ROT_INPOS  = None   # InPosition z řízení rotace  – aktivní HIGH
CH_ROT_ALARM  = None   # Alarm z řízení rotace       – aktivní HIGH
CH_HOME_ROT   = None   # referenční senzor rotace    – bez něj homing nepojede
CH_HOME_VERT  = None   # referenční senzor pojezdu   – bez něj homing nepojede

# ── Vstupy – HV modul  (0x23, globální kanály 33..48, 230 V) ───────────────
CH_ESTOP_IN   = 33     # nouzový stop – aktivní HIGH (stroj OK = LOW)

# ── Výstupy – modul HV8  (0x27, globální kanály 1..8) ──────────────────────
CH_MOTOR_PWR_RESET  = 7  # RESET z webu: krátký pulz → sepne stykače napájení motorů
CH_STATUS_ERR_RELAY = 8  # sepnuto = CHYBA (NO → červená), rozepnuto = OK (NC → zelená)
MOTOR_PWR_PULSE_S   = 1.0 # délka RESET pulzu [s]
#   relé 1–6 rezerva; výstupy 9..24 = modul LV16 (0x22) – zatím nevyužito

# ── Homing ────────────────────────────────────────────────────────────────────
ROT_HOME_BACKOFF  = 20    # Steps back-off after home sensor triggers
VERT_HOME_BACKOFF = 20
HOMING_TIMEOUT_S  = 30.0  # Seconds before homing → ERROR

# ── Kinematika – převod jednotek na kroky motoru ─────────────────────────────
# POZOR: hodnoty jsou orientační, nutná kalibrace na stroji.
# Znaménko klidně otoč, pokud osa jede opačným směrem než DIR pin očekává.
ROT_STEPS_PER_DEG   = 10.0    # [krok/°]  – kalibrace na stroji
VERT_STEPS_PER_MM   = 100.0   # [krok/mm] – kalibrace na stroji
ROT_HOME_ANGLE_DEG  = 120.0   # úhel odpovídající poloze homing senzoru R (= home)
VERT_HOME_MM        = 0.0     # výška odpovídající poloze homing senzoru Z (= nahoře)

# ── Stanoviště rotace (úhel R ve stupních) ───────────────────────────────────
ROT_ANGLE_FURNACE   = -120.0  # pec
ROT_ANGLE_COOLING   = 0.0     # chladicí nádoba
ROT_ANGLE_HOME      = 120.0   # home / klidová poloha

# ── Polohy svislého pojezdu (mm absolutně, 0 mm = nahoře) ────────────────────
Z_TOP_MM            = 0.0     # horní / bezpečná poloha (rotace povolena)
Z_FURNACE_MM        = 400.0   # spuštěno do pece
Z_COOLING_MM        = 300.0   # TODO přesná hodnota (< 400) – spuštěno do chlazení

# ── Receptury P1–P6:  (výdrž v peci [s], výdrž v chlazení [s]) ──────────────
# Před startem cyklu musí obsluha zvolit jednu recepturu (tlačítka P1–P6).
RECIPES = {
    1: (60.0,  60.0),
    2: (120.0, 90.0),
    3: (180.0, 120.0),
    4: (240.0, 150.0),
    5: (300.0, 180.0),
    6: (600.0, 300.0),
}

# ── Interlock / watchdog ────────────────────────────────────────────────────
Z_UP_TOLERANCE_STEPS  = 50    # R se smí otáčet jen když |Z_pos| ≤ tato hodnota
MOVE_TIMEOUT_S        = 60.0  # [s] – watchdog RUNNING pohybu → ERROR

# ── Timing ────────────────────────────────────────────────────────────────────
LOOP_PERIOD           = 0.005  # [s] – perioda control loop (200 Hz)
INPUT_SCAN_INTERVAL_S = 0.010  # [s] – perioda vstupního scanu (100 Hz)
INPUT_DEBOUNCE_SCANS  = 3      # počet shodných čtení než se změna vstupu potvrdí (~30 ms)

# ── Inicializace / homing ──────────────────────────────────────────────────
# !!! DOČASNĚ False – motory jsou mimo stroj, home senzory nejsou zapojené.
# Cyklus i RESET pak homing přeskočí a berou aktuální polohu jako 0.
# Až budou senzory (CH_HOME_ROT / CH_HOME_VERT) → přepni na True.
REQUIRE_HOMING       = False
AUTO_HOME_ON_STARTUP = True    # (platí jen když REQUIRE_HOMING = True)

# ── Arduino serial motor driver (ADRU/code.cpp) ───────────────────────────────
ARDUINO_PORT          = "/dev/ttyACM0"
ARDUINO_BAUD          = 115200
ARDUINO_READY_TIMEOUT = 5.0    # [s] – max čekání na "READY"
ARDUINO_MAX_SPEED     = 5000   # [krok/s] – výchozí max rychlost
ARDUINO_ACCELERATION  = 1000   # [krok/s²] – výchozí akcelerace
ARDUINO_HOME_SPEED    = 500    # [krok/s] – rychlost při homingu
JOG_CHUNK_STEPS       = 2000   # délka jednoho jog pohybu (přeruší se uvolněním tlačítka)

# ── UPS – Suptronics X1205 ────────────────────────────────────────────────────
UPS_ENABLED            = True
UPS_I2C_BUS            = 1
UPS_AC_GPIO_CHIP       = "gpiochip0"
UPS_AC_GPIO_PIN        = 6       # HIGH = AC přítomno, LOW = AC odpojeno
UPS_POLL_INTERVAL_S    = 10.0   # [s]
UPS_LOW_BAT_THRESHOLD  = 20.0   # [%] – vypnutí při AC loss pod tuto hodnotu
UPS_CRITICAL_VOLTAGE_V = 3.20   # [V] – okamžité vypnutí
UPS_SHUTDOWN_DELAY_S   = 30     # [s] – prodleva při nízké baterii


# ── Převodní funkce jednotek → kroky ────────────────────────────────────────
def rot_deg_to_steps(deg: float) -> int:
    """Úhel [°] → absolutní pozice v krocích (0 kroků = ROT_HOME_ANGLE_DEG)."""
    return round((ROT_HOME_ANGLE_DEG - deg) * ROT_STEPS_PER_DEG)


def vert_mm_to_steps(mm: float) -> int:
    """Výška [mm] → absolutní pozice v krocích (0 kroků = VERT_HOME_MM)."""
    return round((mm - VERT_HOME_MM) * VERT_STEPS_PER_MM)


def rot_steps_to_deg(steps: int) -> float:
    return ROT_HOME_ANGLE_DEG - steps / ROT_STEPS_PER_DEG


def vert_steps_to_mm(steps: int) -> float:
    return VERT_HOME_MM + steps / VERT_STEPS_PER_MM
