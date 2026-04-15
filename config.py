# ── I2C board stack addresses (0–7) ──────────────────────────────────────────
INPUT_BOARD1_STACK  = 0
OUTPUT_BOARD1_STACK = 1
OUTPUT_BOARD2_STACK = 2
INPUT_BOARD2_STACK  = 3

# ── Input channel assignments ─────────────────────────────────────────────────
CH_RUN_BTN      = 1    # Run procedure
CH_STOP_BTN     = 2    # Stop / abort
CH_JOG_ROT_FWD  = 6    # Manual jog: rotation forward
CH_JOG_ROT_REV  = 7    # Manual jog: rotation reverse
CH_JOG_VERT_UP  = 8    # Manual jog: vertical up
CH_JOG_VERT_DOWN = 9   # Manual jog: vertical down
CH_HOME_ROT     = 10   # Rotation home sensor (HIGH = at home)
CH_HOME_VERT    = 11   # Vertical home sensor (HIGH = at home)

# ── Output channel assignments ────────────────────────────────────────────────
CH_START_BTN_LED  = 1
CH_STOP_BTN_LED   = 2
CH_ESTOP_LED      = 3

# ── Homing ────────────────────────────────────────────────────────────────────
ROT_HOME_BACKOFF  = 20    # Steps back-off after home sensor triggers
VERT_HOME_BACKOFF = 20
HOMING_TIMEOUT_S  = 30.0  # Seconds before homing → ERROR

# ── Procedure positions (steps from home) ─────────────────────────────────────
ROT_POS_1   = 200
ROT_POS_2   = 400
VERT_POS_1  = 100
VERT_POS_2  = 200
VERT_POS_3  = 150
VERT_POS_4  = 0

# ── Timing ────────────────────────────────────────────────────────────────────
WAIT_TIME_1           = 2.0    # [s] – pauza po VERT_POS_1
WAIT_TIME_2           = 2.0    # [s] – pauza po VERT_POS_3
LOOP_PERIOD           = 0.005  # [s] – perioda control loop (200 Hz)
INPUT_SCAN_INTERVAL_S = 0.010  # [s] – perioda vstupního scanu (100 Hz)

# ── Arduino serial motor driver (ADRU/code.cpp) ───────────────────────────────
ARDUINO_PORT          = "/dev/ttyACM0"
ARDUINO_BAUD          = 115200
ARDUINO_READY_TIMEOUT = 5.0    # [s] – max čekání na "READY"
ARDUINO_MAX_SPEED     = 5000   # [krok/s] – výchozí max rychlost
ARDUINO_ACCELERATION  = 1000   # [krok/s²] – výchozí akcelerace
ARDUINO_HOME_SPEED    = 500    # [krok/s] – rychlost při homingu

# ── UPS – Suptronics X1205 ────────────────────────────────────────────────────
UPS_ENABLED            = True
UPS_I2C_BUS            = 1
UPS_AC_GPIO_CHIP       = "gpiochip0"
UPS_AC_GPIO_PIN        = 6       # HIGH = AC přítomno, LOW = AC odpojeno
UPS_POLL_INTERVAL_S    = 10.0   # [s]
UPS_LOW_BAT_THRESHOLD  = 20.0   # [%] – vypnutí při AC loss pod tuto hodnotu
UPS_CRITICAL_VOLTAGE_V = 3.20   # [V] – okamžité vypnutí
UPS_SHUTDOWN_DELAY_S   = 30     # [s] – prodleva při nízké baterii
