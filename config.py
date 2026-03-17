# ── Hardware mode ─────────────────────────────────────────────────────────────
# "simulate" – žádný hardware, vše v paměti (výchozí pro vývoj)
# "i2c"      – Sequent Microsystems 16DI + 16DO desky přes I²C
# "gpio"     – přímé GPIO piny Raspberry Pi (BCM číslování)
# "mixed"    – kombinace: kanály v GPIO_*_PINS jdou přes GPIO,
#              ostatní kanály jdou přes I²C desky
HARDWARE_MODE = "simulate"

# Zpětná kompatibilita
SIMULATE = HARDWARE_MODE == "simulate"

# ── GPIO pin mapping (BCM číslování) ──────────────────────────────────────────
# Používá se pro HARDWARE_MODE = "gpio" i "mixed".
# Klíč = logický kanál (CH_* konstanty), hodnota = BCM číslo pinu.
#
# V režimu "mixed":
#   kanály UVEDENÉ zde   → GPIO pin
#   kanály NEUVEDENÉ zde → I²C deska (INPUT_BOARD1_STACK / OUTPUT_BOARD1_STACK)

GPIO_OUTPUT_PINS: dict = {
    #  kanál : BCM pin
}
GPIO_OUTPUT_PINS: dict = {
    #  kanál : BCM pin
    1:   4,   # CH_PULSE_ROT
    2:  18,   # CH_DIR_ROT
    3:  15,   # CH_ENABLE_ROT
    4:  14,   # CH_PULSE_VERT
    5:   8,   # CH_DIR_VERT
    6:   7,   # CH_ENABLE_VERT
}

# Pull-up / pull-down pro vstupní GPIO piny ("up", "down", nebo None)
GPIO_INPUT_PULL = "up"   # tlačítka a senzory zpravidla na pull-up

# ── I2C board stack addresses (0–7) ──────────────────────────────────────────
INPUT_BOARD1_STACK  = 0
OUTPUT_BOARD1_STACK = 1
OUTPUT_BOARD2_STACK = 2
INPUT_BOARD2_STACK = 3
RESERVED_STACKS = [4, 5, 6, 7]   # For future expansion (e.g. more I/O boards)

# ── Input channel assignments (16DI, channels 1–16) ──────────────────────────
CH_RUN_BTN          = 1   # Run procedure
CH_STOP_BTN         = 2   # Stop / abort
CH_PROC_SEL_1       = 3   # Procedure select 1 (binary)
CH_PROC_SEL_2       = 4   # Procedure select 2 (binary)
CH_PROC_SEL_3       = 5   # Procedure select 3 (binary);
CH_PROC_SEL_4       = 6   # Procedure select 4 (binary)
CH_PROC_SEL_5       = 7   # Procedure select 5 (binary)
CH_PROC_SEL_6       = 8   # Procedure select 6 (binary)
CH_JOG_ROT_FWD      = 6   # Manual jog: rotation forward
CH_JOG_ROT_REV      = 7   # Manual jog: rotation reverse
CH_JOG_VERT_UP      = 8   # Manual jog: vertical up
CH_JOG_VERT_DOWN    = 9   # Manual jog: vertical down
CH_HOME_ROT         = 10  # Rotation home sensor (HIGH = at home)
CH_HOME_VERT        = 11  # Vertical home sensor (HIGH = at home)

# ── Output channel assignments (16DO, channels 1–16) ─────────────────────────
CH_START_BTN_LED = 1    # Start button LED (HIGH = on)
CH_STOP_BTN_LED  = 2    # Stop button LED (HIGH = on)
CH_ESTOP_LED     = 3    # Emergency stop LED (HIGH = on)
CH_PROC_SEL_1_LED = 4    # Procedure select 1 LED (HIGH = on)
CH_PROC_SEL_2_LED = 5    # Procedure select 2 LED (HIGH = on)
CH_PROC_SEL_3_LED = 6    # Procedure select 3 LED (HIGH = on)
CH_PROC_SEL_4_LED = 7    # Procedure select 4 LED (HIGH = on)
CH_PROC_SEL_5_LED = 8    # Procedure select 5 LED (HIGH = on)
CH_PROC_SEL_6_LED = 9    # Procedure select 6 LED (HIGH = on)
CH_JOG_ROT_FWD_LED   = 6   # Manual jog: rotation forward
CH_JOG_ROT_REV_LED   = 7   # Manual jog: rotation reverse
CH_JOG_VERT_UP_LED   = 8   # Manual jog: vertical up
CH_JOG_VERT_DOWN_LED = 9   # Manual jog: vertical down

# ── Motor control channel assignments (RPI GPIO) ───────────────────
CH_PULSE_ROT   = 1    # Stepper PULSE – rotation
CH_DIR_ROT     = 2    # Stepper DIR   – rotation
CH_ENABLE_ROT  = 3    # Motor ENABLE  – rotation (HIGH = enabled)
CH_PULSE_VERT  = 4    # Stepper PULSE – vertical
CH_DIR_VERT    = 5    # Stepper DIR   – vertical
CH_ENABLE_VERT = 6    # Motor ENABLE  – vertical

# ── Motor parameters ──────────────────────────────────────────────────────────
ROT_PULSE_HZ        = 50    # Steps/sec for rotation axis (keep <= 100)
VERT_PULSE_HZ       = 50    # Steps/sec for vertical axis
ROT_HOME_BACKOFF    = 20    # Steps back-off after home sensor triggers
VERT_HOME_BACKOFF   = 20
HOMING_TIMEOUT_S    = 30.0  # Seconds before homing -> ERROR

# ── Procedure positions (steps from home, 0 = home) ──────────────────────────
ROT_POS_1   = 200   # Rotation position 1 (step count)
ROT_POS_2   = 400   # Rotation position 2
VERT_POS_1  = 100   # Vertical position 1
VERT_POS_2  = 200   # Vertical position 2
VERT_POS_3  = 150   # Vertical position 3
VERT_POS_4  = 0     # Vertical position 4 (home level)

# ── Timing ────────────────────────────────────────────────────────────────────
WAIT_TIME_1 = 2.0    # Seconds to wait after vertical pos 1
WAIT_TIME_2 = 2.0    # Seconds to wait after vertical pos 3
LOOP_PERIOD = 0.005  # Main loop period in seconds (200 Hz)