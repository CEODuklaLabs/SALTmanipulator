# ── Simulation flag ───────────────────────────────────────────────────────────
SIMULATE = True   # Set False on real hardware

# ── I2C board stack addresses (0–7) ──────────────────────────────────────────
INPUT_BOARD_STACK  = 0
OUTPUT_BOARD_STACK = 0

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