"""
SALT Manipulator - entry point
Run: python main.py
"""

import time

from config import (
    SIMULATE,
    INPUT_BOARD_STACK, OUTPUT_BOARD_STACK,
    CH_ESTOP, CH_STOP_BTN, CH_START_BTN,
    CH_JOG_ROT_FWD, CH_JOG_ROT_REV,
    CH_JOG_VERT_UP, CH_JOG_VERT_DOWN,
    CH_PULSE_ROT, CH_DIR_ROT, CH_ENABLE_ROT, CH_HOME_ROT,
    CH_PULSE_VERT, CH_DIR_VERT, CH_ENABLE_VERT, CH_HOME_VERT,
    ROT_PULSE_HZ, VERT_PULSE_HZ,
    ROT_HOME_BACKOFF, VERT_HOME_BACKOFF,
    HOMING_TIMEOUT_S, LOOP_PERIOD,
)
from hardware import HardwareInterface
from motor import MotorController, Direction
from procedure import ProcedureStateMachine, ProcedureStep


def main() -> None:
    hw = HardwareInterface(INPUT_BOARD_STACK, OUTPUT_BOARD_STACK)

    rot_motor = MotorController(
        name               = "ROT",
        hw                 = hw,
        pulse_ch           = CH_PULSE_ROT,
        dir_ch             = CH_DIR_ROT,
        enable_ch          = CH_ENABLE_ROT,
        home_input_ch      = CH_HOME_ROT,
        pulse_hz           = ROT_PULSE_HZ,
        home_backoff_steps = ROT_HOME_BACKOFF,
        homing_timeout_s   = HOMING_TIMEOUT_S,
    )

    vert_motor = MotorController(
        name               = "VERT",
        hw                 = hw,
        pulse_ch           = CH_PULSE_VERT,
        dir_ch             = CH_DIR_VERT,
        enable_ch          = CH_ENABLE_VERT,
        home_input_ch      = CH_HOME_VERT,
        pulse_hz           = VERT_PULSE_HZ,
        home_backoff_steps = VERT_HOME_BACKOFF,
        homing_timeout_s   = HOMING_TIMEOUT_S,
    )

    procedure = ProcedureStateMachine(rot_motor, vert_motor)

    prev_inputs: dict = {ch: False for ch in range(1, 17)}

    print("SALT Manipulator started. SIMULATE =", SIMULATE)
    print("Press Ctrl+C to exit.")

    try:
        while True:
            t_start = time.monotonic()

            inputs = hw.read_all_inputs()

            # -- Priority 1: ESTOP (level-triggered) --------------------------
            if inputs[CH_ESTOP]:
                if procedure.step != ProcedureStep.ESTOP:
                    procedure.cmd_estop()

            else:
                # -- Priority 2: STOP button (rising edge) --------------------
                if inputs[CH_STOP_BTN] and not prev_inputs[CH_STOP_BTN]:
                    procedure.cmd_stop()

                # -- Priority 3: START button (rising edge, IDLE only) --------
                if (inputs[CH_START_BTN] and not prev_inputs[CH_START_BTN]
                        and procedure.step == ProcedureStep.IDLE):
                    procedure.cmd_start()

                # -- Manual jog (procedure must be IDLE) ----------------------
                if procedure.step == ProcedureStep.IDLE:
                    if inputs[CH_JOG_ROT_FWD]:
                        rot_motor.cmd_jog(Direction.FORWARD)
                    elif inputs[CH_JOG_ROT_REV]:
                        rot_motor.cmd_jog(Direction.REVERSE)
                    if inputs[CH_JOG_VERT_UP]:
                        vert_motor.cmd_jog(Direction.FORWARD)
                    elif inputs[CH_JOG_VERT_DOWN]:
                        vert_motor.cmd_jog(Direction.REVERSE)

            # -- Motor ticks (pulse generation) --------------------------------
            rot_motor.update()
            vert_motor.update()

            # -- Procedure tick (sequence advancement) -------------------------
            procedure.update()

            prev_inputs = inputs

            # -- Pace loop to LOOP_PERIOD --------------------------------------
            elapsed = time.monotonic() - t_start
            sleep_t = LOOP_PERIOD - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\nShutdown: disabling motors.")
        rot_motor.cmd_disable()
        vert_motor.cmd_disable()


if __name__ == "__main__":
    main()
