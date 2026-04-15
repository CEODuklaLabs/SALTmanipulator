import time
from enum import Enum, auto
from typing import TYPE_CHECKING

from config import (
    ROT_POS_1, ROT_POS_2,
    VERT_POS_1, VERT_POS_2, VERT_POS_3, VERT_POS_4,
    WAIT_TIME_1, WAIT_TIME_2,
)

if TYPE_CHECKING:
    from motor import ArduinoMotorController


class ProcedureStep(Enum):
    IDLE         = auto()
    HOMING       = auto()
    MOVE_ROT_1   = auto()
    MOVE_VERT_1  = auto()
    WAIT_1       = auto()
    MOVE_VERT_2  = auto()
    MOVE_ROT_2   = auto()
    MOVE_VERT_3  = auto()
    WAIT_2       = auto()
    MOVE_VERT_4  = auto()
    HOMING_FINAL = auto()
    COMPLETE     = auto()
    STOPPED      = auto()
    ESTOP        = auto()


class ProcedureStateMachine:
    """Sekvenční řízení procedury přes dva ArduinoMotorController."""

    def __init__(
        self,
        rot_motor:  "ArduinoMotorController",
        vert_motor: "ArduinoMotorController",
    ):
        self._rot        = rot_motor
        self._vert       = vert_motor
        self._step       = ProcedureStep.IDLE
        self._wait_start = 0.0

    @property
    def step(self) -> ProcedureStep:
        return self._step

    # ── Commands ──────────────────────────────────────────────────────────────

    def cmd_start(self) -> None:
        if self._step == ProcedureStep.IDLE:
            self._rot.cmd_enable()
            self._vert.cmd_enable()
            self._enter_step(ProcedureStep.HOMING)

    def cmd_stop(self) -> None:
        if self._step not in (ProcedureStep.STOPPED, ProcedureStep.ESTOP,
                               ProcedureStep.IDLE, ProcedureStep.COMPLETE):
            self._rot.cmd_stop()
            self._vert.cmd_stop()
            self._step = ProcedureStep.STOPPED

    def cmd_estop(self) -> None:
        self._rot.cmd_estop()
        self._vert.cmd_estop()
        self._step = ProcedureStep.ESTOP

    def cmd_reset(self) -> None:
        if self._step in (ProcedureStep.STOPPED, ProcedureStep.COMPLETE,
                           ProcedureStep.ESTOP):
            self._rot.cmd_clear_error()
            self._vert.cmd_clear_error()
            self._step = ProcedureStep.IDLE

    # ── Tick ──────────────────────────────────────────────────────────────────

    def update(self) -> None:
        step = self._step

        if step == ProcedureStep.HOMING:
            if self._motors_idle():
                self._enter_step(ProcedureStep.MOVE_ROT_1)

        elif step == ProcedureStep.MOVE_ROT_1:
            if self._rot.is_idle:
                self._enter_step(ProcedureStep.MOVE_VERT_1)

        elif step == ProcedureStep.MOVE_VERT_1:
            if self._vert.is_idle:
                self._enter_step(ProcedureStep.WAIT_1)

        elif step == ProcedureStep.WAIT_1:
            if self._wait_elapsed(WAIT_TIME_1):
                self._enter_step(ProcedureStep.MOVE_VERT_2)

        elif step == ProcedureStep.MOVE_VERT_2:
            if self._vert.is_idle:
                self._enter_step(ProcedureStep.MOVE_ROT_2)

        elif step == ProcedureStep.MOVE_ROT_2:
            if self._rot.is_idle:
                self._enter_step(ProcedureStep.MOVE_VERT_3)

        elif step == ProcedureStep.MOVE_VERT_3:
            if self._vert.is_idle:
                self._enter_step(ProcedureStep.WAIT_2)

        elif step == ProcedureStep.WAIT_2:
            if self._wait_elapsed(WAIT_TIME_2):
                self._enter_step(ProcedureStep.MOVE_VERT_4)

        elif step == ProcedureStep.MOVE_VERT_4:
            if self._vert.is_idle:
                self._enter_step(ProcedureStep.HOMING_FINAL)

        elif step == ProcedureStep.HOMING_FINAL:
            if self._motors_idle():
                self._enter_step(ProcedureStep.COMPLETE)

        elif step == ProcedureStep.COMPLETE:
            self._rot.cmd_disable()
            self._vert.cmd_disable()
            self._step = ProcedureStep.IDLE

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _enter_step(self, step: ProcedureStep) -> None:
        self._step = step

        if step == ProcedureStep.HOMING:
            self._rot.cmd_home()
            self._vert.cmd_home()
        elif step == ProcedureStep.MOVE_ROT_1:
            self._rot.cmd_move_to(ROT_POS_1)
        elif step == ProcedureStep.MOVE_VERT_1:
            self._vert.cmd_move_to(VERT_POS_1)
        elif step in (ProcedureStep.WAIT_1, ProcedureStep.WAIT_2):
            self._wait_start = time.monotonic()
        elif step == ProcedureStep.MOVE_VERT_2:
            self._vert.cmd_move_to(VERT_POS_2)
        elif step == ProcedureStep.MOVE_ROT_2:
            self._rot.cmd_move_to(ROT_POS_2)
        elif step == ProcedureStep.MOVE_VERT_3:
            self._vert.cmd_move_to(VERT_POS_3)
        elif step == ProcedureStep.MOVE_VERT_4:
            self._vert.cmd_move_to(VERT_POS_4)
        elif step == ProcedureStep.HOMING_FINAL:
            self._rot.cmd_home()
            self._vert.cmd_home()

    def _motors_idle(self) -> bool:
        return self._rot.is_idle and self._vert.is_idle

    def _wait_elapsed(self, duration: float) -> bool:
        return (time.monotonic() - self._wait_start) >= duration
