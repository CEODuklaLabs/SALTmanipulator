import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import TYPE_CHECKING, Optional

import config

if TYPE_CHECKING:
    from motor import ArduinoMotorController


@dataclass
class Recipe:
    """Aktivní parametry cyklu – hodnoty zvolené receptury P1–P6."""
    furnace_time_s: float = 0.0
    cooling_time_s: float = 0.0

    def to_dict(self) -> dict:
        return {
            "furnace_time_s": round(self.furnace_time_s, 1),
            "cooling_time_s": round(self.cooling_time_s, 1),
        }


class ProcedureStep(Enum):
    IDLE               = auto()
    INIT_Z             = auto()   # inicializační homing po spuštění – Z
    INIT_R             = auto()   # inicializační homing po spuštění – R
    HOMING_Z           = auto()   # nejdřív Z nahoru (kvůli interlocku rotace)
    HOMING_R           = auto()   # pak R na home senzor (+120°)
    ROT_TO_FURNACE     = auto()   # R → -120°
    Z_DOWN_FURNACE     = auto()   # Z → Z_FURNACE_MM (do pece)
    DWELL_FURNACE      = auto()   # výdrž recipe.furnace_time_s
    Z_UP_FROM_FURNACE  = auto()   # Z → nahoru
    ROT_TO_COOLING     = auto()   # R → 0°
    Z_DOWN_COOLING     = auto()   # Z → Z_COOLING_MM (do chlazení)
    DWELL_COOLING      = auto()   # výdrž recipe.cooling_time_s
    Z_UP_FROM_COOLING  = auto()   # Z → nahoru
    ROT_TO_HOME        = auto()   # R → +120°
    COMPLETE           = auto()
    STOPPED            = auto()
    ESTOP              = auto()
    FAULT              = auto()


# Kroky, ve kterých se rotace pohybuje – vyžadují Z nahoře (interlock)
_ROT_STEPS = {
    ProcedureStep.ROT_TO_FURNACE,
    ProcedureStep.ROT_TO_COOLING,
    ProcedureStep.ROT_TO_HOME,
}

# Terminální / neaktivní kroky – automat v nich nic nedělá
_INACTIVE = {
    ProcedureStep.IDLE,
    ProcedureStep.COMPLETE,
    ProcedureStep.STOPPED,
    ProcedureStep.ESTOP,
    ProcedureStep.FAULT,
}

# Pořadí pro dashboard (bez terminálních stavů)
STEP_SEQUENCE = [
    ProcedureStep.IDLE,
    ProcedureStep.HOMING_Z,
    ProcedureStep.HOMING_R,
    ProcedureStep.ROT_TO_FURNACE,
    ProcedureStep.Z_DOWN_FURNACE,
    ProcedureStep.DWELL_FURNACE,
    ProcedureStep.Z_UP_FROM_FURNACE,
    ProcedureStep.ROT_TO_COOLING,
    ProcedureStep.Z_DOWN_COOLING,
    ProcedureStep.DWELL_COOLING,
    ProcedureStep.Z_UP_FROM_COOLING,
    ProcedureStep.ROT_TO_HOME,
    ProcedureStep.COMPLETE,
]


class ProcedureStateMachine:
    """
    Sekvenční řízení cyklu manipulátoru (pec → chlazení → home) přes dva
    ArduinoMotorController.

    Cyklus (jednorázový, po dokončení nutný nový START):
        HOMING_Z → HOMING_R → ROT -120° → Z do pece → výdrž → Z nahoru →
        ROT 0° → Z do chlazení → výdrž → Z nahoru → ROT +120° → COMPLETE
    """

    def __init__(
        self,
        rot_motor:  "ArduinoMotorController",
        vert_motor: "ArduinoMotorController",
        recipe:     Optional[Recipe] = None,
    ):
        self._rot        = rot_motor
        self._vert       = vert_motor
        self._recipe     = recipe or Recipe()
        self._recipe_id: Optional[int] = None   # zvolená receptura P1–P6 (None = žádná)
        self._step       = ProcedureStep.IDLE
        self._wait_start = 0.0
        self._fault_msg  = ""

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def step(self) -> ProcedureStep:
        return self._step

    @property
    def recipe(self) -> Recipe:
        return self._recipe

    @property
    def recipe_id(self) -> Optional[int]:
        return self._recipe_id

    @property
    def fault_msg(self) -> str:
        return self._fault_msg

    @property
    def is_running(self) -> bool:
        return self._step not in _INACTIVE

    # ── Commands ──────────────────────────────────────────────────────────────

    def cmd_start(self) -> None:
        if self._step != ProcedureStep.IDLE:
            return
        if self._recipe_id is None:
            self._fault_msg = "Před startem zvol recepturu (P1–P6)"
            return
        if not self._home_sensors_ok():
            return
        if not self._rot.is_idle or not self._vert.is_idle:
            self._fault("Motory nejsou v klidu")
            return
        self._fault_msg = ""
        self._rot.cmd_enable()
        self._vert.cmd_enable()
        self._enter_step(ProcedureStep.HOMING_Z)

    def _home_sensors_ok(self) -> bool:
        if not config.REQUIRE_HOMING:
            return True
        if config.CH_HOME_ROT is None or config.CH_HOME_VERT is None:
            self._fault_msg = "Home senzory nejsou nakonfigurované (config.py)"
            return False
        return True

    def cmd_init(self) -> None:
        """Inicializační homing po spuštění – po dokončení zůstane v IDLE."""
        if self._step != ProcedureStep.IDLE:
            return
        if not self._rot.is_idle or not self._vert.is_idle:
            return
        if not self._home_sensors_ok():
            return
        self._fault_msg = ""
        self._rot.cmd_enable()
        self._vert.cmd_enable()
        self._enter_step(ProcedureStep.INIT_Z)

    def cmd_stop(self) -> None:
        """Abort – motory zastaví na místě, cyklus končí."""
        if self._step in _INACTIVE:
            return
        self._rot.cmd_stop()
        self._vert.cmd_stop()
        self._step = ProcedureStep.STOPPED

    def cmd_estop(self) -> None:
        self._rot.cmd_estop()
        self._vert.cmd_estop()
        self._step = ProcedureStep.ESTOP

    def cmd_reset(self) -> None:
        if self._step in (ProcedureStep.STOPPED, ProcedureStep.COMPLETE,
                          ProcedureStep.ESTOP, ProcedureStep.FAULT):
            self._rot.cmd_clear_error()
            self._vert.cmd_clear_error()
            self._fault_msg = ""
            self._step = ProcedureStep.IDLE

    def select_recipe(self, recipe_id: int) -> None:
        """Volba receptury P1–P6 – povoleno jen v klidu (IDLE)."""
        if self._step != ProcedureStep.IDLE:
            return
        if recipe_id not in config.RECIPES:
            return
        furnace, cooling = config.RECIPES[recipe_id]
        self._recipe_id = recipe_id
        self._recipe.furnace_time_s = float(furnace)
        self._recipe.cooling_time_s = float(cooling)
        self._fault_msg = ""

    # ── Tick ──────────────────────────────────────────────────────────────────

    def update(self) -> None:
        step = self._step

        # COMPLETE chvíli zůstane viditelný na dashboardu, pak zpět na IDLE
        # (další cyklus se spouští novým STARTem).
        if step == ProcedureStep.COMPLETE:
            if self._wait_elapsed(2.0):
                self._step = ProcedureStep.IDLE
            return

        if step in _INACTIVE:
            return

        # Chyba motoru kdykoliv během cyklu → FAULT
        if self._rot.is_error or self._vert.is_error:
            self._fault("Chyba motoru (timeout / ERR z Arduina)")
            return

        if step == ProcedureStep.INIT_Z:
            if self._vert.is_idle:
                self._enter_step(ProcedureStep.INIT_R)

        elif step == ProcedureStep.INIT_R:
            if self._rot.is_idle:
                self._rot.cmd_disable()
                self._vert.cmd_disable()
                self._step = ProcedureStep.IDLE

        elif step == ProcedureStep.HOMING_Z:
            if self._vert.is_idle:
                self._enter_step(ProcedureStep.HOMING_R)

        elif step == ProcedureStep.HOMING_R:
            if self._rot.is_idle:
                self._enter_step(ProcedureStep.ROT_TO_FURNACE)

        elif step == ProcedureStep.ROT_TO_FURNACE:
            if self._rot.is_idle:
                self._enter_step(ProcedureStep.Z_DOWN_FURNACE)

        elif step == ProcedureStep.Z_DOWN_FURNACE:
            if self._vert.is_idle:
                self._enter_step(ProcedureStep.DWELL_FURNACE)

        elif step == ProcedureStep.DWELL_FURNACE:
            if self._wait_elapsed(self._recipe.furnace_time_s):
                self._enter_step(ProcedureStep.Z_UP_FROM_FURNACE)

        elif step == ProcedureStep.Z_UP_FROM_FURNACE:
            if self._vert.is_idle:
                self._enter_step(ProcedureStep.ROT_TO_COOLING)

        elif step == ProcedureStep.ROT_TO_COOLING:
            if self._rot.is_idle:
                self._enter_step(ProcedureStep.Z_DOWN_COOLING)

        elif step == ProcedureStep.Z_DOWN_COOLING:
            if self._vert.is_idle:
                self._enter_step(ProcedureStep.DWELL_COOLING)

        elif step == ProcedureStep.DWELL_COOLING:
            if self._wait_elapsed(self._recipe.cooling_time_s):
                self._enter_step(ProcedureStep.Z_UP_FROM_COOLING)

        elif step == ProcedureStep.Z_UP_FROM_COOLING:
            if self._vert.is_idle:
                self._enter_step(ProcedureStep.ROT_TO_HOME)

        elif step == ProcedureStep.ROT_TO_HOME:
            if self._rot.is_idle:
                self._enter_step(ProcedureStep.COMPLETE)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _enter_step(self, step: ProcedureStep) -> None:
        # Interlock: rotace jen když je Z prokazatelně nahoře
        if step in _ROT_STEPS and not self._z_is_up():
            self._fault(f"Interlock: Z není nahoře před {step.name}")
            return

        self._step = step

        if step in (ProcedureStep.HOMING_Z, ProcedureStep.INIT_Z):
            self._vert.cmd_home() if config.REQUIRE_HOMING else self._vert.assume_homed()
        elif step in (ProcedureStep.HOMING_R, ProcedureStep.INIT_R):
            self._rot.cmd_home() if config.REQUIRE_HOMING else self._rot.assume_homed()
        elif step == ProcedureStep.ROT_TO_FURNACE:
            self._rot.cmd_move_to(config.rot_deg_to_steps(config.ROT_ANGLE_FURNACE))
        elif step == ProcedureStep.Z_DOWN_FURNACE:
            self._vert.cmd_move_to(config.vert_mm_to_steps(config.Z_FURNACE_MM))
        elif step == ProcedureStep.Z_UP_FROM_FURNACE:
            self._vert.cmd_move_to(config.vert_mm_to_steps(config.Z_TOP_MM))
        elif step == ProcedureStep.ROT_TO_COOLING:
            self._rot.cmd_move_to(config.rot_deg_to_steps(config.ROT_ANGLE_COOLING))
        elif step == ProcedureStep.Z_DOWN_COOLING:
            self._vert.cmd_move_to(config.vert_mm_to_steps(config.Z_COOLING_MM))
        elif step == ProcedureStep.Z_UP_FROM_COOLING:
            self._vert.cmd_move_to(config.vert_mm_to_steps(config.Z_TOP_MM))
        elif step == ProcedureStep.ROT_TO_HOME:
            self._rot.cmd_move_to(config.rot_deg_to_steps(config.ROT_ANGLE_HOME))
        elif step in (ProcedureStep.DWELL_FURNACE, ProcedureStep.DWELL_COOLING):
            self._wait_start = time.monotonic()
        elif step == ProcedureStep.COMPLETE:
            self._wait_start = time.monotonic()
            self._rot.cmd_disable()
            self._vert.cmd_disable()

    def _z_is_up(self) -> bool:
        return (self._vert.position_known
                and abs(self._vert.position) <= config.Z_UP_TOLERANCE_STEPS)

    def _fault(self, msg: str) -> None:
        self._fault_msg = msg
        self._rot.cmd_stop()
        self._vert.cmd_stop()
        self._step = ProcedureStep.FAULT

    def _wait_elapsed(self, duration: float) -> bool:
        return (time.monotonic() - self._wait_start) >= duration
