import time
from enum import Enum, auto

from hardware import HardwareInterface


class MotorState(Enum):
    IDLE    = auto()
    HOMING  = auto()
    RUNNING = auto()
    ERROR   = auto()


class Direction(Enum):
    FORWARD = 1
    REVERSE = 0


class MotorController:
    """
    Per-axis state machine for a stepper motor driven by PULSE/DIR signals.
    Call update() on every main loop iteration.
    """

    def __init__(
        self,
        name: str,
        hw: HardwareInterface,
        pulse_ch: int,
        dir_ch: int,
        enable_ch: int,
        home_input_ch: int,
        pulse_hz: float,
        home_backoff_steps: int,
        homing_timeout_s: float,
    ):
        self._name           = name
        self._hw             = hw
        self._pulse_ch       = pulse_ch
        self._dir_ch         = dir_ch
        self._enable_ch      = enable_ch
        self._home_ch        = home_input_ch
        self._pulse_interval = 1.0 / pulse_hz
        self._backoff_total  = home_backoff_steps
        self._homing_timeout = homing_timeout_s

        self._state          = MotorState.IDLE
        self._position       = 0
        self._target         = 0
        self._direction      = Direction.FORWARD
        self._last_pulse_t   = 0.0
        self._homing_start_t = 0.0
        self._in_backoff     = False
        self._backoff_steps  = 0

    # ── State queries ─────────────────────────────────────────────────────────

    @property
    def state(self) -> MotorState:
        return self._state

    @property
    def position(self) -> int:
        return self._position

    @property
    def is_idle(self) -> bool:
        return self._state == MotorState.IDLE

    @property
    def is_busy(self) -> bool:
        return self._state in (MotorState.HOMING, MotorState.RUNNING)

    @property
    def is_error(self) -> bool:
        return self._state == MotorState.ERROR

    # ── Commands ──────────────────────────────────────────────────────────────

    def cmd_home(self) -> None:
        if self._state in (MotorState.RUNNING, MotorState.HOMING, MotorState.ERROR):
            return
        self._in_backoff     = False
        self._backoff_steps  = 0
        self._homing_start_t = time.monotonic()
        self._direction      = Direction.REVERSE
        self._hw.set_output(self._dir_ch, False)
        self._state          = MotorState.HOMING
        print(f"[{self._name}] HOMING started")

    def cmd_move_to(self, target_steps: int) -> None:
        if self._state != MotorState.IDLE:
            return
        if target_steps == self._position:
            return
        self._target    = target_steps
        self._direction = (Direction.FORWARD if target_steps > self._position
                           else Direction.REVERSE)
        self._hw.set_output(self._dir_ch, self._direction == Direction.FORWARD)
        self._state     = MotorState.RUNNING
        print(f"[{self._name}] RUNNING -> target={target_steps} (pos={self._position})")

    def cmd_stop(self) -> None:
        if self._state in (MotorState.RUNNING, MotorState.HOMING):
            self._state = MotorState.IDLE
            print(f"[{self._name}] STOPPED (pos={self._position})")

    def cmd_jog(self, direction: Direction) -> None:
        if self._state != MotorState.IDLE:
            return
        self._hw.set_output(self._dir_ch, direction == Direction.FORWARD)
        self._emit_pulse(direction)

    def cmd_enable(self) -> None:
        self._hw.set_output(self._enable_ch, True)

    def cmd_disable(self) -> None:
        self._hw.set_output(self._enable_ch, False)

    def cmd_clear_error(self) -> None:
        if self._state == MotorState.ERROR:
            self._state = MotorState.IDLE
            print(f"[{self._name}] ERROR cleared")

    def cmd_estop(self) -> None:
        self._state = MotorState.ERROR
        self._hw.set_output(self._enable_ch, False)
        print(f"[{self._name}] ESTOP -> ERROR")

    # ── Tick ──────────────────────────────────────────────────────────────────

    def update(self) -> None:
        now = time.monotonic()
        if self._state == MotorState.HOMING:
            self._update_homing(now)
        elif self._state == MotorState.RUNNING:
            self._update_running(now)

    # ── Private helpers ───────────────────────────────────────────────────────

    def _should_pulse(self, now: float) -> bool:
        return (now - self._last_pulse_t) >= self._pulse_interval

    def _emit_pulse(self, direction: Direction) -> None:
        self._hw.set_output(self._pulse_ch, True)
        self._hw.set_output(self._pulse_ch, False)
        self._last_pulse_t = time.monotonic()
        if direction == Direction.FORWARD:
            self._position += 1
        else:
            self._position -= 1

    def _update_homing(self, now: float) -> None:
        if (now - self._homing_start_t) > self._homing_timeout:
            self._state = MotorState.ERROR
            self._hw.set_output(self._enable_ch, False)
            print(f"[{self._name}] HOMING TIMEOUT -> ERROR")
            return

        if not self._in_backoff:
            # Phase 1: move REVERSE until home sensor activates
            home_active = self._hw.read_input(self._home_ch)
            if home_active:
                self._in_backoff    = True
                self._backoff_steps = self._backoff_total
                self._direction     = Direction.FORWARD
                self._hw.set_output(self._dir_ch, True)
                print(f"[{self._name}] Home sensor hit, backing off {self._backoff_total} steps")
            else:
                if self._should_pulse(now):
                    self._emit_pulse(Direction.REVERSE)
        else:
            # Phase 2: back-off FORWARD
            if self._backoff_steps > 0:
                if self._should_pulse(now):
                    self._emit_pulse(Direction.FORWARD)
                    self._backoff_steps -= 1
            else:
                self._position = 0
                self._state    = MotorState.IDLE
                print(f"[{self._name}] HOMING complete, position reset to 0")

    def _update_running(self, now: float) -> None:
        if self._position == self._target:
            self._state = MotorState.IDLE
            print(f"[{self._name}] Move complete (pos={self._position})")
            return
        if self._should_pulse(now):
            self._emit_pulse(self._direction)
