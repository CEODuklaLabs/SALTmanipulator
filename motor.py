import logging
import queue
import threading
import time
from enum import Enum, auto
from typing import Optional

import config
from hardware import HardwareInterface

logger = logging.getLogger(__name__)


class MotorState(Enum):
    IDLE    = auto()
    HOMING  = auto()
    RUNNING = auto()
    ERROR   = auto()


class Direction(Enum):
    FORWARD = 1
    REVERSE = 0


try:
    import serial as _serial   # pyserial
    _SERIAL_OK = True
except ImportError:
    _SERIAL_OK = False
    logger.warning("pyserial není dostupný – ArduinoMotorController nebude funkční")


class SerialBridge:
    """
    Thread-safe sériová linka k Arduinu (ADRU/code.cpp).

    TX příkazy:  R+1000  Z-500  RS5000  ZA1000  RX  ZX  RE  ZE  RD  ZD
    RX zprávy:   READY   DONE R   DONE Z   STOP R   STOP Z   ERR ...

    Listeners registrované přes on(event, cb) jsou volány pro každou
    přijatou řádku, jejíž začátek odpovídá ``event`` (prefix match).
    """

    _HOMING_LARGE_MOVE = 999999

    def __init__(self, port: str, baud: int = 115200, ready_timeout: float = 5.0) -> None:
        self._port          = port
        self._baud          = baud
        self._ready_timeout = ready_timeout
        self._ser           = None
        self._tx_q: queue.Queue = queue.Queue()
        self._stop          = threading.Event()
        self._ready         = threading.Event()
        self._listeners: dict[str, list] = {}
        self._lock          = threading.Lock()
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if not _SERIAL_OK:
            raise RuntimeError("pyserial není nainstalován (pip install pyserial)")
        self._stop.clear()
        self._ready.clear()
        self._ser = _serial.Serial(self._port, self._baud, timeout=1.0)
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="arduino-rx")
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True, name="arduino-tx")
        self._rx_thread.start()
        self._tx_thread.start()
        if not self._ready.wait(timeout=self._ready_timeout):
            logger.warning("[ARDUINO] READY nepřišlo do %.1f s", self._ready_timeout)
        else:
            logger.info("[ARDUINO] připojen na %s", self._port)

    def stop(self) -> None:
        self._stop.set()
        self._tx_q.put(None)
        for t in (self._rx_thread, self._tx_thread):
            if t:
                t.join(timeout=2.0)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    @property
    def is_ready(self) -> bool:
        return self._ready.is_set()

    def send(self, cmd: str) -> None:
        self._tx_q.put(cmd)
        logger.debug("[ARDUINO TX] %s", cmd)

    def on(self, event: str, callback) -> None:
        with self._lock:
            self._listeners.setdefault(event, []).append(callback)

    def remove(self, event: str, callback) -> None:
        with self._lock:
            lst = self._listeners.get(event, [])
            if callback in lst:
                lst.remove(callback)

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                logger.debug("[ARDUINO RX] %s", line)
                if line == "READY":
                    self._ready.set()
                self._dispatch(line)
            except Exception as exc:
                if not self._stop.is_set():
                    logger.warning("[ARDUINO RX] chyba: %s", exc)

    def _tx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                cmd = self._tx_q.get(timeout=0.1)
                if cmd is None:
                    break
                self._ser.write((cmd + "\n").encode("ascii"))
                self._ser.flush()
            except queue.Empty:
                pass
            except Exception as exc:
                if not self._stop.is_set():
                    logger.warning("[ARDUINO TX] chyba: %s", exc)

    def _dispatch(self, line: str) -> None:
        with self._lock:
            items = list(self._listeners.items())
        for prefix, callbacks in items:
            if line.startswith(prefix):
                for cb in callbacks:
                    try:
                        cb(line)
                    except Exception as exc:
                        logger.error("[ARDUINO] listener výjimka: %s", exc)


class ArduinoMotorController:
    """
    Řídí jeden stepper přes SerialBridge (firmware ADRU/code.cpp).

    Homingová sekvence (3 fáze):
      APPROACH – velký záporný pohyb, update() hlídá home sensor
      STOPPING – sensor triggernut → RX, čeká STOP potvrzení
      BACKOFF  – backoff kroky vpřed, čeká DONE, nastaví position=0
    """

    class _Phase(Enum):
        APPROACH = auto()
        STOPPING = auto()
        BACKOFF  = auto()

    def __init__(
        self,
        name:               str,
        axis_char:          str,
        bridge:             SerialBridge,
        hw:                 HardwareInterface,
        home_input_ch:      int,
        home_backoff_steps: int,
        homing_timeout_s:   float,
        max_speed:          float = 5000,
        acceleration:       float = 1000,
        home_speed:         float = 500,
        inpos_input_ch:     Optional[int] = None,
        alarm_input_ch:     Optional[int] = None,
    ) -> None:
        assert axis_char in ("R", "Z")
        self._name      = name
        self._ax        = axis_char
        self._bridge    = bridge
        self._hw        = hw
        self._home_ch   = home_input_ch
        self._inpos_ch  = inpos_input_ch    # InPosition z řízení osy (aktivní HIGH), volitelné
        self._alarm_ch  = alarm_input_ch    # Alarm z řízení osy (aktivní HIGH), volitelné
        self._inpos_armed = False           # InPosition už jednou spadl na LOW (pohyb běží)
        self._backoff_total    = home_backoff_steps
        self._homing_timeout   = homing_timeout_s
        self._max_speed        = float(max_speed)
        self._acceleration     = float(acceleration)
        self._home_speed       = float(home_speed)
        self._state            = MotorState.IDLE
        self._position         = 0
        self._target           = 0
        self._position_known   = False   # True po dokončeném homingu, False po jogu
        self._homing_phase: Optional[ArduinoMotorController._Phase] = None
        self._homing_start_t   = 0.0
        self._run_start_t      = 0.0     # čas startu RUNNING pohybu (watchdog)
        self._jogging          = False

        bridge.on(f"DONE {axis_char}", self._on_done)
        bridge.on(f"STOP {axis_char}", self._on_stop)
        bridge.on("ERR", self._on_err)

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def state(self) -> MotorState:
        return self._state

    @property
    def position(self) -> int:
        return self._position

    @property
    def target(self) -> int:
        return self._target

    @property
    def is_idle(self) -> bool:
        return self._state == MotorState.IDLE

    @property
    def is_busy(self) -> bool:
        return self._state in (MotorState.HOMING, MotorState.RUNNING)

    @property
    def is_error(self) -> bool:
        return self._state == MotorState.ERROR

    @property
    def position_known(self) -> bool:
        return self._position_known

    @property
    def max_speed(self) -> float:
        return self._max_speed

    @property
    def acceleration(self) -> float:
        return self._acceleration

    @property
    def home_speed(self) -> float:
        return self._home_speed

    # ── Serial helpers ────────────────────────────────────────────────────────

    def _send_speed(self, speed: float) -> None:
        self._bridge.send(f"{self._ax}S{int(speed)}")

    def _send_accel(self, accel: float) -> None:
        self._bridge.send(f"{self._ax}A{int(accel)}")

    # ── Commands ──────────────────────────────────────────────────────────────

    def cmd_home(self) -> None:
        if self._state in (MotorState.RUNNING, MotorState.HOMING, MotorState.ERROR):
            return
        if self._home_ch is None:
            self._state = MotorState.ERROR
            logger.error("[%s] HOMING nelze – home senzor nenakonfigurován (config.py)", self._name)
            return
        self._homing_phase   = self._Phase.APPROACH
        self._homing_start_t = time.monotonic()
        self._state          = MotorState.HOMING
        self._jogging        = False
        self._position_known = False
        self._send_speed(self._home_speed)
        self._bridge.send(f"{self._ax}-{SerialBridge._HOMING_LARGE_MOVE}")
        logger.info("[%s] HOMING approach zahájen", self._name)

    def cmd_move_to(self, target_steps: int) -> None:
        if self._state != MotorState.IDLE or target_steps == self._position:
            return
        self._target       = target_steps
        self._state        = MotorState.RUNNING
        self._jogging      = False
        self._inpos_armed  = False
        self._run_start_t  = time.monotonic()
        self._send_speed(self._max_speed)
        self._bridge.send(f"{self._ax}{target_steps - self._position:+d}")
        logger.info("[%s] RUNNING target=%d (pos=%d)", self._name, target_steps, self._position)

    def cmd_stop(self) -> None:
        if self._state in (MotorState.RUNNING, MotorState.HOMING):
            self._bridge.send(f"{self._ax}X")

    def cmd_jog_start(self, direction: Direction) -> None:
        """Zahájí ruční pojezd; jede dokud nepřijde cmd_jog_stop() nebo dojezd chunku."""
        if self._state != MotorState.IDLE:
            return
        steps = config.JOG_CHUNK_STEPS if direction == Direction.FORWARD else -config.JOG_CHUNK_STEPS
        self._state          = MotorState.RUNNING
        self._jogging        = True
        self._position_known = False   # po jogu je absolutní poloha nejistá → nutný homing
        self._run_start_t    = time.monotonic()
        self._send_speed(self._home_speed)
        self._bridge.send(f"{self._ax}{steps:+d}")
        logger.info("[%s] JOG start (%s)", self._name, direction.name)

    def cmd_jog_stop(self) -> None:
        if self._jogging and self._state == MotorState.RUNNING:
            self._bridge.send(f"{self._ax}X")

    def cmd_enable(self) -> None:
        self._bridge.send(f"{self._ax}E")

    def cmd_disable(self) -> None:
        self._bridge.send(f"{self._ax}D")

    def assume_homed(self) -> None:
        """Osa fyzicky sedí na home senzoru – přijmi pozici 0 bez pohybu."""
        if self._state in (MotorState.RUNNING, MotorState.HOMING):
            return
        self._position       = 0
        self._target         = 0
        self._position_known = True
        self._state          = MotorState.IDLE
        logger.info("[%s] pozice přijata jako home (0)", self._name)

    def cmd_clear_error(self) -> None:
        if self._state == MotorState.ERROR:
            self._state        = MotorState.IDLE
            self._homing_phase = None
            self._jogging      = False
            logger.info("[%s] ERROR cleared", self._name)

    def cmd_estop(self) -> None:
        self._bridge.send(f"{self._ax}X")
        self._bridge.send(f"{self._ax}D")
        self._state          = MotorState.ERROR
        self._homing_phase   = None
        self._jogging        = False
        self._position_known = False
        logger.warning("[%s] ESTOP", self._name)

    def set_params(
        self,
        max_speed:    Optional[float] = None,
        acceleration: Optional[float] = None,
        home_speed:   Optional[float] = None,
    ) -> None:
        """Aktualizuje parametry pohybu; pokud IDLE, odešle okamžitě na Arduino."""
        for val, attr, send_fn in [
            (max_speed,    "_max_speed",    self._send_speed),
            (acceleration, "_acceleration", self._send_accel),
            (home_speed,   "_home_speed",   None),
        ]:
            if val is not None:
                v = float(val)
                if v > 0:
                    setattr(self, attr, v)
                    if send_fn and self._state == MotorState.IDLE:
                        send_fn(v)
                    logger.info("[%s] %s = %.0f", self._name, attr.lstrip("_"), v)

    # ── Tick ──────────────────────────────────────────────────────────────────

    def update(self) -> None:
        now = time.monotonic()

        # Alarm z řízení osy (aktivní HIGH) → ERROR
        if (self._alarm_ch is not None
                and self._state in (MotorState.RUNNING, MotorState.HOMING)
                and self._hw.read_input(self._alarm_ch)):
            self._bridge.send(f"{self._ax}X")
            self._bridge.send(f"{self._ax}D")
            self._state        = MotorState.ERROR
            self._homing_phase = None
            self._jogging      = False
            logger.error("[%s] ALARM z řízení osy", self._name)
            return

        # RUNNING (mimo jog) – dokončení podle InPosition, jinak watchdog
        if self._state == MotorState.RUNNING and not self._jogging:
            if self._inpos_ch is not None:
                inpos = self._hw.read_input(self._inpos_ch)
                if not self._inpos_armed:
                    if not inpos:
                        self._inpos_armed = True          # řízení hlásí "busy"
                elif inpos:
                    self._position = self._target
                    self._state    = MotorState.IDLE
                    logger.info("[%s] pohyb dokončen dle InPosition (pos=%d)",
                                self._name, self._position)
                    return
            if now - self._run_start_t > config.MOVE_TIMEOUT_S:
                self._bridge.send(f"{self._ax}X")
                self._state = MotorState.ERROR
                logger.error("[%s] MOVE TIMEOUT (>%.0fs)", self._name, config.MOVE_TIMEOUT_S)
            return

        if self._state != MotorState.HOMING:
            return
        if now - self._homing_start_t > self._homing_timeout:
            self._bridge.send(f"{self._ax}X")
            self._bridge.send(f"{self._ax}D")
            self._state        = MotorState.ERROR
            self._homing_phase = None
            logger.error("[%s] HOMING TIMEOUT", self._name)
            return
        if self._homing_phase == self._Phase.APPROACH:
            if self._hw.read_input(self._home_ch):
                self._homing_phase = self._Phase.STOPPING
                self._bridge.send(f"{self._ax}X")

    # ── RX callbacks ─────────────────────────────────────────────────────────

    def _on_done(self, _: str) -> None:
        if self._state == MotorState.HOMING and self._homing_phase == self._Phase.BACKOFF:
            self._position       = 0
            self._state          = MotorState.IDLE
            self._homing_phase   = None
            self._position_known = True
            self._send_speed(self._max_speed)
            logger.info("[%s] HOMING dokončen, position=0", self._name)
        elif self._state == MotorState.RUNNING and self._jogging:
            # jog dojel celý chunk – absolutní poloha je nejistá
            self._state   = MotorState.IDLE
            self._jogging = False
            logger.info("[%s] JOG chunk dokončen", self._name)
        elif self._state == MotorState.RUNNING:
            self._position = self._target
            self._state    = MotorState.IDLE
            logger.info("[%s] pohyb dokončen (pos=%d)", self._name, self._position)

    def _on_stop(self, _: str) -> None:
        if self._state == MotorState.HOMING and self._homing_phase == self._Phase.STOPPING:
            self._homing_phase = self._Phase.BACKOFF
            self._send_speed(self._home_speed)
            self._bridge.send(f"{self._ax}+{self._backoff_total}")
            logger.info("[%s] backoff %d kroků", self._name, self._backoff_total)
        elif self._state in (MotorState.RUNNING, MotorState.HOMING):
            self._state        = MotorState.IDLE
            self._homing_phase = None
            self._jogging      = False

    def _on_err(self, line: str) -> None:
        if self._state in (MotorState.RUNNING, MotorState.HOMING):
            self._bridge.send(f"{self._ax}X")
            self._state        = MotorState.ERROR
            self._homing_phase = None
            self._jogging      = False
            logger.error("[%s] Arduino ERR: %s", self._name, line)
