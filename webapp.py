"""
SALT Manipulator – web dashboard
Run: python webapp.py  →  http://localhost:5000
"""

import json
import logging
import queue
import threading
import time
from typing import Any

from flask import Flask, Response, jsonify, render_template, request

import config
from hardware import HardwareInterface
from motor import ArduinoMotorController, Direction, SerialBridge
from procedure import ProcedureStateMachine, ProcedureStep, Recipe
from ups import UPSMonitor

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(name)s: %(message)s")
log = logging.getLogger("webapp")

app = Flask(__name__)

# ── Hardware ──────────────────────────────────────────────────────────────────

hw = HardwareInterface()
hw.start_input_scan()

# ── Arduino serial bridge ────────────────────────────────────────────────────

_bridge = SerialBridge(
    port          = config.ARDUINO_PORT,
    baud          = config.ARDUINO_BAUD,
    ready_timeout = config.ARDUINO_READY_TIMEOUT,
)
_bridge.start()

rot_motor = ArduinoMotorController(
    name               = "ROT",
    axis_char          = "R",
    bridge             = _bridge,
    hw                 = hw,
    home_input_ch      = config.CH_HOME_ROT,
    home_backoff_steps = config.ROT_HOME_BACKOFF,
    homing_timeout_s   = config.HOMING_TIMEOUT_S,
    max_speed          = config.ARDUINO_MAX_SPEED,
    acceleration       = config.ARDUINO_ACCELERATION,
    home_speed         = config.ARDUINO_HOME_SPEED,
    inpos_input_ch     = config.CH_ROT_INPOS,
    alarm_input_ch     = config.CH_ROT_ALARM,
)
vert_motor = ArduinoMotorController(
    name               = "VERT",
    axis_char          = "Z",
    bridge             = _bridge,
    hw                 = hw,
    home_input_ch      = config.CH_HOME_VERT,
    home_backoff_steps = config.VERT_HOME_BACKOFF,
    homing_timeout_s   = config.HOMING_TIMEOUT_S,
    max_speed          = config.ARDUINO_MAX_SPEED,
    acceleration       = config.ARDUINO_ACCELERATION,
    home_speed         = config.ARDUINO_HOME_SPEED,
    inpos_input_ch     = config.CH_VERT_INPOS,
    alarm_input_ch     = config.CH_VERT_ALARM,
)

procedure = ProcedureStateMachine(rot_motor, vert_motor, Recipe())

# Receptury P1–P6 pro dashboard (statické, z config.RECIPES)
_RECIPES_DICT = {
    str(rid): {"furnace_time_s": f, "cooling_time_s": c}
    for rid, (f, c) in config.RECIPES.items()
}

# ── Řízení: LOCAL (panel) / REMOTE (web) ─────────────────────────────────────
# Po připojení web klienta (claim) přestane panel reagovat (kromě STOP).
# Zpět na LOCAL jen ručně z webu (release) – žádný auto-release.

_mode      = "LOCAL"
_mode_lock = threading.Lock()


def _get_mode() -> str:
    with _mode_lock:
        return _mode


def _set_mode(m: str) -> None:
    global _mode
    with _mode_lock:
        _mode = m


# Ruční jog z fyzického panelu – standardně vypnutý, povoluje se z web aplikace.
# Nezávislé na LOCAL/REMOTE; jog tlačítka na panelu fungují jen když je toto set.
_panel_jog = threading.Event()

# Napájení motorů (stykače). Po spuštění / nouzovém stopu je OFF – obsluha musí
# stisknout RESET ve webu → 1s pulz na relé sepne stykače → _motor_power = ON.
_motor_power = threading.Event()


# ── SSE / state ───────────────────────────────────────────────────────────────

_cmd_queue: queue.Queue        = queue.Queue()
_state_lock                    = threading.Lock()
_current_state: dict[str, Any] = {}
_sse_subscribers: list[queue.Queue] = []
_sse_lock                      = threading.Lock()

# ── UPS ───────────────────────────────────────────────────────────────────────
# Celé RPi (+ I/O desky) běží na UPS. Ztráta AC = total-stop v rozvaděči → ESTOP.

def _enqueue_estop() -> None:
    _panel_jog.clear()
    _cmd_queue.put(("estop", {}))


def _on_ac_loss() -> None:
    log.warning("[UPS] ztráta napájení (total-stop) – ESTOP")
    _enqueue_estop()


ups = UPSMonitor(config, estop_callback=_enqueue_estop,
                 ac_loss_callback=_on_ac_loss)
ups.start()

_INPUT_STR_KEYS = {ch: str(ch) for ch in range(1, hw.num_channels + 1)}

_ups_cache: dict       = {}
_ups_cache_lock        = threading.Lock()


def _ups_cache_updater() -> None:
    while True:
        fresh = ups.get_status()
        with _ups_cache_lock:
            _ups_cache.clear()
            _ups_cache.update(fresh)
        time.sleep(config.UPS_POLL_INTERVAL_S)


threading.Thread(target=_ups_cache_updater, daemon=True, name="ups-cache").start()


def _get_ups_snapshot() -> dict:
    with _ups_cache_lock:
        return dict(_ups_cache)


def _notify_subscribers(data: str) -> None:
    with _sse_lock:
        if not _sse_subscribers:
            return
        dead = []
        for q in _sse_subscribers:
            try:
                q.put_nowait(data)
            except queue.Full:
                dead.append(q)
        for q in dead:
            _sse_subscribers.remove(q)


def _motor_snapshot(mc: ArduinoMotorController) -> dict:
    return {
        "state":          mc.state.name,
        "position":       mc.position,
        "target":         mc.target,
        "position_known": mc.position_known,
        "params": {
            "max_speed":    mc.max_speed,
            "acceleration": mc.acceleration,
            "home_speed":   mc.home_speed,
        },
    }


# ── Jog helpers ─────────────────────────────────────────────────────────────

_JOG_INPUTS = (
    (config.CH_JOG_ROT_FWD,   lambda: rot_motor,  Direction.FORWARD),
    (config.CH_JOG_ROT_REV,   lambda: rot_motor,  Direction.REVERSE),
    (config.CH_JOG_VERT_UP,   lambda: vert_motor, Direction.FORWARD),
    (config.CH_JOG_VERT_DOWN, lambda: vert_motor, Direction.REVERSE),
)


def _motor_by_axis(axis: str):
    return rot_motor if axis == "rot" else (vert_motor if axis == "vert" else None)


# ── Control loop (200 Hz) ─────────────────────────────────────────────────────

def _estop_active(raw_val) -> bool:
    """Vyhodnotí HV vstup nouzového stopu (respektuje ESTOP_INPUT_ENABLED / polaritu)."""
    if not config.ESTOP_INPUT_ENABLED:
        return False
    return bool(raw_val) == config.ESTOP_INPUT_ACTIVE_HIGH


def _run_init_homing() -> None:
    """Po RESETu / obnově napájení: rameno na home senzorech → přijmi pozici;
    jinak (a je-li AUTO_HOME_ON_STARTUP) automatický homing."""
    if not config.REQUIRE_HOMING:
        rot_motor.assume_homed()
        vert_motor.assume_homed()
        log.info("[INIT] homing vypnutý (REQUIRE_HOMING=False) – poloha = 0")
        return
    if config.CH_HOME_ROT is None or config.CH_HOME_VERT is None:
        log.warning("[INIT] home senzory nejsou nakonfigurované – přeskočeno")
        return
    inp = hw.read_all_inputs()
    if inp.get(config.CH_HOME_ROT) and inp.get(config.CH_HOME_VERT):
        rot_motor.assume_homed()
        vert_motor.assume_homed()
        log.info("[INIT] rameno v home pozici – pozice přijata")
    elif config.AUTO_HOME_ON_STARTUP:
        log.info("[INIT] rameno není v home pozici – inicializační homing")
        procedure.cmd_init()
    else:
        log.warning("[INIT] rameno není v home pozici (AUTO_HOME_ON_STARTUP=False)")


def _control_loop() -> None:
    prev_inputs: dict[int, bool] = {ch: False for ch in range(1, hw.num_channels + 1)}
    pj_prev     = False
    rel7_until  = 0.0     # do kdy držet RESET pulz na relé napájení motorů
    init_done   = False   # inicializační homing po RESETu už proběhl

    while True:
        t_start = time.monotonic()

        # ── Web commands ──────────────────────────────────────────────────────
        while not _cmd_queue.empty():
            try:
                cmd, args = _cmd_queue.get_nowait()
            except queue.Empty:
                break

            # Vždy povolené (bezpečnost + přepínání režimu / povolení panelu)
            if cmd == "stop":
                procedure.cmd_stop()
            elif cmd == "estop":
                procedure.cmd_estop()
                _panel_jog.clear()
                _motor_power.clear()
                init_done = False
            elif cmd == "reset":
                if _estop_active(hw.read_input(config.CH_ESTOP_IN)):
                    log.warning("[RESET] odmítnuto – nouzový stop je stále aktivní")
                else:
                    rel7_until = t_start + config.MOTOR_PWR_PULSE_S   # pulz stykačů
                    _motor_power.set()
                    init_done = False
                    procedure.cmd_reset()
                    log.info("[RESET] pulz napájení motorů %.1fs, procedura → IDLE",
                             config.MOTOR_PWR_PULSE_S)
            elif cmd == "claim":
                _set_mode("REMOTE")
            elif cmd == "release":
                _set_mode("LOCAL")
            elif cmd == "jog_stop":
                m = _motor_by_axis(args.get("axis", ""))
                if m:
                    m.cmd_jog_stop()
            elif cmd == "panel_jog":
                if args.get("enabled"):
                    _panel_jog.set()
                else:
                    _panel_jog.clear()

            # Jen v režimu REMOTE (web má řízení)
            elif _get_mode() == "REMOTE":
                if cmd == "start":
                    if _motor_power.is_set():
                        procedure.cmd_start()
                elif cmd == "home":
                    axis = args.get("axis", "both")
                    if _motor_power.is_set() and procedure.step == ProcedureStep.IDLE:
                        if axis in ("rot", "both"):
                            rot_motor.cmd_home()
                        if axis in ("vert", "both"):
                            vert_motor.cmd_home()
                elif cmd == "jog":
                    if procedure.step == ProcedureStep.IDLE:
                        m   = _motor_by_axis(args.get("axis", ""))
                        fwd = args.get("direction", "fwd") == "fwd"
                        if m:
                            m.cmd_jog_start(Direction.FORWARD if fwd else Direction.REVERSE)
                elif cmd == "select_recipe":
                    try:
                        procedure.select_recipe(int(args.get("id")))
                    except (TypeError, ValueError):
                        pass
                elif cmd == "set_motor_params":
                    m = _motor_by_axis(args.get("axis", ""))
                    if m:
                        m.set_params(
                            max_speed    = args.get("max_speed"),
                            acceleration = args.get("acceleration"),
                            home_speed   = args.get("home_speed"),
                        )

        # ── Physical inputs ───────────────────────────────────────────────────
        mode   = _get_mode()
        inputs = hw.read_all_inputs()

        # Nouzový stop (jediný HV vstop) – náběžná hrana → ESTOP
        estop_in = _estop_active(inputs.get(config.CH_ESTOP_IN))
        if estop_in and not _estop_active(prev_inputs.get(config.CH_ESTOP_IN)):
            procedure.cmd_estop()
            _panel_jog.clear()
            _motor_power.clear()
            init_done = False
            log.warning("[ESTOP] nouzový stop")

        # STOP – funguje vždy, i v REMOTE
        if inputs.get(config.CH_STOP_BTN) and not prev_inputs.get(config.CH_STOP_BTN):
            procedure.cmd_stop()

        if mode == "LOCAL":
            # Volba receptury P1–P6 – náběžná hrana, jen v IDLE
            if procedure.step == ProcedureStep.IDLE:
                for rid, ch in config.CH_RECIPE_BTN.items():
                    if inputs.get(ch) and not prev_inputs.get(ch):
                        procedure.select_recipe(rid)

            # RUN – jen z panelu, jen v IDLE, jen při zapnutém napájení motorů
            if (inputs.get(config.CH_RUN_BTN) and not prev_inputs.get(config.CH_RUN_BTN)
                    and procedure.step == ProcedureStep.IDLE and _motor_power.is_set()):
                procedure.cmd_start()

        # Ruční jog z panelu – nezávislé na LOCAL/REMOTE, jen po povolení z webu
        # (_panel_jog) a jen v IDLE. Drž tlačítko = jeď; uvolnění = stop.
        pj_now = _panel_jog.is_set()
        if pj_now:
            jog_ok = procedure.step == ProcedureStep.IDLE
            for ch, get_motor, direction in _JOG_INPUTS:
                now_on, was_on = inputs.get(ch), prev_inputs.get(ch)
                if now_on and jog_ok:
                    get_motor().cmd_jog_start(direction)
                elif was_on and not now_on:
                    get_motor().cmd_jog_stop()
        elif pj_prev:
            # panel jog byl právě zakázán – zastav případný běžící pojezd
            for _, get_motor, _dir in _JOG_INPUTS:
                get_motor().cmd_jog_stop()
        pj_prev = pj_now

        # ── Inicializační homing po zapnutí napájení motorů ──────────────────
        if (_motor_power.is_set() and not init_done and _bridge.is_ready
                and t_start >= rel7_until
                and procedure.step in (ProcedureStep.IDLE, ProcedureStep.ESTOP)):
            if procedure.step == ProcedureStep.ESTOP:
                procedure.cmd_reset()
            _run_init_homing()
            init_done = True

        # ── Motor / procedure tick ────────────────────────────────────────────
        rot_motor.update()
        vert_motor.update()
        procedure.update()

        prev_inputs = inputs

        # ── Výstupy (8× relé) ───────────────────────────────────────────────
        step = procedure.step

        # relé 7 – RESET pulz stykačů napájení motorů
        hw.set_output(config.CH_MOTOR_PWR_RESET, t_start < rel7_until)

        # relé 8 – stavová kontrolka: sepnuto = CHYBA (červená), rozepnuto = OK (zelená)
        error = (step in (ProcedureStep.ESTOP, ProcedureStep.FAULT)
                 or rot_motor.is_error or vert_motor.is_error
                 or not _bridge.is_ready
                 or not _motor_power.is_set()
                 or estop_in)
        hw.set_output(config.CH_STATUS_ERR_RELAY, error)

        # ── State snapshot ────────────────────────────────────────────────────
        state: dict[str, Any] = {
            "arduino_connected": _bridge.is_ready,
            "mode":              mode,
            "procedure":         step.name,
            "status_ok":         not error,
            "fault_msg":         procedure.fault_msg,
            "motor_power":       _motor_power.is_set(),
            "estop_input":       estop_in,
            "recipe":            procedure.recipe.to_dict(),
            "recipe_id":         procedure.recipe_id,
            "recipes":           _RECIPES_DICT,
            "panel_jog":         _panel_jog.is_set(),
            "rot":               _motor_snapshot(rot_motor),
            "vert":              _motor_snapshot(vert_motor),
            "inputs":            {_INPUT_STR_KEYS[k]: v for k, v in inputs.items()},
            "outputs":           {str(k): v for k, v in hw.get_all_outputs().items()},
            "ups":               _get_ups_snapshot(),
        }

        with _state_lock:
            _current_state.clear()
            _current_state.update(state)

        _notify_subscribers("data: " + json.dumps(state) + "\n\n")

        elapsed = time.monotonic() - t_start
        sleep_t = config.LOOP_PERIOD - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)


threading.Thread(target=_control_loop, daemon=True, name="control-loop").start()

# ── Routes ────────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    with _state_lock:
        return jsonify(dict(_current_state))


@app.route("/api/events")
def api_events():
    q: queue.Queue = queue.Queue(maxsize=20)
    with _sse_lock:
        _sse_subscribers.append(q)

    def generate():
        try:
            while True:
                try:
                    yield q.get(timeout=5.0)
                except queue.Empty:
                    yield ": keepalive\n\n"
        finally:
            with _sse_lock:
                if q in _sse_subscribers:
                    _sse_subscribers.remove(q)

    return Response(generate(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.route("/api/cmd/<command>", methods=["POST"])
def api_command(command: str):
    allowed = {"start", "stop", "estop", "reset", "home", "jog", "jog_stop",
               "set_motor_params", "select_recipe", "claim", "release", "panel_jog"}
    if command not in allowed:
        return jsonify({"ok": False, "error": "neznámý příkaz"}), 400
    _cmd_queue.put((command, request.get_json(silent=True) or {}))
    return jsonify({"ok": True})


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print(f"SALT Manipulator → http://localhost:5000  (Arduino: {config.ARDUINO_PORT})")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
