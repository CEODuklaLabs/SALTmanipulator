"""
SALT Manipulator – web dashboard
Run: python webapp.py  →  http://localhost:5000
"""

import json
import queue
import threading
import time
from typing import Any

from flask import Flask, Response, jsonify, render_template, request

import config
from hardware import HardwareInterface
from motor import ArduinoMotorController, Direction, SerialBridge
from procedure import ProcedureStateMachine, ProcedureStep
from ups import UPSMonitor

app = Flask(__name__)

# ── Hardware ──────────────────────────────────────────────────────────────────

hw = HardwareInterface(config.INPUT_BOARD1_STACK, config.OUTPUT_BOARD1_STACK)
hw.start_input_scan()

# ── Arduino serial bridge ─────────────────────────────────────────────────────

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
)

procedure = ProcedureStateMachine(rot_motor, vert_motor)

# ── UPS ───────────────────────────────────────────────────────────────────────

ups = UPSMonitor(config, estop_callback=procedure.cmd_estop)
ups.start()

# ── SSE / state ───────────────────────────────────────────────────────────────

_cmd_queue: queue.Queue        = queue.Queue()
_state_lock                    = threading.Lock()
_current_state: dict[str, Any] = {}
_sse_subscribers: list[queue.Queue] = []
_sse_lock                      = threading.Lock()

# Pre-computed str keys to avoid repeated str() calls in the hot loop
_INPUT_STR_KEYS = {ch: str(ch) for ch in range(1, 25)}

# UPS cache – read from UPS thread every 10 s, no need to re-read every 5 ms
_ups_cache: dict       = {}
_ups_cache_lock        = threading.Lock()
_ups_cache_updated     = threading.Event()


def _ups_cache_updater() -> None:
    """Separate thread that refreshes the UPS dict at the UPS poll rate."""
    while True:
        fresh = ups.get_status()
        with _ups_cache_lock:
            _ups_cache.clear()
            _ups_cache.update(fresh)
        _ups_cache_updated.set()
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
        "state":    mc.state.name,
        "position": mc.position,
        "target":   mc.target,
        "params": {
            "max_speed":    mc.max_speed,
            "acceleration": mc.acceleration,
            "home_speed":   mc.home_speed,
        },
    }


# ── Control loop (200 Hz) ─────────────────────────────────────────────────────

def _control_loop() -> None:
    prev_inputs: dict[int, bool] = {ch: False for ch in range(1, 25)}

    while True:
        t_start = time.monotonic()

        # ── Web commands ──────────────────────────────────────────────────────
        while not _cmd_queue.empty():
            try:
                cmd, args = _cmd_queue.get_nowait()
            except queue.Empty:
                break

            if cmd == "start":
                if procedure.step == ProcedureStep.IDLE:
                    procedure.cmd_start()
            elif cmd == "stop":
                procedure.cmd_stop()
            elif cmd == "estop":
                procedure.cmd_estop()
            elif cmd == "reset":
                procedure.cmd_reset()
            elif cmd == "home":
                axis = args.get("axis", "both")
                if axis in ("rot", "both"):
                    rot_motor.cmd_home()
                if axis in ("vert", "both"):
                    vert_motor.cmd_home()
            elif cmd == "jog":
                if procedure.step == ProcedureStep.IDLE:
                    axis = args.get("axis", "")
                    fwd  = args.get("direction", "fwd") == "fwd"
                    direction = Direction.FORWARD if fwd else Direction.REVERSE
                    if axis == "rot":
                        rot_motor.cmd_jog(direction)
                    elif axis == "vert":
                        vert_motor.cmd_jog(direction)
            elif cmd == "set_motor_params":
                axis  = args.get("axis", "")
                motor = rot_motor if axis == "rot" else (vert_motor if axis == "vert" else None)
                if motor:
                    motor.set_params(
                        max_speed    = args.get("max_speed"),
                        acceleration = args.get("acceleration"),
                        home_speed   = args.get("home_speed"),
                    )

        # ── Physical inputs ───────────────────────────────────────────────────
        inputs = hw.read_all_inputs()

        if inputs.get(config.CH_STOP_BTN) and not prev_inputs.get(config.CH_STOP_BTN):
            procedure.cmd_stop()

        if (inputs.get(config.CH_RUN_BTN) and not prev_inputs.get(config.CH_RUN_BTN)
                and procedure.step == ProcedureStep.IDLE):
            procedure.cmd_start()

        if procedure.step == ProcedureStep.IDLE:
            if inputs.get(config.CH_JOG_ROT_FWD):
                rot_motor.cmd_jog(Direction.FORWARD)
            elif inputs.get(config.CH_JOG_ROT_REV):
                rot_motor.cmd_jog(Direction.REVERSE)
            if inputs.get(config.CH_JOG_VERT_UP):
                vert_motor.cmd_jog(Direction.FORWARD)
            elif inputs.get(config.CH_JOG_VERT_DOWN):
                vert_motor.cmd_jog(Direction.REVERSE)

        # ── Motor / procedure tick ────────────────────────────────────────────
        rot_motor.update()
        vert_motor.update()
        procedure.update()

        prev_inputs = inputs

        # ── State snapshot ────────────────────────────────────────────────────
        state: dict[str, Any] = {
            "arduino_connected": _bridge.is_ready,
            "procedure":         procedure.step.name,
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
    allowed = {"start", "stop", "estop", "reset", "home", "jog", "set_motor_params"}
    if command not in allowed:
        return jsonify({"ok": False, "error": "neznámý příkaz"}), 400
    _cmd_queue.put((command, request.get_json(silent=True) or {}))
    return jsonify({"ok": True})


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print(f"SALT Manipulator → http://localhost:5000  (Arduino: {config.ARDUINO_PORT})")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
