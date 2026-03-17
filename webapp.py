"""
SALT Manipulator – web dashboard
Run: python webapp.py
Then open http://localhost:5000 in a browser.

Requires Flask:  pip install flask
"""

import json
import queue
import threading
import time
from typing import Any

from flask import Flask, Response, jsonify, render_template, request

import config
from hardware import HardwareInterface
from motor import Direction, MotorController
from procedure import ProcedureStateMachine, ProcedureStep

# ── Flask app ─────────────────────────────────────────────────────────────────

app = Flask(__name__)

# ── Shared control objects ────────────────────────────────────────────────────

hw = HardwareInterface(config.INPUT_BOARD1_STACK, config.OUTPUT_BOARD1_STACK)

rot_motor = MotorController(
    name               = "ROT",
    hw                 = hw,
    pulse_ch           = config.CH_PULSE_ROT,
    dir_ch             = config.CH_DIR_ROT,
    enable_ch          = config.CH_ENABLE_ROT,
    home_input_ch      = config.CH_HOME_ROT,
    pulse_hz           = config.ROT_PULSE_HZ,
    home_backoff_steps = config.ROT_HOME_BACKOFF,
    homing_timeout_s   = config.HOMING_TIMEOUT_S,
)

vert_motor = MotorController(
    name               = "VERT",
    hw                 = hw,
    pulse_ch           = config.CH_PULSE_VERT,
    dir_ch             = config.CH_DIR_VERT,
    enable_ch          = config.CH_ENABLE_VERT,
    home_input_ch      = config.CH_HOME_VERT,
    pulse_hz           = config.VERT_PULSE_HZ,
    home_backoff_steps = config.VERT_HOME_BACKOFF,
    homing_timeout_s   = config.HOMING_TIMEOUT_S,
)

procedure = ProcedureStateMachine(rot_motor, vert_motor)

# ── Thread-safe command queue & state ─────────────────────────────────────────

_cmd_queue: queue.Queue = queue.Queue()
_state_lock = threading.Lock()
_current_state: dict[str, Any] = {}
_mode_error: str = ""          # poslední chybová zpráva při set_mode

# SSE subscribers
_sse_subscribers: list[queue.Queue] = []
_sse_lock = threading.Lock()


def _notify_subscribers(state: dict) -> None:
    data = "data: " + json.dumps(state) + "\n\n"
    with _sse_lock:
        dead = []
        for q in _sse_subscribers:
            try:
                q.put_nowait(data)
            except queue.Full:
                dead.append(q)
        for q in dead:
            _sse_subscribers.remove(q)


# ── Control loop (background thread) ─────────────────────────────────────────

def _control_loop() -> None:
    global _mode_error
    prev_inputs: dict[int, bool] = {ch: False for ch in range(1, 17)}

    while True:
        t_start = time.monotonic()

        # ── Process web commands ───────────────────────────────────────────────
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
                    fwd = args.get("direction", "fwd") == "fwd"
                    direction = Direction.FORWARD if fwd else Direction.REVERSE
                    if axis == "rot":
                        rot_motor.cmd_jog(direction)
                    elif axis == "vert":
                        vert_motor.cmd_jog(direction)

            elif cmd == "sim_input":
                ch = int(args.get("channel", 0))
                state = bool(args.get("state", False))
                if 1 <= ch <= 16:
                    hw.set_sim_input(ch, state)

            elif cmd == "set_mode":
                new_mode = args.get("mode", "simulate")
                try:
                    procedure.cmd_stop()
                    rot_motor.cmd_disable()
                    vert_motor.cmd_disable()
                    hw.set_mode(new_mode)
                    _mode_error = ""
                except Exception as exc:
                    _mode_error = str(exc)
                    print(f"[WEBAPP] set_mode failed: {exc}")

        # ── Read hardware inputs ───────────────────────────────────────────────
        inputs = hw.read_all_inputs()

        # STOP button (rising edge)
        if inputs[config.CH_STOP_BTN] and not prev_inputs[config.CH_STOP_BTN]:
            procedure.cmd_stop()

        # RUN button (rising edge, IDLE only)
        if (inputs[config.CH_RUN_BTN] and not prev_inputs[config.CH_RUN_BTN]
                and procedure.step == ProcedureStep.IDLE):
            procedure.cmd_start()

        # Manual jog (IDLE only)
        if procedure.step == ProcedureStep.IDLE:
            if inputs[config.CH_JOG_ROT_FWD]:
                rot_motor.cmd_jog(Direction.FORWARD)
            elif inputs[config.CH_JOG_ROT_REV]:
                rot_motor.cmd_jog(Direction.REVERSE)
            if inputs[config.CH_JOG_VERT_UP]:
                vert_motor.cmd_jog(Direction.FORWARD)
            elif inputs[config.CH_JOG_VERT_DOWN]:
                vert_motor.cmd_jog(Direction.REVERSE)

        # ── Motor & procedure ticks ────────────────────────────────────────────
        rot_motor.update()
        vert_motor.update()
        procedure.update()

        prev_inputs = inputs

        # ── Build shared state snapshot ────────────────────────────────────────
        outputs = {ch: hw.get_output(ch) for ch in range(1, 17)}
        state: dict[str, Any] = {
            "hw_mode":    hw.mode,
            "mode_error": _mode_error,
            "simulate":   hw.mode == "simulate",
            "procedure": procedure.step.name,
            "rot": {
                "state":    rot_motor.state.name,
                "position": rot_motor.position,
                "target":   rot_motor._target,
            },
            "vert": {
                "state":    vert_motor.state.name,
                "position": vert_motor.position,
                "target":   vert_motor._target,
            },
            "inputs":  {str(k): v for k, v in inputs.items()},
            "outputs": {str(k): v for k, v in outputs.items()},
        }

        with _state_lock:
            _current_state.clear()
            _current_state.update(state)

        _notify_subscribers(state)

        # ── Pace loop ──────────────────────────────────────────────────────────
        elapsed = time.monotonic() - t_start
        sleep_t = config.LOOP_PERIOD - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)


_control_thread = threading.Thread(target=_control_loop, daemon=True)
_control_thread.start()

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
    """Server-Sent Events stream – real-time updates."""
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
                    headers={"Cache-Control": "no-cache",
                             "X-Accel-Buffering": "no"})


@app.route("/api/cmd/<command>", methods=["POST"])
def api_command(command: str):
    allowed = {"start", "stop", "estop", "reset", "home", "jog",
               "sim_input", "set_mode"}
    if command not in allowed:
        return jsonify({"ok": False, "error": "unknown command"}), 400
    args = request.get_json(silent=True) or {}
    _cmd_queue.put((command, args))
    return jsonify({"ok": True})


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("SALT Manipulator web UI -> http://localhost:5000")
    print(f"HARDWARE_MODE = {config.HARDWARE_MODE}")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
