import logging
import threading
import time

import SM8relind    # type: ignore[import]
import SM16inpind   # type: ignore[import]
import SM16relind   # type: ignore[import]
import config as _cfg

logger = logging.getLogger(__name__)

_EMPTY_IMAGE: dict = {ch: False for ch in range(1, 25)}


class HardwareInterface:
    """
    Hardwarové I/O přes Sequent Microsystems desky (I²C).

    Vstupní scan:
      Pozadí vlákno čte vstupy v pravidelném intervalu a ukládá je do
      vstupního obrazu (_input_image) – analogie PAE u Simatic PLC.
      read_input() / read_all_inputs() čtou z tohoto obrazu.
    """

    def __init__(
        self,
        input_stack1:  int = _cfg.INPUT_BOARD1_STACK,
        output_stack1: int = _cfg.OUTPUT_BOARD1_STACK,
        output_stack2: int = _cfg.OUTPUT_BOARD2_STACK,
        input_stack2:  int = _cfg.INPUT_BOARD2_STACK,
    ):
        self._in_stack   = input_stack1
        self._out_stack  = output_stack1
        self._in_stack2  = input_stack2
        self._out_stack2 = output_stack2
        self._out_mirror: dict = {}
        self._input_image: dict = dict(_EMPTY_IMAGE)
        self._image_lock  = threading.Lock()
        self._scan_thread: threading.Thread | None = None
        self._scan_stop   = threading.Event()

    # ── Input scan ────────────────────────────────────────────────────────────

    def start_input_scan(self, interval_s: float = _cfg.INPUT_SCAN_INTERVAL_S) -> None:
        if self._scan_thread and self._scan_thread.is_alive():
            return
        self._scan_stop.clear()
        self._scan_thread = threading.Thread(
            target=self._scan_loop, args=(interval_s,),
            daemon=True, name="hw-input-scan",
        )
        self._scan_thread.start()
        logger.info("[HW] vstupní scan spuštěn (%.0f ms)", interval_s * 1000)

    def stop_input_scan(self) -> None:
        self._scan_stop.set()
        if self._scan_thread:
            self._scan_thread.join(timeout=2.0)
            self._scan_thread = None

    def _scan_loop(self, interval_s: float) -> None:
        while not self._scan_stop.is_set():
            t0 = time.monotonic()
            try:
                new_image = self._read_hw_inputs()
                # Acquire lock only when inputs actually changed
                if new_image != self._input_image:
                    with self._image_lock:
                        self._input_image = new_image
            except Exception as exc:
                logger.warning("[HW] Chyba čtení vstupů: %s", exc)

            wait = interval_s - (time.monotonic() - t0)
            if wait > 0:
                self._scan_stop.wait(timeout=wait)

    def _read_hw_inputs(self) -> dict:
        raw1 = SM8relind.get_all(self._in_stack)
        raw2 = SM16inpind.get_all(self._in_stack2)
        result = {ch: bool(raw1 & (1 << (ch - 1))) for ch in range(1, 9)}
        result.update({ch + 8: bool(raw2 & (1 << (ch - 1))) for ch in range(1, 17)})
        return result

    # ── Inputs ────────────────────────────────────────────────────────────────

    def read_input(self, channel: int) -> bool:
        if self._scan_thread and self._scan_thread.is_alive():
            with self._image_lock:
                return self._input_image.get(channel, False)
        if channel <= 8:
            return bool(SM8relind.get(self._in_stack, channel))
        return bool(SM16inpind.get(self._in_stack2, channel - 8))

    def read_all_inputs(self) -> dict:
        if self._scan_thread and self._scan_thread.is_alive():
            with self._image_lock:
                return dict(self._input_image)
        return self._read_hw_inputs()

    # ── Outputs ───────────────────────────────────────────────────────────────

    def set_output(self, channel: int, state: bool) -> None:
        self._out_mirror[channel] = state
        SM16relind.set(self._out_stack, channel, 1 if state else 0)

    def get_output(self, channel: int) -> bool:
        return self._out_mirror.get(channel, False)

    def get_all_outputs(self) -> dict:
        """Vrátí kopii výstupního zrcadla {channel: bool}."""
        return dict(self._out_mirror)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup(self) -> None:
        self.stop_input_scan()
