import logging
import threading
import time

import config as _cfg

logger = logging.getLogger(__name__)

try:
    import smbus                        # type: ignore[import]
except ImportError:                      # pragma: no cover
    import smbus2 as smbus              # type: ignore[import]


# ─────────────────────────────────────────────────────────────────────────────
#  Přímý I²C přístup na desky Sequent Microsystems
# ─────────────────────────────────────────────────────────────────────────────
#  Vstup:  byte se čte z 'regs' (regs[0] = kanály 1–8, regs[1] = 9–16).
#  Výstup: drží se stínová kopie bytu na každý registr; při set_channel() se
#          změní jen dotčený bit a zapíše se celý byte.
#  Konfigurace desek: config.INPUT_BOARDS / config.OUTPUT_BOARDS.
# ─────────────────────────────────────────────────────────────────────────────

class _InputBoard:
    def __init__(self, bus, spec: dict):
        self._bus      = bus
        self._addr     = spec["addr"]
        self._regs     = spec["regs"]
        self._channels = spec["channels"]
        self._invert   = spec.get("invert", False)
        self._mask     = (1 << self._channels) - 1
        self._name     = spec.get("name", hex(self._addr))

    def read_all(self) -> int:
        val = 0
        for i, reg in enumerate(self._regs):
            b = self._bus.read_byte_data(self._addr, reg) & 0xFF
            val |= b << (8 * i)
        if self._invert:
            val = ~val
        return val & self._mask


class _OutputBoard:
    def __init__(self, bus, spec: dict):
        self._bus      = bus
        self._addr     = spec["addr"]
        self._regs     = spec["regs"]
        self._channels = spec["channels"]
        self._invert   = spec.get("invert", False)
        self._name     = spec.get("name", hex(self._addr))
        self._shadow   = [0] * len(self._regs)      # jeden byte na registr
        for i in range(len(self._regs)):
            self._flush(i)                          # výstupy do 0 při startu

    def _flush(self, i: int) -> None:
        b = self._shadow[i] & 0xFF
        if self._invert:
            b = (~b) & 0xFF
        self._bus.write_byte_data(self._addr, self._regs[i], b)

    def set_channel(self, channel: int, state: bool) -> None:
        """channel je 1-based v rámci této desky."""
        i   = (channel - 1) // 8
        bit = (channel - 1) % 8
        if i >= len(self._regs):
            return
        if state:
            self._shadow[i] |= (1 << bit)
        else:
            self._shadow[i] &= ~(1 << bit)
        self._flush(i)


# ─────────────────────────────────────────────────────────────────────────────
#  Veřejné rozhraní
# ─────────────────────────────────────────────────────────────────────────────

class HardwareInterface:
    """
    Hardwarové I/O přes I²C moduly Sequent Microsystems (dle config.INPUT_BOARDS
    a config.OUTPUT_BOARDS).  Kanály jsou číslované globálně přes všechny moduly
    daného směru (1. modul → 1..N, 2. modul → N+1.. atd.).

    Vstupní scan běží v pozadí a ukládá debouncovaný obraz vstupů (PAE);
    read_input() / read_all_inputs() čtou z něj.
    """

    def __init__(self):
        self._bus = smbus.SMBus(_cfg.I2C_BUS)

        # ── Vstupní moduly ─────────────────────────────────────────────────
        self._in_boards = []          # [(board, first_ch, channels), ...]
        ch0 = 1
        for spec in _cfg.INPUT_BOARDS:
            self._in_boards.append((_InputBoard(self._bus, spec), ch0, spec["channels"]))
            ch0 += spec["channels"]
        self._num_ch = ch0 - 1

        # ── Výstupní moduly ───────────────────────────────────────────────
        self._out_boards = []
        o0 = 1
        for spec in _cfg.OUTPUT_BOARDS:
            self._out_boards.append((_OutputBoard(self._bus, spec), o0, spec["channels"]))
            o0 += spec["channels"]
        self._num_out = o0 - 1

        empty = {ch: False for ch in range(1, self._num_ch + 1)}
        self._out_mirror: dict  = {}
        self._input_image: dict = dict(empty)          # debouncovaný obraz (PAE)
        self._raw_prev: dict    = dict(empty)
        self._stable_cnt: dict  = {ch: 0 for ch in range(1, self._num_ch + 1)}
        self._debounce_n  = max(1, int(_cfg.INPUT_DEBOUNCE_SCANS))
        self._image_lock  = threading.Lock()
        self._scan_thread: threading.Thread | None = None
        self._scan_stop   = threading.Event()

    @property
    def num_channels(self) -> int:
        return self._num_ch

    @property
    def num_output_channels(self) -> int:
        return self._num_out

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
        logger.info("[HW] vstupní scan spuštěn (%d kanálů, %.0f ms)",
                    self._num_ch, interval_s * 1000)

    def stop_input_scan(self) -> None:
        self._scan_stop.set()
        if self._scan_thread:
            self._scan_thread.join(timeout=2.0)
            self._scan_thread = None

    def _scan_loop(self, interval_s: float) -> None:
        while not self._scan_stop.is_set():
            t0 = time.monotonic()
            try:
                self._debounce_step(self._read_hw_inputs())
            except Exception as exc:
                logger.warning("[HW] Chyba čtení vstupů: %s", exc)

            wait = interval_s - (time.monotonic() - t0)
            if wait > 0:
                self._scan_stop.wait(timeout=wait)

    def _debounce_step(self, raw: dict) -> None:
        """Kanál se do obrazu propíše až po INPUT_DEBOUNCE_SCANS shodných čteních."""
        updated = {}
        for ch, val in raw.items():
            if val == self._raw_prev[ch]:
                if self._stable_cnt[ch] < self._debounce_n:
                    self._stable_cnt[ch] += 1
            else:
                self._stable_cnt[ch] = 1
            self._raw_prev[ch] = val
            if (self._stable_cnt[ch] >= self._debounce_n
                    and self._input_image[ch] != val):
                updated[ch] = val

        if updated:
            with self._image_lock:
                new_image = dict(self._input_image)
                new_image.update(updated)
                self._input_image = new_image

    def _read_hw_inputs(self) -> dict:
        result = {}
        for board, first_ch, n in self._in_boards:
            raw = board.read_all()
            for i in range(n):
                result[first_ch + i] = bool(raw & (1 << i))
        return result

    # ── Inputs ────────────────────────────────────────────────────────────────

    def read_input(self, channel) -> bool:
        if channel is None:
            return False
        with self._image_lock:
            return self._input_image.get(channel, False)

    def read_all_inputs(self) -> dict:
        with self._image_lock:
            return dict(self._input_image)

    # ── Outputs ───────────────────────────────────────────────────────────────

    def _out_target(self, channel: int):
        for board, first_ch, n in self._out_boards:
            if first_ch <= channel < first_ch + n:
                return board, channel - first_ch + 1
        return None, None

    def set_output(self, channel: int, state: bool) -> None:
        state = bool(state)
        if self._out_mirror.get(channel) == state:
            return
        board, local = self._out_target(channel)
        if board is None:
            logger.warning("[HW] neznámý výstupní kanál %s", channel)
            return
        self._out_mirror[channel] = state
        try:
            board.set_channel(local, state)
        except Exception as exc:
            logger.warning("[HW] Chyba zápisu výstupu %s: %s", channel, exc)

    def get_output(self, channel: int) -> bool:
        return self._out_mirror.get(channel, False)

    def get_all_outputs(self) -> dict:
        """Vrátí kopii výstupního zrcadla {channel: bool}."""
        return dict(self._out_mirror)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup(self) -> None:
        self.stop_input_scan()
