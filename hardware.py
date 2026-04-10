import SM8relind    # type: ignore[import]
import SM16inpind   # type: ignore[import]
import SM16relind   # type: ignore[import]
import config as _cfg


class HardwareInterface:
    """
    Hardwarové I/O přes Sequent Microsystems SM8relind + SM16inpind (vstupy) + SM16relind (výstupy) desky (I²C).
    """

    def __init__(self,
                 input_stack1: int = _cfg.INPUT_BOARD1_STACK,
                 output_stack1: int = _cfg.OUTPUT_BOARD1_STACK,
                 output_stack2: int = _cfg.OUTPUT_BOARD2_STACK,
                 input_stack2: int = _cfg.INPUT_BOARD2_STACK):
        self._in_stack   = input_stack1
        self._out_stack  = output_stack1
        self._in_stack2  = input_stack2
        self._out_stack2 = output_stack2
        self._out_mirror: dict = {}

    # ── Inputs ────────────────────────────────────────────────────────────────

    def read_input(self, channel: int) -> bool:
        """Kanály 1–8: SM8relind (stack1), kanály 9–24: SM16inpind (stack2, offset 8)."""
        if channel <= 8:
            return bool(SM8relind.get(self._in_stack, channel))
        return bool(SM16inpind.get(self._in_stack2, channel - 8))

    def read_all_inputs(self) -> dict:
        """Vrátí {channel: bool}: 1–8 z SM8relind (stack1), 9–24 z SM16inpind (stack2)."""
        raw1 = SM8relind.get_all(self._in_stack)
        raw2 = SM16inpind.get_all(self._in_stack2)
        result = {ch: bool(raw1 & (1 << (ch - 1))) for ch in range(1, 9)}
        result.update({ch + 8: bool(raw2 & (1 << (ch - 1))) for ch in range(1, 17)})
        return result

    # ── Outputs ───────────────────────────────────────────────────────────────

    def set_output(self, channel: int, state: bool) -> None:
        self._out_mirror[channel] = state
        SM16relind.set(self._out_stack, channel, 1 if state else 0)

    def get_output(self, channel: int) -> bool:
        return self._out_mirror.get(channel, False)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup(self) -> None:
        pass
