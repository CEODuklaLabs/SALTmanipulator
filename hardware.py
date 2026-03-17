from config import SIMULATE


class HardwareInterface:
    """
    Thin wrapper over Sequent Microsystems lib16inpind / lib16relind.
    When SIMULATE=True, all calls are printed instead of executed.
    """

    def __init__(self, input_stack: int, output_stack: int):
        self._in_stack  = input_stack
        self._out_stack = output_stack
        self._sim_inputs: dict  = {}   # channel -> bool (simulation)
        self._sim_outputs: dict = {}   # channel -> bool (simulation)

        if not SIMULATE:
            import lib16inpind
            import lib16relind
            self._lib_in  = lib16inpind
            self._lib_out = lib16relind
        else:
            self._lib_in  = None
            self._lib_out = None

    # ── Inputs ────────────────────────────────────────────────────────────────

    def read_input(self, channel: int) -> bool:
        if SIMULATE:
            return self._sim_inputs.get(channel, False)
        return bool(self._lib_in.get(self._in_stack, channel))

    def read_all_inputs(self) -> dict:
        """Returns {channel: bool} for channels 1-16."""
        if SIMULATE:
            return {ch: self._sim_inputs.get(ch, False) for ch in range(1, 17)}
        raw = self._lib_in.get_all(self._in_stack)  # 16-bit int
        return {ch: bool(raw & (1 << (ch - 1))) for ch in range(1, 17)}

    def set_sim_input(self, channel: int, state: bool) -> None:
        """Simulation only: manually set an input channel state."""
        self._sim_inputs[channel] = state

    # ── Outputs ───────────────────────────────────────────────────────────────

    def set_output(self, channel: int, state: bool) -> None:
        if SIMULATE:
            prev = self._sim_outputs.get(channel)
            if prev != state:
                print(f"  [HW] OUT ch{channel:02d} = {'1' if state else '0'}")
            self._sim_outputs[channel] = state
            return
        self._lib_out.set(self._out_stack, channel, 1 if state else 0)

    def get_output(self, channel: int) -> bool:
        return self._sim_outputs.get(channel, False)
