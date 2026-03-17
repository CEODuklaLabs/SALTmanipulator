import config as _cfg


class HardwareInterface:
    """
    Abstrakce hardwarového I/O.  Podporuje čtyři backendy:

      "simulate" – vše v paměti, žádný reálný hardware
      "i2c"      – Sequent Microsystems 16DI + 16DO desky (I²C)
      "gpio"     – přímé GPIO piny Raspberry Pi (RPi.GPIO, BCM)
      "mixed"    – kanály v GPIO_*_PINS → GPIO, ostatní → I²C

    Mód lze přepnout za běhu přes set_mode().
    """

    MODES = ("simulate", "i2c", "gpio", "mixed")

    def __init__(self, input_stack: int = _cfg.INPUT_BOARD1_STACK,
                 output_stack: int = _cfg.OUTPUT_BOARD1_STACK):
        self._in_stack   = input_stack
        self._out_stack  = output_stack
        self._mode: str  = _cfg.HARDWARE_MODE
        self._out_mirror: dict = {}
        self._sim_inputs: dict = {}
        self._lib_in  = None
        self._lib_out = None
        self._gpio    = None
        self._init_backends()

    # ── Backend init / teardown ───────────────────────────────────────────────

    def _init_backends(self) -> None:
        use_i2c  = self._mode in ("i2c",  "mixed")
        use_gpio = self._mode in ("gpio", "mixed")

        if use_i2c:
            import lib16inpind   # type: ignore[import]
            import lib16relind   # type: ignore[import]
            self._lib_in  = lib16inpind
            self._lib_out = lib16relind

        if use_gpio:
            import RPi.GPIO as GPIO  # type: ignore[import]
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            pull = (GPIO.PUD_UP   if _cfg.GPIO_INPUT_PULL == "up"   else
                    GPIO.PUD_DOWN if _cfg.GPIO_INPUT_PULL == "down" else
                    GPIO.PUD_OFF)

            for pin in _cfg.GPIO_INPUT_PINS.values():
                GPIO.setup(pin, GPIO.IN, pull_up_down=pull)
            for pin in _cfg.GPIO_OUTPUT_PINS.values():
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

            self._gpio = GPIO

    def _cleanup_backends(self) -> None:
        if self._gpio is not None:
            self._gpio.cleanup()
            self._gpio = None
        self._lib_in  = None
        self._lib_out = None

    def set_mode(self, mode: str) -> None:
        """Přepne hardware backend za běhu. Bezpečné volat z control loopy.
        Při selhání inicializace se automaticky vrátí do simulate módu."""
        if mode not in self.MODES:
            raise ValueError(f"Neznámý mód: {mode!r}. Povolené: {self.MODES}")
        if mode == self._mode:
            return
        self._cleanup_backends()
        self._mode = mode
        try:
            self._init_backends()
            print(f"[HW] Mód přepnut na: {mode}")
        except Exception as exc:
            print(f"[HW] Inicializace módu '{mode}' selhala: {exc}")
            print("[HW] Záloha: přepínám zpět na simulate")
            self._cleanup_backends()
            self._mode = "simulate"
            # simulate nemá žádné importy, _init_backends() pro něj nic nedělá

    @property
    def mode(self) -> str:
        return self._mode

    # ── Inputs ────────────────────────────────────────────────────────────────

    def read_input(self, channel: int) -> bool:
        if self._mode == "simulate":
            return self._sim_inputs.get(channel, False)

        if self._gpio is not None and channel in _cfg.GPIO_INPUT_PINS:
            raw = self._gpio.input(_cfg.GPIO_INPUT_PINS[channel])
            return (not raw) if _cfg.GPIO_INPUT_PULL == "up" else bool(raw)

        if self._lib_in is not None:
            return bool(self._lib_in.get(self._in_stack, channel))

        return False

    def read_all_inputs(self) -> dict:
        """Vrátí {channel: bool} pro kanály 1–16."""
        if self._mode == "simulate":
            return {ch: self._sim_inputs.get(ch, False) for ch in range(1, 17)}

        if self._mode == "gpio":
            return {ch: self.read_input(ch) for ch in range(1, 17)}

        if self._mode in ("i2c", "mixed"):
            raw = self._lib_in.get_all(self._in_stack)
            result = {ch: bool(raw & (1 << (ch - 1))) for ch in range(1, 17)}
            if self._mode == "mixed":
                for ch in _cfg.GPIO_INPUT_PINS:
                    result[ch] = self.read_input(ch)
            return result

        return {ch: False for ch in range(1, 17)}

    def set_sim_input(self, channel: int, state: bool) -> None:
        """Nastav simulovaný/override stav vstupu (funguje ve všech módech)."""
        self._sim_inputs[channel] = state

    # ── Outputs ───────────────────────────────────────────────────────────────

    def set_output(self, channel: int, state: bool) -> None:
        self._out_mirror[channel] = state

        if self._mode == "simulate":
            return

        if self._gpio is not None and channel in _cfg.GPIO_OUTPUT_PINS:
            import RPi.GPIO as GPIO  # type: ignore[import]
            self._gpio.output(_cfg.GPIO_OUTPUT_PINS[channel],
                              GPIO.HIGH if state else GPIO.LOW)
            return

        if self._lib_out is not None:
            self._lib_out.set(self._out_stack, channel, 1 if state else 0)

    def get_output(self, channel: int) -> bool:
        return self._out_mirror.get(channel, False)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup(self) -> None:
        self._cleanup_backends()
