"""
UPS Monitor – Suptronics X1205 UPS Shield pro Raspberry Pi 5.

Hardware:
  - MAX17048 fuel gauge  →  I2C adresa 0x36, registr 0x02 (napětí), 0x04 (kapacita)
  - GPIO pin 6 (gpiochip0)  →  HIGH = AC přítomno, LOW = AC odpojeno

Logika vypnutí:
  1. Ztráta AC + kapacita < UPS_LOW_BAT_THRESHOLD  →  varování, po UPS_SHUTDOWN_DELAY_S sekundách
     ESTOP + system shutdown
  2. Napětí < UPS_CRITICAL_VOLTAGE_V  →  okamžité vypnutí bez prodlevy
"""

import logging
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

logger = logging.getLogger(__name__)

try:
    import smbus2 as smbus_mod
    _SMBUS_OK = True
except ImportError:
    _SMBUS_OK = False
    logger.debug("smbus2 není dostupný – UPS nebude číst data")

try:
    import gpiod
    _GPIOD_OK = True
except ImportError:
    _GPIOD_OK = False
    logger.debug("gpiod není dostupný – AC status bude neznámý")

_FUEL_GAUGE_ADDR = 0x36
_REG_VCELL       = 0x02   # 16-bit big-endian, LSB = 1.25/16 mV
_REG_SOC         = 0x04   # 16-bit big-endian, MSB = %, LSB = 1/256 %


def _byteswap16(raw: int) -> int:
    """SMBus read_word_data vrací little-endian, MAX17048 posílá big-endian."""
    return ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF)


@dataclass
class UPSStatus:
    voltage:     float          = 0.0
    capacity:    float          = 0.0
    ac_present:  bool           = True
    charging:    bool           = False
    warning:     bool           = False
    shutdown_in: Optional[float] = None
    available:   bool           = False
    error:       str            = ""

    def to_dict(self) -> dict:
        return {
            "voltage":     round(self.voltage, 3),
            "capacity":    round(self.capacity, 1),
            "ac_present":  self.ac_present,
            "charging":    self.charging,
            "warning":     self.warning,
            "shutdown_in": round(self.shutdown_in, 1) if self.shutdown_in is not None else None,
            "available":   self.available,
            "error":       self.error,
        }


class UPSMonitor:
    """Monitoruje stav UPS X1205 v samostatném vlákně."""

    def __init__(
        self,
        cfg,
        estop_callback: Optional[Callable[[], None]] = None,
        ac_loss_callback: Optional[Callable[[], None]] = None,
        enabled: Optional[bool] = None,
    ) -> None:
        self._cfg       = cfg
        self._estop_cb  = estop_callback
        self._ac_loss_cb = ac_loss_callback   # volá se jednou při hraně ztráty AC (total-stop)
        self._enabled   = enabled if enabled is not None else getattr(cfg, "UPS_ENABLED", True)
        self._status    = UPSStatus()
        self._lock      = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._ac_lost_since: Optional[float] = None
        self._shutdown_triggered = False
        self._bus        = None
        self._gpio_chip  = None
        self._gpio_line  = None

    def start(self) -> None:
        if not self._enabled:
            logger.info("[UPS] monitoring vypnut")
            return
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="ups-monitor")
        self._thread.start()
        logger.info("[UPS] vlákno spuštěno")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=5.0)
        self._cleanup_hw()

    def get_status(self) -> dict:
        with self._lock:
            return self._status.to_dict()

    # ── Hardware init / cleanup ───────────────────────────────────────────────

    def _init_hw(self) -> bool:
        ok = True
        if _SMBUS_OK:
            try:
                self._bus = smbus_mod.SMBus(self._cfg.UPS_I2C_BUS)
            except Exception as exc:
                logger.warning("[UPS] Nelze otevřít I2C bus %d: %s", self._cfg.UPS_I2C_BUS, exc)
                ok = False
        else:
            ok = False

        if _GPIOD_OK:
            try:
                self._gpio_chip = gpiod.Chip(self._cfg.UPS_AC_GPIO_CHIP)
                self._gpio_line = self._gpio_chip.get_line(self._cfg.UPS_AC_GPIO_PIN)
                self._gpio_line.request(consumer="ups-monitor", type=gpiod.LINE_REQ_DIR_IN)
            except Exception as exc:
                logger.warning("[UPS] Nelze otevřít GPIO pin %d: %s",
                               self._cfg.UPS_AC_GPIO_PIN, exc)

        return ok

    def _cleanup_hw(self) -> None:
        for obj, method in [
            (self._gpio_line, "release"),
            (self._gpio_chip, "close"),
            (self._bus,       "close"),
        ]:
            if obj:
                try:
                    getattr(obj, method)()
                except Exception:
                    pass
        self._bus = self._gpio_chip = self._gpio_line = None

    # ── Sensor reads ─────────────────────────────────────────────────────────

    def _read_voltage(self) -> float:
        return _byteswap16(self._bus.read_word_data(_FUEL_GAUGE_ADDR, _REG_VCELL)) * 1.25 / 16000.0

    def _read_capacity(self) -> float:
        return _byteswap16(self._bus.read_word_data(_FUEL_GAUGE_ADDR, _REG_SOC)) / 256.0

    def _read_ac(self) -> bool:
        if self._gpio_line is None:
            return True
        try:
            return self._gpio_line.get_value() == 1
        except Exception:
            return True

    # ── Monitor loop ──────────────────────────────────────────────────────────

    def _run(self) -> None:
        hw_ok = self._init_hw()
        if not hw_ok:
            logger.warning("[UPS] hardware nedostupný")

        while not self._stop_event.is_set():
            status = UPSStatus(available=hw_ok)

            if hw_ok and self._bus:
                try:
                    status.voltage    = self._read_voltage()
                    status.capacity   = self._read_capacity()
                    status.ac_present = self._read_ac()
                    status.charging   = status.ac_present and status.capacity < 99.0
                except Exception as exc:
                    status.error     = str(exc)
                    status.available = False
                    logger.warning("[UPS] Chyba čtení: %s", exc)

            if status.available:
                self._evaluate_shutdown(status)

            with self._lock:
                self._status = status

            self._stop_event.wait(timeout=self._cfg.UPS_POLL_INTERVAL_S)

        self._cleanup_hw()

    def _evaluate_shutdown(self, status: UPSStatus) -> None:
        now = time.monotonic()

        if status.voltage > 0 and status.voltage < self._cfg.UPS_CRITICAL_VOLTAGE_V:
            status.warning = True
            if not self._shutdown_triggered:
                logger.critical("[UPS] Kritické napětí %.3fV – nouzové vypnutí!", status.voltage)
                self._trigger_shutdown(status, immediate=True)
            return

        if not status.ac_present:
            if self._ac_lost_since is None:
                self._ac_lost_since = now
                logger.warning("[UPS] AC odpojeno (total-stop?). Baterie: %.1f%%", status.capacity)
                if self._ac_loss_cb:
                    try:
                        self._ac_loss_cb()
                    except Exception as exc:
                        logger.error("[UPS] ac_loss callback selhal: %s", exc)

            status.warning = True
            if status.capacity < self._cfg.UPS_LOW_BAT_THRESHOLD:
                remaining = self._cfg.UPS_SHUTDOWN_DELAY_S - (now - self._ac_lost_since)
                if remaining > 0:
                    status.shutdown_in = remaining
                    logger.warning("[UPS] Nízká baterie – vypnutí za %.0fs", remaining)
                elif not self._shutdown_triggered:
                    logger.critical("[UPS] Timeout nízké baterie – nouzové vypnutí!")
                    self._trigger_shutdown(status, immediate=False)
        else:
            if self._ac_lost_since is not None:
                logger.info("[UPS] AC obnoveno")
            self._ac_lost_since      = None
            self._shutdown_triggered = False

    def _trigger_shutdown(self, status: UPSStatus, immediate: bool) -> None:
        self._shutdown_triggered = True
        status.warning     = True
        status.shutdown_in = 0.0

        if self._estop_cb:
            try:
                self._estop_cb()
            except Exception as exc:
                logger.error("[UPS] ESTOP callback selhal: %s", exc)

        cmd = ["sudo", "shutdown", "-h", "now" if immediate else "+0"]
        logger.critical("[UPS] System shutdown: %s", " ".join(cmd))
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:
            logger.error("[UPS] Nelze spustit shutdown: %s", exc)
