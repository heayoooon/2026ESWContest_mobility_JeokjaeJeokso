"""
Minimal HX711 driver using lgpio.

Works on Raspberry Pi 4 and Pi 5 (Bookworm), where the old RPi.GPIO is
deprecated. Only depends on `lgpio` (pip install lgpio, or
sudo apt install python3-lgpio).

HX711 is a 24-bit ADC for load cells. This driver bit-bangs the two-wire
protocol: pulse SCK, read DOUT for 24 bits, then extra pulses to select the
gain/channel for the next reading.
"""

import time
import statistics
import lgpio


class HX711:
    # gain -> number of extra clock pulses after the 24 data bits
    _GAIN_PULSES = {128: 1, 64: 3, 32: 2}  # 128/64 = channel A, 32 = channel B

    def __init__(self, dout_pin, sck_pin, gain=128, chip=None):
        self.dout_pin = dout_pin
        self.sck_pin = sck_pin
        self._gain_pulses = self._GAIN_PULSES[gain]

        # gpiochip4 is the header on Pi 5; gpiochip0 on Pi 4 and earlier.
        if chip is None:
            for candidate in (4, 0):
                try:
                    self._h = lgpio.gpiochip_open(candidate)
                    break
                except Exception:
                    self._h = None
            if self._h is None:
                raise RuntimeError("Could not open any gpiochip (tried 4, 0)")
        else:
            self._h = lgpio.gpiochip_open(chip)

        lgpio.gpio_claim_output(self._h, self.sck_pin, 0)
        lgpio.gpio_claim_input(self._h, self.dout_pin)

        self.offset = 0.0        # tare (raw units)
        self.scale = 1.0         # raw units per gram
        time.sleep(0.4)          # let the chip settle / power up

    # --- low level ---------------------------------------------------------
    def _is_ready(self):
        # DOUT goes LOW when a conversion is ready
        return lgpio.gpio_read(self._h, self.dout_pin) == 0

    def _wait_ready(self, timeout=2.0):
        start = time.time()
        while not self._is_ready():
            if time.time() - start > timeout:
                raise TimeoutError(
                    "HX711 not responding. Check wiring: VCC, GND, DT, SCK "
                    "and that the load cell is attached."
                )
            time.sleep(0.001)

    def _read_raw_once(self):
        self._wait_ready()
        value = 0
        for _ in range(24):
            lgpio.gpio_write(self._h, self.sck_pin, 1)
            value = (value << 1) | lgpio.gpio_read(self._h, self.dout_pin)
            lgpio.gpio_write(self._h, self.sck_pin, 0)

        # gain/channel selection pulses
        for _ in range(self._gain_pulses):
            lgpio.gpio_write(self._h, self.sck_pin, 1)
            lgpio.gpio_write(self._h, self.sck_pin, 0)

        # convert 24-bit two's complement to signed int
        if value & 0x800000:
            value -= 0x1000000
        return value

    # --- averaged reads ----------------------------------------------------
    def read_raw(self, times=10):
        """Median-filtered raw reading (robust against spikes)."""
        vals = []
        for _ in range(max(1, times)):
            vals.append(self._read_raw_once())
        return statistics.median(vals)

    def tare(self, times=20):
        """Set the current load as zero."""
        self.offset = self.read_raw(times)
        return self.offset

    def get_units(self, times=10):
        """Weight in grams using stored offset and scale."""
        raw = self.read_raw(times)
        if self.scale == 0:
            return 0.0
        return (raw - self.offset) / self.scale

    def close(self):
        try:
            lgpio.gpiochip_close(self._h)
        except Exception:
            pass
