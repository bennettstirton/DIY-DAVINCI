# MCP23017 16-bit I2C GPIO expander — minimal driver for digital inputs.
#
# Configured here for crash/homing limit switches: all 16 pins as inputs with
# internal pull-ups. INTA/INTB are unused — the caller polls read_gpio() every
# main-loop tick instead, so there's no interrupt wiring or ISR to maintain.

_IODIRA = 0x00
_IODIRB = 0x01
_GPPUA  = 0x0C
_GPPUB  = 0x0D
_GPIOA  = 0x12
_GPIOB  = 0x13


class MCP23017:
    def __init__(self, i2c, addr=0x20):
        self.i2c  = i2c
        self.addr = addr
        self._write8(_IODIRA, 0xFF)
        self._write8(_IODIRB, 0xFF)
        self._write8(_GPPUA, 0xFF)
        self._write8(_GPPUB, 0xFF)

    def _write8(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def read_gpio(self):
        """Read both ports in one transaction. Returns a 16-bit int: bit0-7 = PA0-7, bit8-15 = PB0-7."""
        data = self.i2c.readfrom_mem(self.addr, _GPIOA, 2)
        return data[0] | (data[1] << 8)

    def read_porta(self):
        """Read just Port A (PA0-7) — cheaper single-byte read for callers that
        only need a couple of those bits and poll faster than the main loop."""
        return self.i2c.readfrom_mem(self.addr, _GPIOA, 1)[0]
