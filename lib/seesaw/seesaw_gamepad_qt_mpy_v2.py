# MIT License

# MicroPython Port Copyright (c) 2019
# Mihai Dinculescu

# CircuitPython Implementation Copyright (c) 2017
# Dean Miller for Adafruit Industries

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
This is a lightweight port from CircuitPython to MicroPython
of Dean Miller's https://github.com/adafruit/Adafruit_CircuitPython_seesaw/blob/master/adafruit_seesaw/seesaw.py

* Author(s): Mihai Dinculescu

Implementation Notes
--------------------

**Hardware:**
* Adafruit ATSAMD09 Breakout with SeeSaw: https://www.adafruit.com/product/3657

**Software and Dependencies:**
* MicroPython firmware: https://micropython.org

**Tested on:**
* Hardware: Adafruit HUZZAH32 - ESP32 Feather https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/overview
* Firmware: MicroPython v1.12 https://micropython.org/resources/firmware/esp32-idf3-20191220-v1.12.bin

See: https://github.com/mihai-dinculescu/micropython-adafruit-drivers/tree/master/seesaws

** 2024-02-07 changes by @PaulskPt
**Tested on:**
* Hardware: Pimoroni Inky Frame 5.7
* Firmware: MicroPython v1.21.0, inky_frame v1.21.0 on 2023-10-06; Raspberry Pi Pico W with RP2040
* Attached hardware: Adafruit Gamepad QT (ID 5743)
*
* This is a "slim" version of the original Adafruit_seesaw module
* to preserve memory to use with a Raspberry Pi PicoW on a Pimoroni Inky Frame 5.7
* For this reason the functions digital_read_bulk() and pin_mode_bulk() have gotten default values for
* some parameters, so these parameters don't have to be passed (on a stack).

"""
import struct
import time

try:
    from micropython import const
except ImportError:

    def const(x):
        return x

STATUS_BASE = const(0x00)
_GPIO_BASE = const(0x01)
_ADC_BASE = const(0x09)
TOUCH_BASE = const(0x0F)

_STATUS_HW_ID = const(0x01)
_STATUS_SWRST = const(0x7F)

_SAMD09_HW_ID_CODE = const(0x55)
_HW_ID_CODE = const(0x87)

_GPIO_DIRSET_BULK = const(0x02)
_GPIO_DIRCLR_BULK = const(0x03)
_GPIO_BULK = const(0x04)
_GPIO_BULK_SET = const(0x05)
_GPIO_BULK_CLR = const(0x06)

_GPIO_PULLENSET = const(0x0B)
_GPIO_PULLENCLR = const(0x0C)

_ADC_CHANNEL_OFFSET = const(0x07)

class Seesaw:
    """Driver for SeeSaw I2C generic conversion trip.
       :param I2C i2c: I2C bus the SeeSaw is connected to.
       :param int addr: I2C address of the SeeSaw device."""
    
    INPUT = const(0x00)
    OUTPUT = const(0x01)
    INPUT_PULLUP = const(0x02)
    INPUT_PULLDOWN = const(0x03)
    
    def __init__(self, i2c, addr):
        self.i2c = i2c
        self.addr = addr
        self.sw_reset()
        
        from seesaw.attinyx16 import ATtinyx16_Pinmap
        self.pin_mapping = ATtinyx16_Pinmap

    def sw_reset(self):
        """Trigger a software reset of the SeeSaw chip"""
        self._write8(STATUS_BASE, _STATUS_SWRST, 0xFF)
        time.sleep(.500)

        self.chip_id = self._read8(STATUS_BASE, _STATUS_HW_ID)

        if self.chip_id != _HW_ID_CODE:
            raise RuntimeError("SeeSaw hardware ID returned (0x{:x}) is not "
                               "correct! Expected 0x{:x}. Please check your wiring."
                               .format(self.chip_id, _HW_ID_CODE))

    def _write8(self, reg_base, reg, value):
        self._write(reg_base, reg, bytearray([value]))

    def _read8(self, reg_base, reg):
        ret = bytearray(1)
        self._read(reg_base, reg, ret)
        return ret[0]

    def _read(self, reg_base, reg, buf, delay=.005):
        self._write(reg_base, reg)

        time.sleep(delay)

        self.i2c.readfrom_into(self.addr, buf)

    def _write(self, reg_base, reg, buf=None):
        full_buffer = bytearray([reg_base, reg])
        if buf is not None:
            full_buffer += buf

        self.i2c.writeto(self.addr, full_buffer)
        
    def pin_mode(self, pin, mode):
        """Set the mode of a pin by number"""
        if pin >= 32:
            self.pin_mode_bulk_b(1 << (pin - 32), mode)
        else:
            self.pin_mode_bulk(1 << pin, mode)
            
    def pin_mode_bulk_b(self, pins, mode):
        """Set the mode of all the pins on the 'B' port as a bitmask"""
        self._pin_mode_bulk_x(8, 4, pins, mode)
            
    def _pin_mode_bulk_x(self, capacity, offset, pins, mode):
        print(f"_pin_mode_bulk_x(): pins= {pins}, mode= {mode}")
        cmd = bytearray(capacity)
        cmd[offset:] = struct.pack(">I", pins)
        if mode == self.OUTPUT:
            self._write(_GPIO_BASE, _GPIO_DIRSET_BULK, cmd)
        elif mode == self.INPUT:
            self._write(_GPIO_BASE, _GPIO_DIRCLR_BULK, cmd)
            self._write(_GPIO_BASE, _GPIO_PULLENCLR, cmd)

        elif mode == self.INPUT_PULLUP:
            self._write(_GPIO_BASE, _GPIO_DIRCLR_BULK, cmd)
            self._write(_GPIO_BASE, _GPIO_PULLENSET, cmd)
            self._write(_GPIO_BASE, _GPIO_BULK_SET, cmd)

        elif mode == self.INPUT_PULLDOWN:
            self._write(_GPIO_BASE, _GPIO_DIRCLR_BULK, cmd)
            self._write(_GPIO_BASE, _GPIO_PULLENSET, cmd)
            self._write(_GPIO_BASE, _GPIO_BULK_CLR, cmd)

        else:
            raise ValueError("Invalid pin mode")

    def pin_mode_bulk(self, pins=0x10067, mode=None):
        if mode is None:
            mode = self.INPUT_PULLUP
        """Set the mode of all the pins on the 'A' port as a bitmask"""
        self._pin_mode_bulk_x(4, 0, pins, mode)
    
    def digital_read_bulk(self, pins=0x10067, delay=0.008):
        """Get the values of all the pins on the 'A' port as a bitmask"""
        buf = bytearray(4)
        self._read(_GPIO_BASE, _GPIO_BULK, buf, delay=delay)
        try:
            ret = struct.unpack(">I", buf)[0]
        except OverflowError:
            buf[0] = buf[0] & 0x3F
            ret = struct.unpack(">I", buf)[0]
        return ret & pins

    def analog_read(self, pin, delay=0.008):
        """Read the value of an analog pin by number"""
        buf = bytearray(2)
        if pin not in self.pin_mapping.analog_pins:
            raise ValueError("Invalid ADC pin")
        if self.chip_id == _SAMD09_HW_ID_CODE:
            offset = self.pin_mapping.analog_pins.index(pin)
        else:
            offset = pin
        self._read(_ADC_BASE, _ADC_CHANNEL_OFFSET + offset, buf, delay)
        ret = struct.unpack(">H", buf)[0]
        return ret
    
