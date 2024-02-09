import machine, os
from machine import Pin, SPI
import sdcard
import uos
try:
    # set up the SD card
    sd_spi = SPI(0, sck=Pin(18, Pin.OUT), mosi=Pin(19, Pin.OUT), miso=Pin(16, Pin.OUT))
    sd = sdcard.SDCard(sd_spi, Pin(22))
    uos.mount(sd, "/sd")
    #sd.info()
    print("File: boot.py: SD card mounted at \"/sd\"")
except OSError as e:
    print("Error: {e}")
    print("Mounting SD-Card failed")
    pass
