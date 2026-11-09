"""Creates every hardware object once, at import.

main.py and test.py both import from here, so the pin setup lives in one
place. Pin numbers and bus speeds come from config.py.
"""

from machine import ADC, PWM, Pin, UART

import config as cfg
from disc import Disc

error_led = Pin(cfg.PIN_ERROR_LED, Pin.OUT, value=1)  # starts off
harp_led = Pin(cfg.PIN_HARP_LED, Pin.OUT, value=0)
status_led = Pin(cfg.PIN_STATUS_LED, Pin.OUT, value=1)

sync_uart = UART(1, baudrate=cfg.SYNC_BAUD, rx=Pin(cfg.PIN_SYNC_RX))
disc = Disc(UART(0, baudrate=cfg.SERVO_BAUD, tx=Pin(cfg.PIN_SERVO_TX), rx=Pin(cfg.PIN_SERVO_RX)))

encoder = ADC(Pin(cfg.PIN_ENCODER))
trigger = Pin(cfg.PIN_TRIGGER, Pin.IN, Pin.PULL_UP)
ir_beam = Pin(cfg.PIN_IR_BEAM, Pin.IN, Pin.PULL_UP)
ir_led = PWM(Pin(cfg.PIN_IR_LED), freq=50_000, duty_u16=65535)
ir_angle = ADC(Pin(cfg.PIN_IR_ANGLE))


def set_error_led(on):
    error_led.value(0 if on else 1)  # active-low
