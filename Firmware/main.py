import asyncio
import sys

import micropython
from machine import ADC, PWM, Pin, UART
from micropython import const

import usb.device
from usb.device.cdc import CDCInterface

from version import FW_VERSION, HW_VERSION, HARP_VERSION, WHO_AM_I
from disc import Disc
from microharp import (
    EVENT,
    PT_U8,
    PT_U16,
    READ_ONLY,
    READ_WRITE,
    WRITE_ONLY,
    CdcTransport,
    HarpDevice,
)
from microharp.registers import R_HARP_VERSION_H, R_HARP_VERSION_L


SAMPLE_PERIOD_MS = const(10)
ADDR_STATUS = 0x20
ADDR_ENCODER = 0x21
ADDR_IR_BEAM = 0x22
ADDR_DELIVERY = 0x23

    
errorLED = Pin(21, Pin.OUT, value=1)
harpLED = Pin(20, Pin.OUT, value=0)
myled = Pin(19, Pin.OUT, value=1)


myCLK = UART(1, baudrate=100_000, rx=Pin(25))
disc = Disc(UART(0, baudrate=1_000_000, tx=Pin(12), rx=Pin(13)))
encoder = ADC(Pin(28))
trigger = Pin(27, Pin.IN, Pin.PULL_UP)
irBeam = Pin(0, Pin.IN, Pin.PULL_UP)
irLED = PWM(Pin(9), freq=50_000, duty_u16=65535)
irAng = ADC(Pin(26))
# builtin_driver=True keeps the MicroPython REPL on its own USB serial port
# next to the Harp port. mpremote and tracebacks stay available. Set it to
# False for a release build with a single port.
cdc = CDCInterface(baudrate=1_000_000, timeout=0, txbuf=2048, rxbuf=512)
usb.device.get().init(cdc, builtin_driver=True)

device = HarpDevice(
    transport=CdcTransport(cdc),
    sync_uart=myCLK,
    led_pin=myled,
    who_am_i=WHO_AM_I,
    fw_version=FW_VERSION,
    hw_version=HW_VERSION,
    device_name=b"Feeder"
)
# microharp has no parameter for the core version. Write the registers.
device.bank.get(R_HARP_VERSION_H).storage[0] = HARP_VERSION[0]
device.bank.get(R_HARP_VERSION_L).storage[0] = HARP_VERSION[1]


async def worker():
    if disc._ismoving:
        disc.stop()
        # while irBeam.value() == 0 or disc._ismoving:
        #     await asyncio.sleep_ms(100)
        disc.standby()

device.add_u8(ADDR_STATUS, access=EVENT, name="status")
device.add_u16(ADDR_ENCODER, access=EVENT, name="Encoder")
device.add_u8(ADDR_DELIVERY, access=WRITE_ONLY, name="Delivery")

device.bind_pin_event(
    irBeam,
    address=ADDR_IR_BEAM,
    payload_type=PT_U8,
    trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
    name="IR_Beam",
    task=worker,
    active_level=0,
)

def _move_disc(_arg):
    if not disc._ismoving:
        disc.deliver()


def callback(pin):
    micropython.schedule(_move_disc, 0)


trigger.irq(handler=callback, trigger=Pin.IRQ_FALLING, hard=True)

@device.on_write(address=ADDR_DELIVERY, payload_type=PT_U8, name="Delivery")
async def _deliver(reg, payload):
    # DisableMotor is a state, not a one-off command. The disc does not
    # move until the host enables the motor again.
    if (not disc._ismoving) and (payload[0] > 0):
        disc.deliver()

@device.task
async def _state_task():
    """Report the gate state on every change, and the motor state when it
    changes. Both are driven by the gate, not by a timer."""
    payload = bytearray(1)
    while True:
        await disc.isr.wait()
        payload[0] = disc.status
        await device.emit(ADDR_STATUS, payload, PT_U8)
        disc.isr.clear()

# Periodic emission task.
@device.task
async def _adc_stream():
    payload = bytearray(2)
    while True:
        raw = encoder.read_u16()
        payload[0] = raw & 0xFF
        payload[1] = raw >> 8

        # adc_reg.storage[0] = payload[0]
        await device.emit(ADDR_ENCODER, payload, PT_U16)
        await asyncio.sleep_ms(SAMPLE_PERIOD_MS)

asyncio.run(device.run())



