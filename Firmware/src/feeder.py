"""Feeder behaviour on top of a HarpDevice.

attach(device) adds the application registers and tasks:

    IR beam    beam broken during a move -> stop the disc, go to standby
    trigger    button press -> start a delivery
    Delivery   host writes > 0 -> start a delivery
    Status     event on every disc status change; error LED follows ERROR
    Encoder    ADC value streamed every SAMPLE_PERIOD_MS

Interrupt handlers only set an asyncio.ThreadSafeFlag. A task waits on the
flag and does the real work, so servo I/O never runs inside an interrupt
and asyncio is never touched from one.
"""

import asyncio

from machine import Pin
from microharp import EVENT, PT_U8, PT_U16, READ_ONLY, WRITE_ONLY
from microharp.registers import OP_OP_MODE_ACTIVE, OP_OP_MODE_MASK, R_OPERATION_CONTROL

import config as cfg
from disc import ERROR
from hardware import disc, encoder, ir_beam, set_error_led, trigger


def attach(device):
    op_reg = device.bank.get(R_OPERATION_CONTROL)

    def active():
        """Harp events are only allowed in Active mode."""
        return op_reg is not None and (op_reg.storage[0] & OP_OP_MODE_MASK) == OP_OP_MODE_ACTIVE

    _add_beam_stop(device)
    _add_trigger(device)
    _add_delivery_register(device)
    _add_status_events(device, active)
    _add_encoder_stream(device, active)


def _add_beam_stop(device):
    flag = asyncio.ThreadSafeFlag()  # type: ignore[attr-defined]

    def on_beam(level, _set=flag.set):
        # Hard IRQ: no allocation. Only the beam-broken edge matters.
        if level == 0:
            _set()

    # Also sends the IR_Beam event and serves register reads.
    device.bind_pin_event(
        ir_beam,
        address=cfg.ADDR_IR_BEAM,
        payload_type=PT_U8,
        trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
        name="IR_Beam",
        on_irq=on_beam,
    )

    @device.task
    async def _beam_task():
        while True:
            await flag.wait()
            if disc.is_moving:
                disc.stop()
                disc.standby()


def _add_trigger(device):
    # Bounces merge into one flag, and deliver() refuses while moving.
    flag = asyncio.ThreadSafeFlag()  # type: ignore[attr-defined]

    def on_press(_pin, _set=flag.set):
        _set()

    trigger.irq(handler=on_press, trigger=Pin.IRQ_FALLING, hard=True)

    @device.task
    async def _trigger_task():
        while True:
            await flag.wait()
            disc.deliver()


def _add_delivery_register(device):
    device.add_u8(cfg.ADDR_DELIVER, access=WRITE_ONLY, name="Delivery")

    @device.on_write(address=cfg.ADDR_DELIVER, payload_type=PT_U8, name="Delivery")
    async def _deliver(reg, payload):
        reg.storage[0] = payload[0]
        if payload[0] > 0:
            disc.deliver()


def _add_status_events(device, active):
    reg = device.add_u8(cfg.ADDR_STATUS, access=READ_ONLY | EVENT, name="status")

    @device.task
    async def _status_task():
        payload = bytearray(1)
        while True:
            await disc.isr.wait()
            # Clear before sending, so a change made during an emit is kept.
            disc.isr.clear()
            while disc.pending:
                status = disc.pending.pop(0)
                reg.storage[0] = status
                set_error_led(status == ERROR)
                if active():
                    payload[0] = status
                    await device.emit(cfg.ADDR_STATUS, payload, PT_U8)


def _add_encoder_stream(device, active):
    reg = device.add_u16(cfg.ADDR_ENCODER, access=READ_ONLY | EVENT, name="Encoder")

    @device.task
    async def _encoder_task():
        payload = bytearray(2)
        while True:
            raw = encoder.read_u16()
            payload[0] = raw & 0xFF
            payload[1] = raw >> 8
            reg.storage[0] = payload[0]
            reg.storage[1] = payload[1]
            if active():
                await device.emit(cfg.ADDR_ENCODER, payload, PT_U16)
            await asyncio.sleep_ms(cfg.SAMPLE_PERIOD_MS)
