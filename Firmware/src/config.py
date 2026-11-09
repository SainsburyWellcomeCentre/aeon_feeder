"""Feeder settings: pins, bus speeds, Harp registers and device identity.

Change values here. The other files read them from this module.
"""

from micropython import const

# ---- Pins (Seeed XIAO RP2350) ---------------------------------------------
PIN_ERROR_LED = const(21)  # active-low: 0 = on
PIN_HARP_LED = const(20)
PIN_STATUS_LED = const(19)  # Harp status LED, driven by microharp
PIN_SYNC_RX = const(25)  # Harp clock sync input
PIN_SERVO_TX = const(12)
PIN_SERVO_RX = const(13)
PIN_ENCODER = const(28)
PIN_TRIGGER = const(27)  # manual delivery button, active-low
PIN_IR_BEAM = const(0)  # 0 = beam broken
PIN_IR_LED = const(9)
PIN_IR_ANGLE = const(26)

# ---- Buses ----------------------------------------------------------------
SYNC_BAUD = const(100_000)
SERVO_BAUD = const(1_000_000)

# ---- Harp application registers -------------------------------------------
ADDR_IR_BEAM = const(0x20)  # U8, event on every beam edge
ADDR_DELIVER = const(0x21)  # U8, write > 0 to start a delivery
ADDR_STATUS = const(0x22)  # U8, disc status (see disc.py)
ADDR_ENCODER = const(0x23)  # U16, ADC reading streamed every SAMPLE_PERIOD_MS

SAMPLE_PERIOD_MS = const(10)

# ---- Harp device identity -------------------------------------------------
WHO_AM_I = const(3003)
FW_VERSION = (0, 1)
HW_VERSION = (2, 1)
HARP_VERSION = (1, 13)
DEVICE_NAME = b"Feeder"
