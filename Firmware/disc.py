from dynamixel import Dynamixel, DynamixelModel
from dynamixel.table import ControlTableItem
from micropython import const
from asyncio import Event
import asyncio


TRQ_LIM = const(0xC8)
VEL_LIM = const(0xFF)

TRQ_DEFAULT = const(150)
VEL_DEFAULT = const(15)
VEL_STANDBY = const(25)

IDLE = const(0x00)
MOVING = const(0x03)
ERROR = const(0xFF)

TOLERANCE = const(15)


class Disc(Dynamixel):

    def __init__(self, uart):
        super().__init__(uart, model=DynamixelModel.XL330_M288, id=1)
        self._ismoving = False
        self._iscalibrating = False
        self._iserror = False
        self.isr = Event()
        self._task = None

        # The servo may be absent or unpowered. The device must still start,
        # so a failure here only sets the ERROR state. Calibrate retries.
        try:
            self._setup_servo()
        except Exception:
            self._iserror = True

    def _setup_servo(self):
        """Apply the operating mode and the default limits.

        Raises if the servo does not answer.
        """
        self.torque_enabled = False
        self.operating_mode = 5  # current control mode
        self.speed = VEL_DEFAULT
        self.torque = TRQ_DEFAULT
        self.target_pos = self.present_position

    @property
    def status(self) -> int:
        if self._iserror:
            return ERROR
        elif self._ismoving:
            return MOVING
        return IDLE

    @property
    def torque(self):
        return self.current_limit >> 2

    @torque.setter
    def torque(self, val):
        val = max(0, min(val, TRQ_LIM))
        temp = self.torque_enabled
        self.torque_enabled = False
        self.current_limit = val << 2
        # Do not switch the motor on if the host disabled it.
        self.torque_enabled = temp

    @property
    def speed(self):
        return self.profile_velocity

    @speed.setter
    def speed(self, vel):
        vel = max(0, min(vel, VEL_LIM))
        temp = self.torque_enabled
        self.torque_enabled = False
        self.profile_velocity = vel
        # Do not switch the motor on if the host disabled it.
        self.torque_enabled = temp

    @property
    def telemetry(self):
        """Voltage in 0.1 V, temperature in C, current in mA, and the servo
        hardware error status. Raises if the servo does not answer.
        """
        return (
            self._read_value(ControlTableItem.PRESENT_INPUT_VOLTAGE),
            self._read_value(ControlTableItem.PRESENT_TEMPERATURE),
            self.present_current,
            self._read_value(ControlTableItem.HARDWARE_ERROR_STATUS),
        )

    def deliver(self):
        self.speed = VEL_DEFAULT
        self._move(-8912)

    def standby(self):
        self.speed = VEL_STANDBY
        self._move(-153)

    def stop(self):
        if self._task and not self._task.done():
            self._task.cancel()
            self._task = None
            try:
                self.goal_extend_position = self.present_position
                self.target_pos = self.present_position
            except Exception:
                self._iserror = True
            self._ismoving = False
            self.isr.set()

    def _move(self, pos):
        self.target_pos += pos
        self._start(self._run())

    def _start(self, coro):
        if self._task and not self._task.done():
            self._task.cancel()
        self._task = asyncio.create_task(coro)

    async def _run(self):
        count = 0
        last_pos = self.present_position
        try:
            self._enable()

            while self.present_position > self.target_pos + TOLERANCE:
                if count >= 10:
                    break
                elif self.present_position == last_pos:
                    count +=1
                else:
                    count = 0
                    last_pos = self.present_position
                await asyncio.sleep_ms(50)
            self.goal_extend_position = self.present_position
            self.target_pos = self.present_position
            await asyncio.sleep_ms(150)
        except Exception:
            # A servo comms error. A CancelledError passes through.
            self._iserror = True
        finally:
            self._disable()

    def _disable(self):
        self._ismoving = False
        self.torque_enabled = False
        self.isr.set()

    def _enable(self):
        self.torque_enabled = True
        self.goal_extend_position = self.target_pos
        self._ismoving = True
        self.isr.set()
