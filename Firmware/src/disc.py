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

DELIVER_STEPS = const(8912)
STANDBY_STEPS = const(153)

IDLE = const(0x00)
MOVING = const(0x01)
# Added to IDLE or MOVING when the last delivery did the full travel with
# no stop(). Cleared when the next delivery starts.
TIMEOUT = const(0x02)
ERROR = const(0x04)

TOLERANCE = const(15)
# A poll that moves the disc by STALL_TICKS or less counts as no motion.
# This absorbs encoder jitter. STALL_POLLS such polls in a row is a stall.
STALL_TICKS = const(2)
STALL_POLLS = const(10)
POLL_MS = const(50)


class Disc(Dynamixel):

    def __init__(self, uart):
        super().__init__(uart, model=DynamixelModel.XL330_M288, id=1)
        self._ismoving = False
        self._iserror = False
        self._istimeout = False
        self._ready = False
        # Bumped whenever a move is started or stopped. A cancelled move
        # task sees a newer value and leaves the shared state alone.
        self._gen = 0
        self._task = None
        self._reported = None
        self.target_pos = 0
        # Status values not yet sent to the host, oldest first.
        self.pending = []
        self.isr = Event()

        # The servo may be absent or unpowered. The device must still start,
        # so a failure here only sets the ERROR state. The next move retries.
        self._try_setup()
        self._report()

    def _try_setup(self):
        try:
            self._setup_servo()
            self._ready = True
        except Exception:
            self._ready = False
            self._iserror = True
        return self._ready

    def _setup_servo(self):
        """Apply the operating mode and the default limits.

        Raises if the servo does not answer.
        """
        self.torque_enabled = False
        self.operating_mode = 5  # current-based position control
        self.speed = VEL_DEFAULT
        self.torque = TRQ_DEFAULT
        self.target_pos = self.present_position

    @property
    def is_moving(self) -> bool:
        return self._ismoving

    @property
    def status(self) -> int:
        if self._iserror:
            return ERROR
        s = MOVING if self._ismoving else IDLE
        if self._istimeout:
            s |= TIMEOUT
        return s

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
        # Profile Velocity is in RAM, so it can be written with torque on.
        self.profile_velocity = max(0, min(vel, VEL_LIM))

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
        """Start one delivery. Returns False if the disc is already moving.

        No servo I/O happens here; the move runs in its own task. If the
        disc completes the full DELIVER_STEPS without stop() being called,
        the TIMEOUT bit is set in the status.
        """
        if self._ismoving:
            return False
        self._istimeout = False
        self._start(self._run(VEL_DEFAULT, -DELIVER_STEPS, True))
        return True

    def standby(self):
        """Move to the standby position. Replaces any move in progress."""
        self._start(self._run(VEL_STANDBY, -STANDBY_STEPS, False))

    def stop(self):
        """Hold the disc where it is and end the current move."""
        if not self._ismoving:
            return
        self._cancel()
        self._gen += 1
        if self._ready:
            try:
                pos = self.present_position
                self.goal_extend_position = pos
                self.target_pos = pos
            except Exception:
                self._iserror = True
                self._ready = False
        self._finish()

    def _cancel(self):
        t = self._task
        self._task = None
        if t is not None and not t.done():
            t.cancel()

    def _start(self, coro):
        self._cancel()
        self._gen += 1
        # Set before the task runs, so a second request is refused at once.
        self._ismoving = True
        self._iserror = False
        self._report()
        self._task = asyncio.create_task(coro)

    async def _run(self, vel, steps, must_stop):
        gen = self._gen
        full_travel = False
        try:
            if not self._ready and not self._try_setup():
                return
            self.speed = vel
            self.target_pos += steps
            self.torque_enabled = True
            self.goal_extend_position = self.target_pos

            stalled = 0
            last_pos = self.present_position
            while True:
                pos = self.present_position
                if pos <= self.target_pos + TOLERANCE:
                    full_travel = must_stop
                    break
                if abs(pos - last_pos) <= STALL_TICKS:
                    stalled += 1
                    if stalled >= STALL_POLLS:
                        break
                else:
                    stalled = 0
                    last_pos = pos
                await asyncio.sleep_ms(POLL_MS)
            pos = self.present_position
            self.goal_extend_position = pos
            self.target_pos = pos
            await asyncio.sleep_ms(150)
        except Exception:
            # A servo comms error. A CancelledError passes through.
            self._iserror = True
            self._ready = False
        finally:
            if gen == self._gen:
                if full_travel:
                    self._istimeout = True
                self._finish()

    def _finish(self):
        self._ismoving = False
        self.torque_enabled = False
        self._report()

    def _report(self):
        """Queue the status for the host if it changed."""
        s = self.status
        if s != self._reported:
            self._reported = s
            self.pending.append(s)
            self.isr.set()
