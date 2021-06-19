from itertools import cycle

from gpiozero import Device, DigitalOutputDevice
from gpiozero.threads import GPIOThread

class StepperMotor(Device):
    """
    GPIO output device to control a stepper motor.

    Each stepper coil (usually 4) is attached to its own DigitalOutputDevice to turn it on and off.
    The StepperMotor then switches the coils in the correct sequence in order to make the motor
    actually rotate in one direction or the other.

    There are two possible sequences. With half_steps=False, exactly one coil will be active at a time
    and a step moves to the next coil.
        (1, 0, 0, 0)
        (0, 1, 0, 0)
        (0, 0, 1, 0)
        (0, 0, 0, 1)
    With half_steps=True, additinal steps will be inserted into the sequence with two consecutive coils
    active concurrently, which moves the motor into an intermediate position.
        (1, 0, 0, 0)
        (1, 1, 0, 0)
        (0, 1, 0, 0)
        (0, 1, 1, 0)
        (0, 0, 1, 0)
        (0, 0, 1, 1)
        (0, 0, 0, 1)
        (1, 0, 0, 1)

    While it is possible to set a value for all coils directly, e.g.
        motor = StepperMotor((18, 23, 24, 25))
        motor.value = (1,0,0,0)
    it is recommended to use the position property instead
        motor.position = 0
    Changing the value directly will not be recognised as a change of its position. Calling forward(),
    backward() or setting a new position will just continue with the value corresponding to the new
    position, independent from any value set directly.

    It does not make much sense to enable opposing coils simultaniously, but the class will not prevent
    you drom doing so.

    The StepperMotor will disable all outputs initially (value = (0,0,0,0)). When setting a value or
    position, calling on(), forward() or backward() will activate coils independent from the actual
    physical position of the stepper, since we cannot read that anyways. The physical motor will snap
    into some position, possibly moving forward or backward slightly.

    The design of this class is based on gpiozero.RGBLED.
    """

    def __init__(self, pins, *, half_steps=False, pin_factory=None):
        self._coils = tuple(
            DigitalOutputDevice(pin, pin_factory=pin_factory) for pin in pins)
        self._coil_values = tuple(
            tuple(1 if i == j else 0 for j in range(len(pins)))
            for i in range(len(pins)))
        if half_steps:
            self._coil_values = tuple(
                tuple(min(1,self._coil_values[i//2][j] + self._coil_values[((i+1)//2)%len(pins)][j]) for j in range(len(pins)))
                for i in range(2*len(pins)))
        self._position = 0
        self._zero = tuple(0 for i in range(len(pins)))
        self._move_thread = None
        super().__init__(pin_factory=pin_factory)
        self.value = self._zero

    def close(self):
        if getattr(self, '_coils', None):
            self._stop_move()
            for coil in self._coils:
                coil.close()
        self._coils = ()
        super().close()

    @property
    def closed(self):
        return len(self._coils) == 0

    @property
    def value(self):
        return tuple(coil.value for coil in self._coils)

    @value.setter
    def value(self, value):
        for v in value:
            if v not in (0, 1):
                raise OutputDeviceBadValue(
                    'each component must be 0 or 1')
        self._stop_move()
        # TODO: update _position if value matches any? Should we throw if it matches none?
        for coil, v in zip(self._coils, value):
            coil.value = v

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, pos):
        self._stop_move()
        self._position = pos
        self.value = self._coil_values[pos % len(self._coil_values)]

    @property
    def is_active(self):
        return self.value != self._zero

    def off(self):
        self._stop_move()
        self.value = self._zero

    def on(self):
        self._stop_move()
        self.value = self._coil_values[self._position]

    def step(self, direction):
        self._stop_move()
        if direction > 0:
            self._position = (self._position + 1) % len(self._coil_values)
        else:
            self._position = (self._position + len(self._coil_values) - 1) % len(self._coil_values)
        self.value = self._coil_values[self._position]

    def forward(self, n, fps=25, background=True):
        self._stop_move()
        self._move_thread = GPIOThread(self._move_device, (1, n, fps))
        self._move_thread.start()
        if not background:
            self._move_thread.join()
            self._move_thread = None

    def backward(self, n, fps=25, background=True):
        self._stop_move()
        self._move_thread = GPIOThread(self._move_device, (0, n, fps))
        self._move_thread.start()
        if not background:
            self._move_thread.join()
            self._move_thread = None

    def _stop_move(self):
        if self._move_thread:
            self._move_thread.stop()
            self._move_thread = None

    def _move_device(self, direction, n, fps=25):
        delay = 1 / fps
        if n is None:
            sequence = cycle((0, ))
        else:
            sequence = range(n)
        # setting c._controller will make c.__del__ call c._controller._stop_blink(), which obviously
        # does not exist, because we called it _stop_move().
        #for c in self._coils:
        #    c._controller = self
        for x in sequence:
            if direction > 0:
                self._position = (self._position + 1) % len(self._coil_values)
            else:
                self._position = (self._position + len(self._coil_values) - 1) % len(self._coil_values)
            for c, v in zip(self._coils, self._coil_values[self._position]):
                c._write(v)
            if self._move_thread.stopping.wait(delay):
                break

def main():
    motor = StepperMotor((18, 23, 24, 25), half_steps=True)
    motor.forward(1000, fps=100, background=False)
    print(motor.position, motor.value)
    motor.close()

if __name__ == "__main__":
    main()
