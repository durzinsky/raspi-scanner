from itertools import cycle

from gpiozero import Device, DigitalOutputDevice
from gpiozero.threads import GPIOThread

class StepperMotor(Device):
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
        # TODO: update _position if value matches any?
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
        # setting c._controller will make c.__del__ call c._controller._stop_blink()
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
