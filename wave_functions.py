'''
The primary method of interacting with every servo.
'''
import time
from math import sin, pi
from .controller import Controller

BEAD_TO_MM = 1
VERTICAL_OFFSET = 100
Y_RANGE = 1
X_RANGE = 2



class WaveGenerator:
    'Uses the controller class to run servos in sync to make forms'

    def __init__(self):
        self.controller = Controller()
        self.controller.stop_all()
        for piny in range(Y_RANGE):
            for pinx in range(X_RANGE):
                self.controller.zero(pinx, piny)
        self.controller.calibrate_servo(0, 0, False)
        self.controller.calibrate_servo(1, 0, False)

    def sine_wave(self, x_velocity: int, y_velocity: int, \
            x_wavelength: int, y_wavelength: int, \
            x_amplitude: int, y_amplitude: int):
        '''Moves the system in waves in the x and y direction.
        Velocity in mm/s, wavelength in mm, amplitude in mm'''
        x_wave_number = 2*pi/x_wavelength
        x_omega = x_wave_number * x_velocity
        y_wave_number = 2*pi/y_wavelength
        y_omega = y_wave_number * y_velocity
        x_wave = lambda x, t: \
            x_amplitude * sin(x_wave_number*x + x_omega*t)
        y_wave = lambda y, t: \
            y_amplitude * sin(y_wave_number*y + y_omega*t)

        run_time = time.time()
        while time.time() - run_time < 60:
            for piny in range(Y_RANGE):
                for pinx in range(X_RANGE):
                    time_0 = time.time()
                    self.controller.set_pin_position(pinx, piny, \
                        x_wave(pinx * BEAD_TO_MM, time_0) \
                            + y_wave(piny * BEAD_TO_MM, time_0) + VERTICAL_OFFSET)
        self.controller.stop_all()
        self.controller.finish()
