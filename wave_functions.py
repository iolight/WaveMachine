
import time
from .controller import Controller
from math import sin

DIST_MULT = 1
TIME_MULT = 1



class Wave_Generator:
    'Uses the controller class to run servos in sync to make forms'

    def __init__(self):
        pass

    def sine_wave(self, x_velocity: int, y_velocity: int, \
            x_wavelength: int, y_wavelength: int, \
            x_amplitude: int,  y_amplitude: int) # velocity in mm/s, wavelength in mm, amplitude in mm
        'Moves the system in waves in the x and y direction'
        x_wave = lambda x,t: \
            x_amplitude * sin(x * x_wavelength * DIST_MULT + t * x_velocity * TIME_MULT )
        y_wave = lambda y,t: \
            y_amplitude * sin(x * y_wavelength * DIST_MULT + t * y_velocity * TIME_MULT )