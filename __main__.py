'A module for use with one _very_ specific piece of hardware.'
import RPi.GPIO as GPIO
from .controller import Controller
from .wave_functions import WaveGenerator


def main():
    'Creates a `Controller` and does stuff.'
    #try:

    wave = WaveGenerator()
    wave.sine_wave(10, 0, 100, 1, 40, 0)
    GPIO.cleanup()
    print("done")
    #finally:
    #    controller.stop_all()
# test comment 3


if __name__ == "__main__":
    main()
