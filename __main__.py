'A module for use with one _very_ specific piece of hardware.'
#from .controller import Controller
from .wave_functions import WaveGenerator


def main():
    'Creates a `Controller` and does stuff.'
    wave = WaveGenerator()
    try:
        wave.zero()
        #wave.calibrate()
        wave.sine_wave(2, 0, 10, 1, 40, 0)
    finally:
        wave.finish()
        print("all done")


if __name__ == "__main__":
    main()
