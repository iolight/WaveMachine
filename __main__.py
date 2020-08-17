'A module for use with one _very_ specific piece of hardware.'
import RPi.GPIO as GPIO
from .controller import Controller


def main():
    'Creates a `Controller` and does stuff.'
    #try:
    controller = Controller()
    controller.calibrate_servo(0, 0, False)
    controller.finish()
    controller.manual_control(0,0)
    GPIO.cleanup()
    #finally:
    #    controller.stop_all()
# test comment 3


if __name__ == "__main__":
    main()
