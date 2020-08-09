'A module for use with one _very_ specific piece of hardware.'
from .controller import Controller


def main():
    'Creates a `Controller` and does stuff.'
    try:
        controller = Controller()
        controller.calibrate_servo(1, 0)
        controller.manual_control()
    finally:
        controller.set_pin_speed(1, 0, 0)

# test comment 3


if __name__ == "__main__":
    main()
