import time
from scipy.interpolate import interp1d
import Adafruit_PCA9685
import RPi.GPIO as GPIO

neutral = 1250
min_on_time = 50


class Controller:
    def __init__(self):
        self.last_theoretical_on_time = [0] * 100
        self.last_real_on_time = [0] * 100
        self.theoretical_position = [0] * 100
        self.real_position = [0] * 100
        self.last_time = [0] * 100
        self.calibration: interp1d = None
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(14, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # limit switch pin
        GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # encoder pin
        self.mc = Adafruit_PCA9685.PCA9685(address=0x40)

    def set_pin_speed(self, pinx: int, piny: int, speed_mms: float) -> float:
        pin_num = Controller._get_pin_num(pinx, piny)
        if pin_num in self.calibration:
            servo_calib = self.calibration[pin_num]
            if speed_mms in servo_calib:
                on_time = servo_calib[speed_mms]
            else:
                pass
        return self._set_pin_pwm(pin_num, on_time)

    def zero(self, pinx: int, piny: int) -> float:
        self.set_pin_speed(pinx, piny, -10)
        while Controller._poll_limit_switch(pinx, piny):
            time.sleep(0.001)
        return self.set_pin_speed(pinx, piny, 0)

    def manual_control(self):
        while True:
            self.set_pin_speed(1, 0, 0)
            speed = int(input("Input Speed in mm/s"))
            try:
                while True:
                    self.set_pin_speed(1, 0, speed)
                    time.sleep(0.025)
            except KeyboardInterrupt:
                pass

    def _set_pin_pwm(self, pinnum: int, on_time_us: int) -> float:
        pin_num = piny * 10 + pinx  # order from row and col
        board_num = pin_num//15  # every 15 pins is 1 board
        channel_num = pin_num % 15  # each board is 15 pins
        self.theoretical_position[pin_num] += self.last_theoretical_on_time[pin_num] * (
            time.time() - self.last_time[pin_num])
        self.real_position[pin_num] += self.last_real_on_time[pin_num] * (
            time.time() - self.last_time[pin_num])
        if abs(on_time_us) < min_on_time:
            if self.real_position[pin_num] < self.theoretical_position[pin_num]:
                duration = min_on_time
                self.last_real_on_time[pin_num] = min_on_time
            else:
                duration = 0
                self.last_real_on_time[pin_num] = 0
        else:
            duration = on_time_us
            self.last_real_on_time[pin_num] = on_time_us
        self.last_theoretical_on_time[pin_num] = on_time_us
        self.mc.set_pwm(channel_num, 0, neutral + duration)
        return time.time()

    @staticmethod
    def _get_pin_num(pinx: int, piny: int) -> int:
        return piny * 10 + pinx

    @staticmethod
    def _poll_limit_switch(pinx: int, piny: int) -> bool:
        return GPIO.input(14)
