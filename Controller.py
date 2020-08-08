import time
from scipy.interpolate import interp1d
from math import pi
import Adafruit_PCA9685
import RPi.GPIO as GPIO

min_on_time = 50
limit_switch_pin = 14
encoder_pin = 15
update_period = 1/30 # frequency
sign = lambda a: (a>0) - (a<0)


class Controller:
    def __init__(self):
        self.neutral = [1250] * 100
        self.last_theoretical_on_time = [0] * 100
        self.last_real_on_time = [0] * 100
        self.theoretical_position = [0] * 100
        self.real_position = [0] * 100
        self.last_time = [0] * 100
        self.calibration: interp1d = [None]*100
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(limit_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # limit switch pin
        GPIO.setup(encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # encoder pin
        self.mc = Adafruit_PCA9685.PCA9685(address=0x40)

    def set_pin_speed(self, pinx: int, piny: int, speed_mms: float) -> float:
        pin_num = Controller._get_pin_num(pinx, piny)
        on_time = self.calibration[pin_num](speed_mms)
        return self._set_pin_pwm(pin_num, on_time)

    def zero(self, pinx: int, piny: int) -> float:
        pin_num = Controller._get_pin_num(pinx, piny)
        self._set_pin_pwm(pin_num, -min_on_time)
        while Controller._poll_limit_switch(pinx, piny):
            self._set_pin_pwm(pin_num, -min_on_time)
        return self._set_pin_pwm(pin_num, 0)

    def manual_control(self):
        while True:
            self.set_pin_speed(1, 0, 0)
            speed = int(input("Input Speed in mm/s: "))
            try:
                while True:
                    self.set_pin_speed(1, 0, speed)
            except KeyboardInterrupt:
                pass

    def calibrate_servo(self, pinx: int, piny: int) -> interp1d: # gets data to convert on_time to mm/sec
        pin_num = self._get_pin_num(pinx, piny)
        ydata = range(-100,101,10)
        xdata = [0]*len(ydata)
        on_time_order = sorted(ydata, key=abs)[::-1] # puts the on times in descending order with alternating positive and negative

        self.zero(pinx, piny)
        self._set_pin_pwm(pin_num, min_on_time)
        time.sleep(2)
        self._find_servo_neutral(pin_num)

        rotation_count = 4 # distance to average speed over
        # number can be increased to bump up accuracy at the cost of a longer calibration time
        for i, on_time in enumerate(on_time_order):
            if on_time == 0:
                continue
            rotation_time = self._time_encoder_ticks(pin_num, on_time, rotation_count * 4) # good average even with "large" update_period
            xdata[i] = sign(on_time)*7*pi*rotation_count/rotation_time
        xdata = sorted(xdata) # this should put all the speeds in the correct order for ydata as defined above
        self.calibration[pin_num] = interp1d(xdata, ydata, assume_sorted=True)
        return


    def _find_servo_neutral(self, pin_num: int) -> int:
        while True:
            t0 = self._time_encoder_ticks(pin_num, min_on_time, 4)
            t1 = self._time_encoder_ticks(pin_num, -min_on_time, 4)
            time.sleep(0.1)
            if abs(t0-t1) < update_period*1.2:
                break
            elif t1 < t0:
                self.neutral[pin_num] += 1
            elif t0 < t1:
                self.neutral[pin_num] -= 1
        print("Neutral = {}".format(self.neutral[pin_num]))


    def _set_pin_pwm(self, pin_num: int, on_time_us: int) -> float:
        while time.time() - self.last_time[pin_num] < update_period:
            pass
        sync_time = time.time()

        board_num = pin_num//15  # every 15 pins is 1 board
        channel_num = pin_num % 15  # each board is 15 pins
        self.theoretical_position[pin_num] += self.last_theoretical_on_time[pin_num] * (
            sync_time - self.last_time[pin_num])
        self.real_position[pin_num] += self.last_real_on_time[pin_num] * (
            sync_time - self.last_time[pin_num])
        if abs(on_time_us) < min_on_time and on_time_us != 0:
            if (self.real_position[pin_num] < self.theoretical_position[pin_num]) ^ (on_time_us < 0):
                duration = min_on_time * sign(on_time_us)
                self.last_real_on_time[pin_num] = duration
            else:
                duration = 0
                self.last_real_on_time[pin_num] = 0
        else:
            duration = on_time_us
            self.last_real_on_time[pin_num] = on_time_us
        self.last_theoretical_on_time[pin_num] = on_time_us
        self.last_time[pin_num] = sync_time
        self.mc.set_pwm(channel_num, 0, self.neutral[pin_num] + duration)
        return time.time()

    def  _time_encoder_ticks(self, pin_num: int, on_time: int, tick_goal: int) -> float:
        # starts timing at the first tick it sees. Timing is only accurate to update_period
        encoder_ticks = 0
        current_encoder_reading = GPIO.input(encoder_pin)
        self._set_pin_pwm(pin_num, on_time)
        while GPIO.input(encoder_pin) == current_encoder_reading:
            self._set_pin_pwm(pin_num, on_time) # frequncy update rate (30 Hz)
        current_encoder_reading = not current_encoder_reading
        t0 = time.time()
        while encoder_ticks < tick_goal:
            if GPIO.input(encoder_pin) != current_encoder_reading:
                current_encoder_reading = not current_encoder_reading
                encoder_ticks += 1
            self._set_pin_pwm(pin_num, on_time)
        t1 = time.time()
        while time.time() - t1 < update_period*2: # allows the servo to overshoot, so it has room to accelerate for the next timing
            self._set_pin_pwm(pin_num, on_time)
        self._set_pin_pwm(pin_num, 0)
        time.sleep(0.1)
        return t1 - t0

    @staticmethod
    def _set_limit_switch_mux(pinx: int, piny: int):
        pass

    @staticmethod
    def _set_encoder_mux(pinx: int, piny: int):
        pass

    @staticmethod
    def _get_pin_num(pinx: int, piny: int) -> int:
        return piny * 10 + pinx

    @staticmethod
    def _poll_limit_switch(pinx: int, piny: int) -> bool:
        return GPIO.input(limit_switch_pin)
