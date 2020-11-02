'''
This is a good module full of good code written by good people who gooded.
'''

# todo:
#       track locational offset
#       multithread the i2c call

from time import time, sleep
import pickle
from typing import List, Tuple
from math import pi
from dataclasses import dataclass
import Adafruit_PCA9685
import RPi.GPIO as GPIO
from scipy.interpolate import interp1d
from .utils.sign import sign

MIN_ON_TIME = 50
K_P = 1
MAX_SPEED_MMS = 25
SERVO_CIRC = 7*pi # circumfrence in mm

LIMIT_SWITCH_PIN = 15
ENCODER_PIN = 14
MUX_CHANNEL_PINS = [17, 27, 22, 23] # [23, 22, 27, 17]
MUX_BOARD_PINS = [5, 6, 16, 26] # [26, 16, 6, 5]

UPDATE_PERIOD = 1/30  # frequency



@dataclass
class CData:
    'Pure dataclass for tracking and calibrating the servos.'
    neutral: List[int]
    calibration: List[interp1d]
    encoder_offset: List[float] # units: mm
    last_time: List[float]
    last_theoretical_on_time: List[float]
    last_real_on_time: List[float]
    theoretical_on_time_sum: List[float]
    real_on_time_sum: List[float]
    last_speed: List[float]
    last_encoder_reading: List[bool]
    physical_position: List[float] # units: mm
    encoder_ticks: List[bool]


class Controller:
    'Handles everything related to the motor controllers, setup, sensors, and calibration.'

    def __init__(self):
        with open('wave_machine/data.pkl', 'rb') as data_file:
            temp_neutral, temp_calibration, temp_offset = pickle.load(data_file)
        self.c_data = CData(
            neutral=temp_neutral,
            calibration=temp_calibration,
            encoder_offset=temp_offset,
            last_time=[0] * 100,
            last_theoretical_on_time=[0] * 100,
            last_real_on_time=[0] * 100,
            theoretical_on_time_sum=[0] * 100,
            real_on_time_sum=[0] * 100,
            last_speed=[0] * 100,
            last_encoder_reading=[0] * 100,
            encoder_ticks=[0] * 100,
            physical_position=[0] * 100)
        print(self.c_data.neutral)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN,
                   pull_up_down=GPIO.PUD_UP)    # limit switch
        GPIO.setup(ENCODER_PIN, GPIO.IN,
                   pull_up_down=GPIO.PUD_UP)    # encoder
        for pin in MUX_BOARD_PINS + MUX_CHANNEL_PINS:
            GPIO.setup(pin, GPIO.OUT)
        self.controller = Adafruit_PCA9685.PCA9685(address=0x40)

    def set_pin_position(self, pinx: float, piny: float, position: float):
        '''Uses a proportional controller to align the bead position with what it is declared as.
        Has a max speed of 25 mm/s'''
        pin_num = Controller._get_pin_num(pinx, piny)
        print(str(round(position)).ljust(4, ' '), round(self.c_data.physical_position[pin_num]))
        speed_mms = max(min((position - self.c_data.physical_position[pin_num]) \
            * K_P, MAX_SPEED_MMS), -MAX_SPEED_MMS)
        self.set_pin_speed(pinx, piny, speed_mms)

    def set_pin_speed(self, pinx: int, piny: int, speed_mms: float) -> float:
        'Sets the speed of the servo on pinx, piny to the given speed in mm/s.'
        Controller._set_mux(pinx, piny)
        pin_num = Controller._get_pin_num(pinx, piny)
        on_time = int(self.c_data.calibration[pin_num](speed_mms).round())
        self.c_data.last_speed[pin_num] = speed_mms
        return self._set_pin_pwm(pin_num, on_time)

#       self.c_data.physical_position[pin_num] \
#       += self.c_data.last_speed[pin_num] * (time() - self.c_data.last_time_speed[pin_num])
#       self.c_data.last_time_speed[pin_num] = time()

    def zero(self, pinx: int, piny: int):
        'Runs the servo until the upper limit switch is triggered.'
        print("Zeroing {}, {}".format(pinx, piny))
        Controller._set_mux(pinx, piny)
        pin_num = Controller._get_pin_num(pinx, piny)
        self._set_pin_pwm(pin_num, -MIN_ON_TIME*4)
        while GPIO.input(LIMIT_SWITCH_PIN):
            self._set_pin_pwm(pin_num, -MIN_ON_TIME*4)
        print("zeroed")
        self._set_pin_pwm(pin_num, 0)
        self.c_data.physical_position[pin_num] = 0
        self.c_data.last_encoder_reading[pin_num] = GPIO.input(ENCODER_PIN)
        self.c_data.encoder_ticks[pin_num] = 0

    def manual_control(self, pinx: int, piny: int):
        '''Allows the user to manually input speeds in mm/s on the command line.
        Ctrl-c in terminal to stop and input the next speed.
        SIGINT? More like "thank you, next."
        This is a bad function.'''
        while True:
            self.set_pin_speed(pinx, piny, 0)
            speed = float(input("Input Speed in mm/s: "))
            try:
                while True:
                    self.set_pin_speed(pinx, piny, speed)
            except KeyboardInterrupt: # this is the bad
                pass

    def stop_all(self):
        'Stops all servos.'
        self.controller.set_all_pwm(0, 0)

    def finish(self):
        'A simple halt function to guarantee that every servo is fully stopped'
        self.stop_all()
        GPIO.cleanup()
        with open('wave_machine/data.pkl', 'wb') as data_file:
            pickle.dump([self.c_data.neutral, self.c_data.calibration, \
                self.c_data.encoder_offset], data_file)

    def calibrate_servo(self, pinx: int, piny: int, recalibrate: bool = False):
        '''Generates an interpolation curve for the servo pinx,piny
        in order to translate speeds in mm/s to pwm timings for servos.'''
        pin_num = self._get_pin_num(pinx, piny)
        Controller._set_mux(pinx, piny)

        if recalibrate or self.c_data.calibration[pin_num] is None:

            ydata = range(-150, 151, 10)
            xdata = [0]*len(ydata)
            # puts the on times in descending order with alternating positive and negative
            on_time_order = sorted(ydata, key=abs)[::-1]

            self.zero(pinx, piny)
            self._time_encoder_ticks(pin_num, 200, 20)
            self._find_servo_neutral(pin_num)

            rotation_count = 4  # distance to average speed over
            # number can be increased to bump up accuracy at the cost of a longer calibration time
            for i, on_time in enumerate(on_time_order):
                if on_time == 0:
                    continue
                # good average even with "large" update_period
                rotation_time = self._time_encoder_ticks(
                    pin_num, on_time, rotation_count * 4)
                xdata[i] = sign(on_time)*SERVO_CIRC*rotation_count/rotation_time
            # this should put all the speeds in the correct order for ydata as defined above
            xdata = sorted(xdata)
            self.c_data.calibration[pin_num] = interp1d(
                xdata, ydata, assume_sorted=True)
            self.zero(pinx, piny)
            self._find_encoder_offset(pinx, piny)
            self.zero(pinx, piny)

    def _find_servo_neutral(self, pin_num: int):
        # Runs the servo backwards and forwards while adjusting the neutral
        # of the servo in order to match their speeds
        while True:
            if self._time_encoder_ticks(pin_num, 0, tick_goal=1, timeout=1, override=True):
                drift_direction = input("Did that go (u)p or (d)own?")
                if drift_direction == 'u':
                    self.c_data.neutral[pin_num] += 30
                elif drift_direction == 'd':
                    self.c_data.neutral[pin_num] -= 30
            else:
                break

        while True:
            time_0 = self._time_encoder_ticks(pin_num, MIN_ON_TIME, 4)
            time_1 = self._time_encoder_ticks(pin_num, -MIN_ON_TIME, 4)
            sleep(0.1)
            if abs(time_0-time_1) < UPDATE_PERIOD*1.2:
                break
            elif time_1 < time_0:
                self.c_data.neutral[pin_num] += 1
            elif time_0 < time_1:
                self.c_data.neutral[pin_num] -= 1
        print("Neutral = {}".format(self.c_data.neutral[pin_num]))

    def _find_encoder_offset(self, pinx: int, piny: int):
        # this is the offset before the first triggering of the encoder
        current_encoder_reading = GPIO.input(ENCODER_PIN)
        finding_speed = 1
        time_0 = self.set_pin_speed(pinx, piny, finding_speed)
        while GPIO.input(ENCODER_PIN) == current_encoder_reading:
            self.set_pin_speed(pinx, piny, finding_speed)
        time_1 = time()
        self.set_pin_speed(pinx, piny, 0)
        self.c_data.encoder_offset[Controller._get_pin_num(pinx, piny)] \
            = (time_1 - time_0) * finding_speed

    def _set_pin_pwm(self, pin_num: int, on_time_us: int, override: bool = False) -> float:
        print(on_time_us)
        while time() - self.c_data.last_time[pin_num] < UPDATE_PERIOD:
            pass # waits for the 30 Hz signal to sync up
        sync_time = time()

        if not GPIO.input(LIMIT_SWITCH_PIN) and on_time_us < 0:
            # prevents from running into the top
            print("Hit top")
            on_time_us = 0
            self.c_data.encoder_ticks[pin_num] = 0

        if GPIO.input(ENCODER_PIN) != self.c_data.last_encoder_reading[pin_num]:
            self.c_data.last_encoder_reading[pin_num] \
                = not self.c_data.last_encoder_reading[pin_num]
            self.c_data.encoder_ticks[pin_num] \
                += sign(self.c_data.last_theoretical_on_time[pin_num])
            self.c_data.physical_position[pin_num] \
                = self.c_data.encoder_ticks[pin_num] * SERVO_CIRC/4 \
                + self.c_data.encoder_offset[pin_num]

        _board_num, channel_num = Controller._get_board_and_channel(pin_num)
        self.c_data.theoretical_on_time_sum[pin_num] \
            += self.c_data.last_theoretical_on_time[pin_num] \
            * (sync_time - self.c_data.last_time[pin_num])
        self.c_data.real_on_time_sum[pin_num] += self.c_data.last_real_on_time[pin_num] * (
            sync_time - self.c_data.last_time[pin_num])
        if abs(on_time_us) < MIN_ON_TIME and on_time_us != 0:
            if (self.c_data.real_on_time_sum[pin_num] \
                    < self.c_data.theoretical_on_time_sum[pin_num]) \
                    ^ (on_time_us < 0):
                duration = MIN_ON_TIME * sign(on_time_us)
                self.c_data.last_real_on_time[pin_num] = duration
            else:
                duration = 0
                self.c_data.last_real_on_time[pin_num] = 0
        else:
            duration = on_time_us
            self.c_data.last_real_on_time[pin_num] = on_time_us
        self.c_data.last_theoretical_on_time[pin_num] = on_time_us
        self.c_data.last_time[pin_num] = sync_time
        if on_time_us == 0 and not override:
            self.controller.set_pwm(channel_num, 0, 0)
        else:
            self.controller.set_pwm(
                channel_num, 0, self.c_data.neutral[pin_num] + duration)
        return time()

    def _time_encoder_ticks(self, pin_num: int, on_time: int, tick_goal: int, \
        timeout: int = 10, override: bool = False) -> float:
        # Runs for a certain number of encoder ticks
        # Starts timing at the first tick it sees. Timing is only accurate to update_period
        encoder_ticks = 0
        current_encoder_reading = GPIO.input(ENCODER_PIN)
        self._set_pin_pwm(pin_num, on_time, override)
        stuck_time_start = time()
        while GPIO.input(ENCODER_PIN) == current_encoder_reading:
            self._set_pin_pwm(pin_num, on_time, override)  # frequncy update rate (30 Hz)
            if time() - stuck_time_start > timeout:
                return False
        current_encoder_reading = not current_encoder_reading
        time_0 = time()
        while encoder_ticks < tick_goal:
            if GPIO.input(ENCODER_PIN) != current_encoder_reading:
                current_encoder_reading = not current_encoder_reading
                encoder_ticks += 1
            self._set_pin_pwm(pin_num, on_time, override)
        time_1 = time()
        # allows the servo to overshoot, so it has room to accelerate for the next timing
        while time() - time_1 < UPDATE_PERIOD*2:
            self._set_pin_pwm(pin_num, on_time, override)
        self._set_pin_pwm(pin_num, 0)
        sleep(0.1)
        return time_1 - time_0

    @staticmethod
    def _set_mux(pinx: int, piny: int):
        # A two-deep mux system allowing reading of all 200 sensors
        # Encoder and limit switch mux-ing is controlled by the same pins
        pin_num = Controller._get_pin_num(pinx, piny)
        board_num, channel_num = Controller._get_board_and_channel(pin_num)
        for i, bit in enumerate(bin(board_num)[2:].zfill(4)):
            GPIO.output(MUX_BOARD_PINS[i], int(bit))
        for i, bit in enumerate(bin(channel_num)[2:].zfill(4)):
            GPIO.output(MUX_CHANNEL_PINS[i], int(bit))

    @staticmethod
    def _get_board_and_channel(pin_num: int) -> Tuple[int, int]:
        board_num = pin_num//15  # every 15 pins is 1 board
        channel_num = pin_num % 15  # each board is 15 pins
        return board_num, channel_num

    @staticmethod
    def _get_pin_num(pinx: int, piny: int) -> int:
        return piny * 10 + pinx
