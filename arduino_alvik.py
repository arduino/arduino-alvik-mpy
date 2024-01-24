import math

from uart import uart
import _thread
from time import sleep_ms

from ucPack import ucPack

from pinout_definitions import *
from robot_definitions import *
from constants import *


class ArduinoAlvik:

    _update_thread_running = False

    def __init__(self):
        self.packeter = ucPack(200)
        self.left_wheel = _ArduinoAlvikWheel(self.packeter, ord('L'))
        self.right_wheel = _ArduinoAlvikWheel(self.packeter, ord('R'))
        self.led_state = [None]
        self.left_led = _ArduinoAlvikRgbLed(self.packeter, 'left', self.led_state,
                                            rgb_mask=[0b00000100, 0b00001000, 0b00010000])
        self.right_led = _ArduinoAlvikRgbLed(self.packeter, 'right', self.led_state,
                                             rgb_mask=[0b00100000, 0b01000000, 0b10000000])
        self._update_thread_running = False
        self._update_thread_id = None
        self.battery_perc = None
        self.touch_bits = None
        self.behaviour = None
        self.red = None
        self.green = None
        self.blue = None
        self.left_line = None
        self.center_line = None
        self.right_line = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.left_tof = None
        self.center_left_tof = None
        self.center_tof = None
        self.center_right_tof = None
        self.right_tof = None
        self.top_tof = None
        self.bottom_tof = None
        self.version = [None, None, None]

    def begin(self) -> int:
        """
        Begins all Alvik operations
        :return:
        """
        if not CHECK_STM32.value():
            print("\nTurn on your Arduino Alvik!\n")
            return -1
        self._begin_update_thread()
        sleep_ms(100)
        self._reset_hw()
        sleep_ms(1000)
        return 0

    def _begin_update_thread(self):
        """
        Runs robot background operations (e.g. threaded update)
        :return:
        """

        if not ArduinoAlvik._update_thread_running:
            ArduinoAlvik._update_thread_running = True
            self._update_thread_id = _thread.start_new_thread(self._update, (1,))

    def _stop_update_thread(self):
        """
        Stops the background operations
        :return:
        """
        ArduinoAlvik._update_thread_running = False

    def stop(self):
        """
        Stops all Alvik operations
        :return:
        """
        # stop engines
        self.set_wheels_speed(0, 0)

        # turn off UI leds
        self._set_leds(0x00)

        # stop the update thrad
        self._stop_update_thread()

    @staticmethod
    def _reset_hw():
        """
        Resets the STM32
        :return:
        """

        RESET_STM32.value(0)
        sleep_ms(100)
        RESET_STM32.value(1)
        sleep_ms(100)

    def get_wheels_speed(self) -> (float, float):
        """
        Returns the speed of the wheels
        :return: left_wheel_speed, right_wheel_speed
        """
        return self.left_wheel.get_speed(), self.right_wheel.get_speed()

    def set_wheels_speed(self, left_speed: float, right_speed: float, unit: str = 'rpm'):
        """
        Sets left/right motor speed
        :param left_speed:
        :param right_speed:
        :param unit: the speed unit of measurement (default: 'rpm')
        :return:
        """

        if unit not in angular_velocity_units:
            return
        elif unit == '%':
            left_speed = perc_to_rpm(left_speed)
            right_speed = perc_to_rpm(right_speed)

        self.packeter.packetC2F(ord('J'), left_speed, right_speed)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def set_pid(self, side: str, kp: float, ki: float, kd: float):
        """
        Sets motor PID parameters. Side can be 'L' or 'R'
        :param side:
        :param kp:
        :param ki:
        :param kd:
        :return:
        """

        self.packeter.packetC1B3F(ord('P'), ord(side), kp, ki, kd)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def get_orientation(self) -> (float, float, float):
        """
        Returns the orientation of the IMU
        :return: roll, pitch, yaw
        """

        return self.roll, self.pitch, self.yaw

    def get_line_sensors(self) -> (int, int, int):
        """
        Returns the line sensors readout
        :return: left_line, center_line, right_line
        """

        return self.left_line, self.center_line, self.right_line

    def drive(self, linear_velocity: float, angular_velocity: float):
        """
        Drives the robot by linear and angular velocity
        :param linear_velocity:
        :param angular_velocity:
        :return:
        """
        self.packeter.packetC2F(ord('V'), linear_velocity, angular_velocity)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def set_servo_positions(self, a_position: int, b_position: int):
        """
        Sets A/B servomotor angle
        :param a_position: position of A servomotor
        :param b_position: position of B servomotor
        :return:
        """
        self.packeter.packetC2B(ord('S'), a_position & 0xFF, b_position & 0xFF)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    # def send_ack(self):
    #     self.packeter.packetC1B(ord('X'), ACK_)
    #     uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def _set_leds(self, led_state: int):
        """
        Sets the LEDs state
        :param led_state: one byte 0->builtin 1->illuminator 2->left_red 3->left_green 4->left_blue
        5->right_red 6->right_green 7->right_blue
        :return:
        """
        self.led_state[0] = led_state & 0xFF
        self.packeter.packetC1B(ord('L'), self.led_state[0])
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def set_builtin_led(self, value: bool):
        """
        Turns on/off the builtin led
        :param value:
        :return:
        """
        if self.led_state[0] is None:
            self._set_leds(0x00)
        self.led_state[0] = self.led_state[0] | 0b00000001 if value else self.led_state[0] & 0b11111110
        self._set_leds(self.led_state[0])

    def set_illuminator(self, value: bool):
        """
        Turns on/off the illuminator led
        :param value:
        :return:
        """
        if self.led_state[0] is None:
            self._set_leds(0x00)
        self.led_state[0] = self.led_state[0] | 0b00000010 if value else self.led_state[0] & 0b11111101
        self._set_leds(self.led_state[0])

    def _update(self, delay_=1):
        """
        Updates the robot status reading/parsing messages from UART.
        This method is blocking and meant as a thread callback
        Use the method stop to terminate _update and exit the thread
        :param delay_: while loop delay
        :return:
        """
        while True:
            if not ArduinoAlvik._update_thread_running:
                break
            if self._read_message():
                self._parse_message()
            sleep_ms(delay_)

    def _read_message(self) -> bool:
        """
        Read a message from the uC
        :return: True if a message terminator was reached
        """
        while uart.any():
            b = uart.read(1)[0]
            self.packeter.buffer.push(b)
            if b == self.packeter.end_index:
                self.packeter.checkPayload()
                return True
        return False

    def _parse_message(self) -> int:
        """
        Parse a received message
        :return: -1 if parse error 0 if ok
        """
        code = self.packeter.payload[0]
        if code == ord('j'):
            # joint speed
            _, self.left_wheel._speed, self.right_wheel._speed = self.packeter.unpacketC2F()
        elif code == ord('l'):
            # line sensor
            _, self.left_line, self.center_line, self.right_line = self.packeter.unpacketC3I()
        elif code == ord('c'):
            # color sensor
            _, self.red, self.green, self.blue = self.packeter.unpacketC3I()
        elif code == ord('i'):
            # imu
            _, ax, ay, az, gx, gy, gz = self.packeter.unpacketC6F()
        elif code == ord('p'):
            # battery percentage
            _, self.battery_perc = self.packeter.unpacketC1F()
        elif code == ord('d'):
            # distance sensor
            _, self.left_tof, self.center_tof, self.right_tof = self.packeter.unpacketC3I()
        elif code == ord('t'):
            # touch input
            _, self.touch_bits = self.packeter.unpacketC1B()
        elif code == ord('b'):
            # behaviour
            _, self.behaviour = self.packeter.unpacketC1B()
        elif code == ord('f'):
            # tof matrix
            (_, self.left_tof, self.center_left_tof, self.center_tof,
             self.center_right_tof, self.right_tof, self.bottom_tof, self.top_tof) = self.packeter.unpacketC7I()
        elif code == ord('q'):
            # imu position
            _, self.roll, self.pitch, self.yaw = self.packeter.unpacketC3F()
        elif code == 0x7E:
            # firmware version
            _, *self.version = self.packeter.unpacketC3B()
        else:
            return -1

        return 0

    def _get_touch(self) -> int:
        """
        Returns the touch sensor's state
        :return: touch_bits
        """
        return self.touch_bits

    def get_touch_any(self) -> bool:
        """
        Returns true if any button is pressed
        :return:
        """
        return bool(self.touch_bits & 0b00000001)

    def get_touch_ok(self) -> bool:
        """
        Returns true if ok button is pressed
        :return:
        """
        return bool(self.touch_bits & 0b00000010)

    def get_touch_cancel(self) -> bool:
        """
        Returns true if cancel button is pressed
        :return:
        """
        return bool(self.touch_bits & 0b00000100)

    def get_touch_center(self) -> bool:
        """
        Returns true if center button is pressed
        :return:
        """
        return bool(self.touch_bits & 0b00001000)

    def get_touch_up(self) -> bool:
        """
        Returns true if up button is pressed
        :return:
        """
        return bool(self.touch_bits & 0b00010000)

    def get_touch_left(self) -> bool:
        """
        Returns true if left button is pressed
        :return:
        """
        return bool(self.touch_bits & 0b00100000)

    def get_touch_down(self) -> bool:
        """
        Returns true if down button is pressed
        :return:
        """
        return bool(self.touch_bits & 0b01000000)

    def get_touch_right(self) -> bool:
        """
        Returns true if right button is pressed
        :return:
        """
        return bool(self.touch_bits & 0b10000000)

    def get_color(self) -> (int, int, int):
        """
        Returns the RGB color (raw) readout
        :return: red, green, blue
        """

        return self.red, self.green, self.blue
        # return (int((self.red/COLOR_FULL_SCALE)*255),
        #         int((self.green/COLOR_FULL_SCALE)*255),
        #         int((self.blue/COLOR_FULL_SCALE)*255))

    def get_distance(self) -> (int, int, int, int, int, int):
        """
        Returns the distance readout of the TOF sensor
        :return: left_tof, center_left_tof, center_tof, center_right_tof, right_tof
        """
        return self.left_tof, self.center_left_tof, self.center_tof, self.center_right_tof, self.right_tof

    def get_version(self) -> str:
        """
        Returns the firmware version of the Alvik
        :return:
        """
        return f'{self.version[0]}.{self.version[1]}.{self.version[2]}'

    def print_status(self):
        """
        Prints the Alvik status
        :return:
        """
        for a in vars(self):
            if str(a).startswith('_'):
                continue
            print(f'{str(a).upper()} = {getattr(self, str(a))}')


class _ArduinoAlvikWheel:

    def __init__(self, packeter: ucPack, label: int, wheel_diameter_mm: float = WHEEL_DIAMETER_MM):
        self._packeter = packeter
        self._label = label
        self._wheel_diameter_mm = wheel_diameter_mm
        self._speed = None

    def reset(self, initial_position: float = 0.0):
        """
        Resets the wheel reference position
        :param initial_position:
        :return:
        """
        pass

    def set_pid_gains(self, kp: float = MOTOR_KP_DEFAULT, ki: float = MOTOR_KI_DEFAULT, kd: float = MOTOR_KD_DEFAULT):
        """
        Set PID gains for Alvik wheels
        :param kp: proportional gain
        :param ki: integration gain
        :param kd: derivative gain
        :return:
        """

        self._packeter.packetC1B3F(ord('P'), self._label, kp, ki, kd)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def stop(self):
        """
        Stop Alvik wheel
        :return:
        """
        self.set_speed(0)

    def set_speed(self, velocity: float, unit: str = 'rpm'):
        """
        Sets the motor speed
        :param velocity: the speed of the motor
        :param unit: the unit of measurement
        :return:
        """

        if unit not in angular_velocity_units:
            return
        elif unit == '%':
            velocity = perc_to_rpm(velocity)

        self._packeter.packetC2B1F(ord('W'), self._label & 0xFF, ord('V'), velocity)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def get_speed(self) -> float:
        """
        Returns the current RPM speed of the wheel
        :return:
        """
        return self._speed

    def set_position(self, position: float, unit: str = 'deg'):
        """
        Sets left/right motor speed
        :param position: the speed of the motor
        :param unit: the unit of measurement
        :return:
        """

        if unit not in angle_units:
            return
        elif unit == 'rad':
            position = rad_to_deg(position)

        self._packeter.packetC2B1F(ord('W'), self._label & 0xFF, ord('P'), position)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])


class _ArduinoAlvikRgbLed:
    def __init__(self, packeter: ucPack, label: str, led_state: list[int | None], rgb_mask: list[int]):
        self._packeter = packeter
        self.label = label
        self._rgb_mask = rgb_mask
        self._led_state = led_state

    def set_color(self, red: bool, green: bool, blue: bool):
        """
        Sets the LED's r,g,b state
        :param red:
        :param green:
        :param blue:
        :return:
        """
        led_status = self._led_state[0]
        if led_status is None:
            return
        led_status = led_status | self._rgb_mask[0] if red else led_status & (0b11111111 - self._rgb_mask[0])
        led_status = led_status | self._rgb_mask[1] if green else led_status & (0b11111111 - self._rgb_mask[1])
        led_status = led_status | self._rgb_mask[2] if blue else led_status & (0b11111111 - self._rgb_mask[2])
        self._led_state[0] = led_status
        self._packeter.packetC1B(ord('L'), led_status & 0xFF)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])


# Units and unit conversion methods

angular_velocity_units = ['rpm', '%']
angle_units = ['deg', 'rad']
distance_units = ['mm', 'cm']


def perc_to_rpm(percent: float) -> float:
    """
    Converts percent of max_rpm to rpm
    :param percent:
    :return:
    """
    return (percent / 100.0)*MOTOR_MAX_RPM


def rad_to_deg(rad: float) -> float:
    """
    Converts radians to degrees
    :param rad:
    :return:
    """
    return rad*180/math.pi
