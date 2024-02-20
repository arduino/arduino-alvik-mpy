import sys
import gc
import struct

from machine import I2C
from uart import uart
import _thread
from time import sleep_ms

from ucPack import ucPack

from conversions import *
from pinout_definitions import *
from robot_definitions import *
from constants import *


class ArduinoAlvik:

    _update_thread_running = False
    _update_thread_id = None

    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(ArduinoAlvik, cls).__new__(cls)
        return cls.instance

    def __init__(self):
        self.packeter = ucPack(200)
        self.left_wheel = _ArduinoAlvikWheel(self.packeter, ord('L'))
        self.right_wheel = _ArduinoAlvikWheel(self.packeter, ord('R'))
        self.led_state = [None]
        self.left_led = _ArduinoAlvikRgbLed(self.packeter, 'left', self.led_state,
                                            rgb_mask=[0b00000100, 0b00001000, 0b00010000])
        self.right_led = _ArduinoAlvikRgbLed(self.packeter, 'right', self.led_state,
                                             rgb_mask=[0b00100000, 0b01000000, 0b10000000])
        self.battery_perc = None
        self.touch_bits = None
        self.behaviour = None
        self.red = None
        self.green = None
        self.blue = None
        self._white_cal = None
        self._black_cal = None
        self.left_line = None
        self.center_line = None
        self.right_line = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.x = None
        self.y = None
        self.theta = None
        self.ax = None
        self.ay = None
        self.az = None
        self.gx = None
        self.gy = None
        self.gz = None
        self.left_tof = None
        self.center_left_tof = None
        self.center_tof = None
        self.center_right_tof = None
        self.right_tof = None
        self.top_tof = None
        self.bottom_tof = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.last_ack = ''
        self.version = [None, None, None]

    @staticmethod
    def is_alvik_on() -> bool:
        """
        Returns true if robot is on
        :return:
        """
        return CHECK_STM32.value() == 1

    @staticmethod
    def _progress_bar(percentage: float) -> None:
        """
        Prints a progressbar
        :param percentage:
        :return:
        """
        sys.stdout.write('\r')
        if percentage > 98:
            marks_str = ' \U0001F50B'
        else:
            marks_str = ' \U0001FAAB'
        sys.stdout.write(marks_str + f" {percentage}% \t")

    def _idle(self, delay_=1, check_on_thread=False) -> None:
        """
        Alvik's idle mode behaviour
        :return:
        """

        NANO_CHK.value(1)
        sleep_ms(500)
        led_val = 0

        try:
            while not self.is_alvik_on():
                if check_on_thread and not self.__class__._update_thread_running:
                    break
                ESP32_SDA = Pin(A4, Pin.OUT)
                ESP32_SCL = Pin(A5, Pin.OUT)
                ESP32_SCL.value(1)
                ESP32_SDA.value(1)
                sleep_ms(100)
                ESP32_SCL.value(0)
                ESP32_SDA.value(0)

                cmd = bytearray(1)
                cmd[0] = 0x06
                i2c = I2C(0, scl=ESP32_SCL, sda=ESP32_SDA)
                i2c.writeto(0x36, cmd)
                soc_raw = struct.unpack('h', i2c.readfrom(0x36, 2))[0]
                soc_perc = soc_raw*0.00390625
                self._progress_bar(round(soc_perc))
                sleep_ms(delay_)
                if soc_perc > 98:
                    LEDG.value(0)
                    LEDR.value(1)
                else:
                    LEDR.value(led_val)
                    LEDG.value(1)
                    led_val = (led_val + 1) % 2
        except KeyboardInterrupt:
            self.stop()
            sys.exit()
        except Exception as e:
            pass
            #print(f'Unable to read SOC: {e}')
        finally:
            LEDR.value(1)
            LEDG.value(1)
            NANO_CHK.value(0)

    def _snake_robot(self, duration: int = 1000):
        """
        Snake robot animation
        :param percentage:
        :return:
        """
        i = 0

        robot = '\U0001F916'
        snake = '\U0001F40D'

        cycles = int(duration / 200)

        for i in range(0, cycles):
            sys.stdout.write('\r')
            pre = ' ' * i
            between = ' ' * (i % 2 + 1)
            post = ' ' * 5
            frame = pre + snake + between + robot + post
            sys.stdout.write(frame)
            sleep_ms(200)

        sys.stdout.write('\r')
        sys.stdout.write('')

    def begin(self) -> int:
        """
        Begins all Alvik operations
        :return:
        """
        if not self.is_alvik_on():
            print("\nTurn on your Arduino Alvik!\n")
            sleep_ms(1000)
            self._idle()
        self._begin_update_thread()
        sleep_ms(100)
        self._reset_hw()
        self._flush_uart()
        self._snake_robot(1000)
        self._wait_for_ack()
        self._snake_robot(2000)
        self.set_illuminator(True)
        self.set_behaviour(1)
        self._set_color_reference()
        return 0

    def _wait_for_ack(self) -> None:
        """
        Waits until receives 0x00 ack from robot
        :return:
        """
        while self.last_ack != 0x00:
            sleep_ms(20)

    @staticmethod
    def _flush_uart():
        """
        Empties the UART buffer
        :return:
        """
        while uart.any():
            uart.read(1)

    def _begin_update_thread(self):
        """
        Runs robot background operations (e.g. threaded update)
        :return:
        """

        if not self.__class__._update_thread_running:
            self.__class__._update_thread_running = True
            self.__class__._update_thread_id = _thread.start_new_thread(self._update, (1,))

    @classmethod
    def _stop_update_thread(cls):
        """
        Stops the background operations
        :return:
        """
        cls._update_thread_running = False

    def _wait_for_target(self):
        while not self.is_target_reached():
            pass

    def is_target_reached(self) -> bool:
        """
        Returns True if robot has sent an M or R acknowledgment.
        It also responds with an ack received message
        :return:
        """
        if self.last_ack != ord('M') and self.last_ack != ord('R'):
            sleep_ms(50)
            return False
        else:
            self.packeter.packetC1B(ord('X'), ord('K'))
            uart.write(self.packeter.msg[0:self.packeter.msg_size])
            sleep_ms(200)
            return True

    def set_behaviour(self, behaviour: int):
        """
        Sets the behaviour of Alvik
        :param behaviour: behaviour code
        :return:
        """
        self.packeter.packetC1B(ord('B'), behaviour & 0xFF)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def rotate(self, angle: float, unit: str = 'deg', blocking: bool = True):
        """
        Rotates the robot by given angle
        :param angle:
        :param blocking:
        :param unit: the angle unit
        :return:
        """
        angle = convert_angle(angle, unit, 'deg')
        sleep_ms(200)
        self.packeter.packetC1F(ord('R'), angle)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])
        if blocking:
            self._wait_for_target()

    def move(self, distance: float, unit: str = 'cm', blocking: bool = True):
        """
        Moves the robot by given distance
        :param distance:
        :param blocking:
        :param unit: the distance unit
        :return:
        """
        distance = convert_distance(distance, unit, 'mm')
        sleep_ms(200)
        self.packeter.packetC1F(ord('G'), distance)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])
        if blocking:
            self._wait_for_target()

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

        # delete instance
        del self.__class__.instance
        gc.collect()

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

    def get_wheels_speed(self, unit: str = 'rpm') -> (float, float):
        """
        Returns the speed of the wheels
        :param unit: the speed unit of measurement (default: 'rpm')
        :return: left_wheel_speed, right_wheel_speed
        """
        return self.left_wheel.get_speed(unit), self.right_wheel.get_speed(unit)

    def set_wheels_speed(self, left_speed: float, right_speed: float, unit: str = 'rpm'):
        """
        Sets left/right motor speed
        :param left_speed:
        :param right_speed:
        :param unit: the speed unit of measurement (default: 'rpm')
        :return:
        """

        if unit == '%':
            left_speed = (left_speed/100)*MOTOR_MAX_RPM
            right_speed = (right_speed/100)*MOTOR_MAX_RPM
        else:
            left_speed = convert_rotational_speed(left_speed, unit, 'rpm')
            right_speed = convert_rotational_speed(right_speed, unit, 'rpm')

        self.packeter.packetC2F(ord('J'), left_speed, right_speed)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def set_wheels_position(self, left_angle: float, right_angle: float, unit: str = 'deg'):
        """
        Sets left/right motor angle
        :param left_angle:
        :param right_angle:
        :param unit: the speed unit of measurement (default: 'rpm')
        :return:
        """
        self.packeter.packetC2F(ord('A'), convert_angle(left_angle, unit, 'deg'),
                                convert_angle(right_angle, unit, 'deg'))
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def get_wheels_position(self, unit: str = 'deg') -> (float, float):
        """
        Returns the angle of the wheels
        :param unit: the angle unit of measurement (default: 'deg')
        :return: left_wheel_angle, right_wheel_angle
        """
        return (convert_angle(self.left_wheel.get_position(), 'deg', unit),
                convert_angle(self.right_wheel.get_position(), 'deg', unit))

    def get_orientation(self) -> (float, float, float):
        """
        Returns the orientation of the IMU
        :return: roll, pitch, yaw
        """

        return self.roll, self.pitch, self.yaw

    def get_accelerations(self) -> (float, float, float):
        """
        Returns the 3-axial acceleration of the IMU
        :return: ax, ay, az
        """
        return self.ax, self.ay, self.az

    def get_gyros(self) -> (float, float, float):
        """
        Returns the 3-axial angular acceleration of the IMU
        :return: gx, gy, gz
        """
        return self.gx, self.gy, self.gz

    def get_imu(self) -> (float, float, float, float, float, float):
        """
        Returns all the IMUs readouts
        :return: ax, ay, az, gx, gy, gz
        """
        return self.ax, self.ay, self.az, self.gx, self.gy, self.gz

    def get_line_sensors(self) -> (int, int, int):
        """
        Returns the line sensors readout
        :return: left_line, center_line, right_line
        """

        return self.left_line, self.center_line, self.right_line

    def drive(self, linear_velocity: float, angular_velocity: float, linear_unit: str = 'cm/s',
              angular_unit: str = 'deg/s'):
        """
        Drives the robot by linear and angular velocity
        :param linear_velocity:
        :param angular_velocity:
        :param linear_unit: output linear velocity unit of meas
        :param angular_unit: output angular velocity unit of meas
        :return:
        """
        linear_velocity = convert_speed(linear_velocity, linear_unit, 'mm/s')
        if angular_unit == '%':
            angular_velocity = (angular_velocity/100)*ROBOT_MAX_DEG_S
        else:
            angular_velocity = convert_rotational_speed(angular_velocity, angular_unit, 'deg/s')
        self.packeter.packetC2F(ord('V'), linear_velocity, angular_velocity)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def brake(self):
        """
        Brakes the robot
        :return:
        """
        self.drive(0, 0)

    def get_drive_speed(self, linear_unit: str = 'cm/s', angular_unit: str = 'deg/s') -> (float, float):
        """
        Returns linear and angular velocity of the robot
        :param linear_unit: output linear velocity unit of meas
        :param angular_unit: output angular velocity unit of meas
        :return: linear_velocity, angular_velocity
        """
        if angular_unit == '%':
            angular_velocity = (self.angular_velocity/ROBOT_MAX_DEG_S)*100
        else:
            angular_velocity = convert_rotational_speed(self.angular_velocity, 'deg/s', angular_unit)

        return convert_speed(self.linear_velocity, 'mm/s', linear_unit), angular_velocity

    def reset_pose(self, x: float, y: float, theta: float, distance_unit: str = 'cm', angle_unit: str = 'deg'):
        """
        Resets the robot pose
        :param x: x coordinate of the robot
        :param y: y coordinate of the robot
        :param theta: angle of the robot
        :param distance_unit: angle of the robot
        :param angle_unit: angle of the robot
        :return:
        """
        x = convert_distance(x, distance_unit, 'mm')
        y = convert_distance(y, distance_unit, 'mm')
        theta = convert_angle(theta, angle_unit, 'deg')
        self.packeter.packetC3F(ord('Z'), x, y, theta)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])
        sleep_ms(1000)

    def get_pose(self, distance_unit: str = 'cm', angle_unit: str = 'deg') -> (float, float, float):
        """
        Returns the current pose of the robot
        :param distance_unit: unit of x and y outputs
        :param angle_unit: unit of theta output
        :return: x, y, theta
        """
        return (convert_distance(self.x, 'mm', distance_unit),
                convert_distance(self.y, 'mm', distance_unit),
                convert_angle(self.theta, 'deg', angle_unit))

    def set_servo_positions(self, a_position: int, b_position: int):
        """
        Sets A/B servomotor angle
        :param a_position: position of A servomotor (0-180)
        :param b_position: position of B servomotor (0-180)
        :return:
        """
        self.packeter.packetC2B(ord('S'), a_position & 0xFF, b_position & 0xFF)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def get_ack(self):
        """
        Returns last acknowledgement
        :return:
        """
        return self.last_ack

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
        :param delay_: while loop delay (ms)
        :return:
        """
        while True:
            if not self.is_alvik_on():
                print("Alvik not connected")
                self._idle(delay_, check_on_thread=True)
                self._reset_hw()
                self._flush_uart()
                sleep_ms(1000)
                self._wait_for_ack()
                sleep_ms(2000)
                self.set_illuminator(True)
                self.set_behaviour(1)
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
            if b == self.packeter.end_index and self.packeter.checkPayload():
                return True
        return False

    def _parse_message(self) -> int:
        """
        Parse a received message
        :return: -1 if parse error 0 if ok
        """
        code = self.packeter.payloadTop()
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
            _, self.ax, self.ay, self.az, self.gx, self.gy, self.gz = self.packeter.unpacketC6F()
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
             self.center_right_tof, self.right_tof, self.top_tof, self.bottom_tof) = self.packeter.unpacketC7I()
        elif code == ord('q'):
            # imu position
            _, self.roll, self.pitch, self.yaw = self.packeter.unpacketC3F()
        elif code == ord('w'):
            # wheels position
            _, self.left_wheel._position, self.right_wheel._position = self.packeter.unpacketC2F()
        elif code == ord('v'):
            # robot velocity
            _, self.linear_velocity, self.angular_velocity = self.packeter.unpacketC2F()
        elif code == ord('x'):
            # robot ack
            _, self.last_ack = self.packeter.unpacketC1B()
        elif code == ord('z'):
            # robot ack
            _, self.x, self.y, self.theta = self.packeter.unpacketC3F()
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

    @staticmethod
    def _limit(value: float, lower: float, upper: float) -> float:
        """
        Utility function to limit a value between a lower and upper limit
        :param value:
        :param lower:
        :param upper:
        :return:
        """
        assert lower < upper
        if value > upper:
            value = upper
        if value < lower:
            value = lower
        return value

    def _set_color_reference(self):
        try:
            from color_calibration import BLACK_CAL as _B
        except ImportError:
            _B = BLACK_CAL
        try:
            from color_calibration import WHITE_CAL as _W
        except ImportError:
            _W = WHITE_CAL

        self._black_cal = _B
        self._white_cal = _W

    def color_calibration(self, background: str = 'white') -> None:
        """
        Calibrates the color sensor
        :param background: str white or black
        :return:
        """
        if background not in ['black', 'white']:
            return

        red_avg = green_avg = blue_avg = 0

        for _ in range(0, 100):
            red, green, blue = self.get_color_raw()
            red_avg += red
            green_avg += green
            blue_avg += blue
            sleep_ms(10)

        red_avg = int(red_avg/100)
        green_avg = int(green_avg/100)
        blue_avg = int(blue_avg/100)

        if background == 'white':
            self._white_cal = [red_avg, green_avg, blue_avg]
        elif background == 'black':
            self._black_cal = [red_avg, green_avg, blue_avg]

        file_path = './color_calibration.py'

        try:
            with open(file_path, 'r') as file:
                content = file.read().split('\n')
                lines = [line + '\n' for line in content if line]
        except OSError:
            open(file_path, 'a').close()
            lines = []

        found_param_line = False

        for i, line in enumerate(lines):
            if line.startswith(background.upper()):
                lines[i] = f'{background.upper()}_CAL = [{red_avg}, {green_avg}, {blue_avg}]\n'
                found_param_line = True
                break

        if not found_param_line:
            lines.extend([f'{background.upper()}_CAL = [{red_avg}, {green_avg}, {blue_avg}]\n'])

        with open(file_path, 'w') as file:
            for line in lines:
                file.write(line)

    def get_color_raw(self) -> (int, int, int):
        """
        Returns the color sensor's raw readout
        :return: red, green, blue
        """

        return self.red, self.green, self.blue

    def _normalize_color(self, r: float, g: float, b: float) -> (float, float, float):
        """
        Color normalization
        :param r:
        :param g:
        :param b:
        :return:
        """
        r = self._limit(r, self._black_cal[0], self._white_cal[0])
        g = self._limit(g, self._black_cal[1], self._white_cal[1])
        b = self._limit(b, self._black_cal[2], self._white_cal[2])

        r = (r - self._black_cal[0])/(self._white_cal[0] - self._black_cal[0])
        g = (g - self._black_cal[1])/(self._white_cal[1] - self._black_cal[1])
        b = (b - self._black_cal[2])/(self._white_cal[2] - self._black_cal[2])

        return r, g, b

    @staticmethod
    def rgb2hsv(r: float, g: float, b: float) -> (float, float, float):
        """
        Converts normalized rgb to hsv
        :param r:
        :param g:
        :param b:
        :return:
        """
        min_ = min(r, g, b)
        max_ = max(r, g, b)

        v = max_
        delta = max_ - min_

        if delta < 0.00001:
            h = 0
            s = 0
            return h, s, v

        if max_ > 0:
            s = delta / max_
        else:
            s = 0
            h = None
            return h, s, v

        if r >= max_:
            h = (g - b) / delta  # color is between yellow and magenta
        elif g >= max_:
            h = 2.0 + (b - r) / delta
        else:
            h = 4.0 + (r - g) / delta

        h *= 60.0
        if h < 0:
            h += 360.0

        return h, s, v

    def get_color(self, color_format: str = 'rgb') -> (float, float, float):
        """
        Returns the normalized color readout of the color sensor
        :param color_format: rgb or hsv only
        :return:
        """
        assert color_format in ['rgb', 'hsv']

        if color_format == 'rgb':
            return self._normalize_color(*self.get_color_raw())
        elif color_format == 'hsv':
            return self.rgb2hsv(*self._normalize_color(*self.get_color_raw()))

    @staticmethod
    def get_color_label(h, s, v) -> str:
        """
        Returns the color label corresponding to the given normalized HSV color input
        :param h:
        :param s:
        :param v:
        :return:
        """

        if s < 0.1:
            if v < 0.05:
                label = 'BLACK'
            elif v < 0.15:
                label = 'GREY'
            elif v < 0.8:
                label = 'LIGHT GREY'
            else:
                label = 'WHITE'
        else:
            if v > 0.1:
                if 20 <= h < 90:
                    label = 'YELLOW'
                elif 90 <= h < 140:
                    label = 'LIGHT GREEN'
                elif 140 <= h < 170:
                    label = 'GREEN'
                elif 170 <= h < 210:
                    label = 'LIGHT BLUE'
                elif 210 <= h < 260:
                    label = 'BLUE'
                elif 260 <= h < 280:
                    label = 'VIOLET'
                elif 260 <= h < 280:
                    label = 'VIOLET'
                else:                   # h<20 or h>=280 is more problematic
                    if v < 0.5 and s < 0.45:
                        label = 'BROWN'
                    else:
                        if v > 0.77:
                            label = 'ORANGE'
                        else:
                            label = 'RED'
            else:
                label = 'BLACK'
        return label

    def get_distance(self, unit: str = 'cm') -> (float, float, float, float, float):
        """
        Returns the distance readout of the TOF sensor
        :param unit: distance output unit
        :return: left_tof, center_left_tof, center_tof, center_right_tof, right_tof
        """
        return (convert_distance(self.left_tof, 'mm', unit),
                convert_distance(self.center_left_tof, 'mm', unit),
                convert_distance(self.center_tof, 'mm', unit),
                convert_distance(self.center_right_tof, 'mm', unit),
                convert_distance(self.right_tof, 'mm', unit))

    def get_distance_top(self, unit: str = 'cm') -> float:
        """
        Returns the obstacle top distance readout
        :param unit:
        :return:
        """
        return convert_distance(self.top_tof, 'mm', unit)

    def get_distance_bottom(self, unit: str = 'cm') -> float:
        """
        Returns the obstacle bottom distance readout
        :param unit:
        :return:
        """
        return convert_distance(self.bottom_tof, 'mm', unit)

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
        self._position = None

    def reset(self, initial_position: float = 0.0, unit: str = 'deg'):
        """
        Resets the wheel reference position
        :param initial_position:
        :param unit: reference position unit (defaults to deg)
        :return:
        """
        initial_position = convert_angle(initial_position, unit, 'deg')
        self._packeter.packetC2B1F(ord('W'), self._label & 0xFF, ord('Z'), initial_position)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def set_pid_gains(self, kp: float = MOTOR_KP_DEFAULT, ki: float = MOTOR_KI_DEFAULT, kd: float = MOTOR_KD_DEFAULT):
        """
        Set PID gains for Alvik wheels
        :param kp: proportional gain
        :param ki: integration gain
        :param kd: derivative gain
        :return:
        """

        self._packeter.packetC1B3F(ord('P'), self._label & 0xFF, kp, ki, kd)
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

        if unit == '%':
            velocity = (velocity/100)*MOTOR_MAX_RPM
        else:
            velocity = convert_rotational_speed(velocity, unit, 'rpm')

        self._packeter.packetC2B1F(ord('W'), self._label & 0xFF, ord('V'), velocity)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def get_speed(self, unit: str = 'rpm') -> float:
        """
        Returns the current RPM speed of the wheel
        :param unit: the unit of the output speed
        :return:
        """
        if unit == '%':
            speed = (self._speed/MOTOR_MAX_RPM)*100
        else:
            speed = convert_rotational_speed(self._speed, 'rpm', unit)
        return speed

    def get_position(self, unit: str = 'deg') -> float:
        """
        Returns the wheel position (angle with respect to the reference)
        :param unit: the unit of the output position
        :return:
        """
        return convert_angle(self._position, 'deg', unit)

    def set_position(self, position: float, unit: str = 'deg'):
        """
        Sets left/right motor speed
        :param position: the speed of the motor
        :param unit: the unit of measurement
        :return:
        """
        self._packeter.packetC2B1F(ord('W'), self._label & 0xFF, ord('P'),
                                   convert_angle(position, unit, 'deg'))
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
