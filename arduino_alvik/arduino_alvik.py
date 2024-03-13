import sys
import gc
import struct
from machine import I2C
import _thread
from time import sleep_ms, ticks_ms, ticks_diff

from ucPack import ucPack

from .uart import uart
from .conversions import *
from .pinout_definitions import *
from .robot_definitions import *
from .constants import *


class ArduinoAlvik:
    _update_thread_running = False
    _update_thread_id = None
    _touch_events_thread_running = False
    _touch_events_thread_id = None

    def __new__(cls):
        if not hasattr(cls, '_instance'):
            cls._instance = super(ArduinoAlvik, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        self._packeter = ucPack(200)
        self.left_wheel = _ArduinoAlvikWheel(self._packeter, ord('L'))
        self.right_wheel = _ArduinoAlvikWheel(self._packeter, ord('R'))
        self._led_state = list((None,))
        self.left_led = self.DL1 = _ArduinoAlvikRgbLed(self._packeter, 'left', self._led_state,
                                                       rgb_mask=[0b00000100, 0b00001000, 0b00010000])
        self.right_led = self.DL2 = _ArduinoAlvikRgbLed(self._packeter, 'right', self._led_state,
                                                        rgb_mask=[0b00100000, 0b01000000, 0b10000000])
        self._battery_perc = None
        self._touch_byte = None
        self._behaviour = None
        self._red = None
        self._green = None
        self._blue = None
        self._white_cal = None
        self._black_cal = None
        self._left_line = None
        self._center_line = None
        self._right_line = None
        self._roll = None
        self._pitch = None
        self._yaw = None
        self._x = None
        self._y = None
        self._theta = None
        self._ax = None
        self._ay = None
        self._az = None
        self._gx = None
        self._gy = None
        self._gz = None
        self._left_tof = None
        self._center_left_tof = None
        self._center_tof = None
        self._center_right_tof = None
        self._right_tof = None
        self._top_tof = None
        self._bottom_tof = None
        self._linear_velocity = None
        self._angular_velocity = None
        self._last_ack = None
        self._waiting_ack = None
        self._version = [None, None, None]
        self._touch_events = _ArduinoAlvikTouchEvents()

    @staticmethod
    def is_on() -> bool:
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
        sys.stdout.write(bytes('\r'.encode('utf-8')))
        if percentage > 97:
            marks_str = ' \U0001F50B'
        else:
            marks_str = ' \U0001FAAB'
        word = marks_str + f" {percentage}% \t"
        sys.stdout.write(bytes((word.encode('utf-8'))))

    def _idle(self, delay_=1, check_on_thread=False) -> None:
        """
        Alvik's idle mode behaviour
        :return:
        """

        NANO_CHK.value(1)
        sleep_ms(500)
        led_val = 0

        try:
            while not self.is_on():
                if check_on_thread and not self.__class__._update_thread_running:
                    break
                _ESP32_SDA = Pin(A4, Pin.OUT)
                _ESP32_SCL = Pin(A5, Pin.OUT)
                _ESP32_SCL.value(1)
                _ESP32_SDA.value(1)
                sleep_ms(100)
                _ESP32_SCL.value(0)
                _ESP32_SDA.value(0)

                cmd = bytearray(1)
                cmd[0] = 0x06
                i2c = I2C(0, scl=ESP32_SCL, sda=ESP32_SDA)
                i2c.writeto(0x36, cmd)
                soc_raw = struct.unpack('h', i2c.readfrom(0x36, 2))[0]
                soc_perc = soc_raw * 0.00390625
                self._progress_bar(round(soc_perc))
                sleep_ms(delay_)
                if soc_perc > 97:
                    LEDG.value(0)
                    LEDR.value(1)
                else:
                    LEDR.value(led_val)
                    LEDG.value(1)
                    led_val = (led_val + 1) % 2
            print("********** Alvik is on **********")
        except KeyboardInterrupt:
            self.stop()
            sys.exit()
        except Exception as e:
            pass
            # print(f'Unable to read SOC: {e}')
        finally:
            LEDR.value(1)
            LEDG.value(1)
            NANO_CHK.value(0)

    @staticmethod
    def _snake_robot(duration: int = 1000):
        """
        Snake robot animation
        :param duration:
        :return:
        """

        robot = '\U0001F916'
        snake = '\U0001F40D'

        cycles = int(duration / 200)

        frame = ''
        for i in range(0, cycles):
            sys.stdout.write(bytes('\r'.encode('utf-8')))
            pre = ' ' * i
            between = ' ' * (i % 2 + 1)
            post = ' ' * 5
            frame = pre + snake + between + robot + post
            sys.stdout.write(bytes(frame.encode('utf-8')))
            sleep_ms(200)

        sys.stdout.write(bytes('\r'.encode('utf-8')))
        clear_frame = ' ' * len(frame)
        sys.stdout.write(bytes(clear_frame.encode('utf-8')))

    def begin(self) -> int:
        """
        Begins all Alvik operations
        :return:
        """
        if not self.is_on():
            print("\n********** Please turn on your Arduino Alvik! **********\n")
            sleep_ms(1000)
            self._idle(1000)
        self._begin_update_thread()
        sleep_ms(100)
        if self._touch_events.has_callbacks():
            print('Starting touch events')
            self._start_touch_events_thread()
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
        self._waiting_ack = 0x00
        while self._last_ack != 0x00:
            sleep_ms(20)
        self._waiting_ack = None

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

    def _wait_for_target(self, idle_time):
        start = ticks_ms()
        while True:
            if ticks_diff(ticks_ms(), start) >= idle_time * 1000 and self.is_target_reached():
                break
            else:
                # print(self._last_ack)
                sleep_ms(100)

    def is_target_reached(self) -> bool:
        """
        Returns True if robot has sent an M or R acknowledgment.
        It also responds with an ack received message
        :return:
        """
        if self._waiting_ack is None:
            return True
        if self._last_ack == self._waiting_ack:
            self._packeter.packetC1B(ord('X'), ord('K'))
            uart.write(self._packeter.msg[0:self._packeter.msg_size])
            sleep_ms(100)
            self._last_ack = 0x00
            self._waiting_ack = None
            return True
        return False

    def set_behaviour(self, behaviour: int):
        """
        Sets the behaviour of Alvik
        :param behaviour: behaviour code
        :return:
        """
        self._packeter.packetC1B(ord('B'), behaviour & 0xFF)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

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
        self._packeter.packetC1F(ord('R'), angle)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])
        self._waiting_ack = ord('R')
        if blocking:
            self._wait_for_target(idle_time=(angle / MOTOR_CONTROL_DEG_S))

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
        self._packeter.packetC1F(ord('G'), distance)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])
        self._waiting_ack = ord('M')
        if blocking:
            self._wait_for_target(idle_time=(distance / MOTOR_CONTROL_MM_S))

    def stop(self):
        """
        Stops all Alvik operations
        :return:
        """
        # stop engines
        self.set_wheels_speed(0, 0)

        # turn off UI leds
        self._set_leds(0x00)

        # stop the update thread
        self._stop_update_thread()

        # stop touch events thread
        self._stop_touch_events_thread()

        # delete _instance
        del self.__class__._instance
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

    def get_wheels_speed(self, unit: str = 'rpm') -> (float | None, float | None):
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
            left_speed = (left_speed / 100) * MOTOR_MAX_RPM
            right_speed = (right_speed / 100) * MOTOR_MAX_RPM
        else:
            left_speed = convert_rotational_speed(left_speed, unit, 'rpm')
            right_speed = convert_rotational_speed(right_speed, unit, 'rpm')

        self._packeter.packetC2F(ord('J'), left_speed, right_speed)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def set_wheels_position(self, left_angle: float, right_angle: float, unit: str = 'deg'):
        """
        Sets left/right motor angle
        :param left_angle:
        :param right_angle:
        :param unit: the speed unit of measurement (default: 'rpm')
        :return:
        """
        self._packeter.packetC2F(ord('A'), convert_angle(left_angle, unit, 'deg'),
                                 convert_angle(right_angle, unit, 'deg'))
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def get_wheels_position(self, unit: str = 'deg') -> (float | None, float | None):
        """
        Returns the angle of the wheels
        :param unit: the angle unit of measurement (default: 'deg')
        :return: left_wheel_angle, right_wheel_angle
        """
        return (convert_angle(self.left_wheel.get_position(), 'deg', unit),
                convert_angle(self.right_wheel.get_position(), 'deg', unit))

    def get_orientation(self) -> (float | None, float | None, float | None):
        """
        Returns the orientation of the IMU
        :return: roll, pitch, yaw
        """

        return self._roll, self._pitch, self._yaw

    def get_accelerations(self) -> (float | None, float | None, float | None):
        """
        Returns the 3-axial acceleration of the IMU
        :return: ax, ay, az
        """
        return self._ax, self._ay, self._az

    def get_gyros(self) -> (float | None, float | None, float | None):
        """
        Returns the 3-axial angular acceleration of the IMU
        :return: gx, gy, gz
        """
        return self._gx, self._gy, self._gz

    def get_imu(self) -> (float | None, float | None, float | None, float | None, float | None, float | None):
        """
        Returns all the IMUs readouts
        :return: ax, ay, az, gx, gy, gz
        """
        return self._ax, self._ay, self._az, self._gx, self._gy, self._gz

    def get_line_sensors(self) -> (int | None, int | None, int | None):
        """
        Returns the line sensors readout
        :return: left_line, center_line, right_line
        """

        return self._left_line, self._center_line, self._right_line

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
            angular_velocity = (angular_velocity / 100) * ROBOT_MAX_DEG_S
        else:
            angular_velocity = convert_rotational_speed(angular_velocity, angular_unit, 'deg/s')
        self._packeter.packetC2F(ord('V'), linear_velocity, angular_velocity)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def brake(self):
        """
        Brakes the robot
        :return:
        """
        self.drive(0, 0)

    def get_drive_speed(self, linear_unit: str = 'cm/s', angular_unit: str = 'deg/s') -> (float | None, float | None):
        """
        Returns linear and angular velocity of the robot
        :param linear_unit: output linear velocity unit of meas
        :param angular_unit: output angular velocity unit of meas
        :return: linear_velocity, angular_velocity
        """
        if angular_unit == '%':
            angular_velocity = (self._angular_velocity / ROBOT_MAX_DEG_S) * 100 \
                if self._angular_velocity is not None else None
        else:
            angular_velocity = convert_rotational_speed(self._angular_velocity, 'deg/s', angular_unit)

        return convert_speed(self._linear_velocity, 'mm/s', linear_unit), angular_velocity

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
        self._packeter.packetC3F(ord('Z'), x, y, theta)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])
        sleep_ms(1000)

    def get_pose(self, distance_unit: str = 'cm', angle_unit: str = 'deg') \
            -> (float | None, float | None, float | None):
        """
        Returns the current pose of the robot
        :param distance_unit: unit of x and y outputs
        :param angle_unit: unit of theta output
        :return: x, y, theta
        """
        return (convert_distance(self._x, 'mm', distance_unit),
                convert_distance(self._y, 'mm', distance_unit),
                convert_angle(self._theta, 'deg', angle_unit))

    def set_servo_positions(self, a_position: int, b_position: int):
        """
        Sets A/B servomotor angle
        :param a_position: position of A servomotor (0-180)
        :param b_position: position of B servomotor (0-180)
        :return:
        """
        self._packeter.packetC2B(ord('S'), a_position & 0xFF, b_position & 0xFF)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def get_ack(self) -> str:
        """
        Returns last acknowledgement
        :return:
        """
        return self._last_ack

    # def send_ack(self):
    #     self._packeter.packetC1B(ord('X'), ACK_)
    #     uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def _set_leds(self, led_state: int):
        """
        Sets the LEDs state
        :param led_state: one byte 0->builtin 1->illuminator 2->left_red 3->left_green 4->left_blue
        5->right_red 6->right_green 7->right_blue
        :return:
        """
        self._led_state[0] = led_state & 0xFF
        self._packeter.packetC1B(ord('L'), self._led_state[0])
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def set_builtin_led(self, value: bool):
        """
        Turns on/off the builtin led
        :param value:
        :return:
        """
        if self._led_state[0] is None:
            self._set_leds(0x00)
        self._led_state[0] = self._led_state[0] | 0b00000001 if value else self._led_state[0] & 0b11111110
        self._set_leds(self._led_state[0])

    def set_illuminator(self, value: bool):
        """
        Turns on/off the illuminator led
        :param value:
        :return:
        """
        if self._led_state[0] is None:
            self._set_leds(0x00)
        self._led_state[0] = self._led_state[0] | 0b00000010 if value else self._led_state[0] & 0b11111101
        self._set_leds(self._led_state[0])

    def _update(self, delay_=1):
        """
        Updates the robot status reading/parsing messages from UART.
        This method is blocking and meant as a thread callback
        Use the method stop to terminate _update and exit the thread
        :param delay_: while loop delay (ms)
        :return:
        """
        while True:
            if not self.is_on():
                print("Alvik is off")
                self._idle(1000, check_on_thread=True)
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
            self._packeter.buffer.push(b)
            if b == self._packeter.end_index and self._packeter.checkPayload():
                return True
        return False

    def _parse_message(self) -> int:
        """
        Parse a received message
        :return: -1 if parse error 0 if ok
        """
        code = self._packeter.payloadTop()
        if code == ord('j'):
            # joint speed
            _, self.left_wheel._speed, self.right_wheel._speed = self._packeter.unpacketC2F()
        elif code == ord('l'):
            # line sensor
            _, self._left_line, self._center_line, self._right_line = self._packeter.unpacketC3I()
        elif code == ord('c'):
            # color sensor
            _, self._red, self._green, self._blue = self._packeter.unpacketC3I()
        elif code == ord('i'):
            # imu
            _, self._ax, self._ay, self._az, self._gx, self._gy, self._gz = self._packeter.unpacketC6F()
        elif code == ord('p'):
            # battery percentage
            _, self._battery_perc = self._packeter.unpacketC1F()
        elif code == ord('d'):
            # distance sensor
            _, self._left_tof, self._center_tof, self._right_tof = self._packeter.unpacketC3I()
        elif code == ord('t'):
            # touch input
            _, self._touch_byte = self._packeter.unpacketC1B()
        elif code == ord('b'):
            # behaviour
            _, self._behaviour = self._packeter.unpacketC1B()
        elif code == ord('f'):
            # tof matrix
            (_, self._left_tof, self._center_left_tof, self._center_tof,
             self._center_right_tof, self._right_tof, self._top_tof, self._bottom_tof) = self._packeter.unpacketC7I()
        elif code == ord('q'):
            # imu position
            _, self._roll, self._pitch, self._yaw = self._packeter.unpacketC3F()
        elif code == ord('w'):
            # wheels position
            _, self.left_wheel._position, self.right_wheel._position = self._packeter.unpacketC2F()
        elif code == ord('v'):
            # robot velocity
            _, self._linear_velocity, self._angular_velocity = self._packeter.unpacketC2F()
        elif code == ord('x'):
            # robot ack
            if self._waiting_ack is not None:
                _, self._last_ack = self._packeter.unpacketC1B()
            else:
                self._packeter.unpacketC1B()
                self._last_ack = 0x00
        elif code == ord('z'):
            # robot ack
            _, self._x, self._y, self._theta = self._packeter.unpacketC3F()
        elif code == 0x7E:
            # firmware version
            _, *self._version = self._packeter.unpacketC3B()
        else:
            return -1

        return 0

    def get_battery_charge(self) -> int | None:
        """
        Returns the battery SOC
        :return:
        """
        if self._battery_perc is None:
            return None
        if self._battery_perc > 100:
            return 100
        return round(self._battery_perc)

    @property
    def _touch_bits(self) -> int:
        """
        Returns the touch sensor's state
        :return: touch_bits
        """
        return (self._touch_byte & 0xFF) if self._touch_byte is not None else 0x00

    def get_touch_any(self) -> bool:
        """
        Returns true if any button is pressed
        :return:
        """
        return bool(self._touch_bits & 0b00000001)

    def get_touch_ok(self) -> bool:
        """
        Returns true if ok button is pressed
        :return:
        """
        return bool(self._touch_bits & 0b00000010)

    def get_touch_cancel(self) -> bool:
        """
        Returns true if cancel button is pressed
        :return:
        """
        return bool(self._touch_bits & 0b00000100)

    def get_touch_center(self) -> bool:
        """
        Returns true if center button is pressed
        :return:
        """
        return bool(self._touch_bits & 0b00001000)

    def get_touch_up(self) -> bool:
        """
        Returns true if up button is pressed
        :return:
        """
        return bool(self._touch_bits & 0b00010000)

    def get_touch_left(self) -> bool:
        """
        Returns true if left button is pressed
        :return:
        """
        return bool(self._touch_bits & 0b00100000)

    def get_touch_down(self) -> bool:
        """
        Returns true if down button is pressed
        :return:
        """
        return bool(self._touch_bits & 0b01000000)

    def get_touch_right(self) -> bool:
        """
        Returns true if right button is pressed
        :return:
        """
        return bool(self._touch_bits & 0b10000000)

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

        red_avg = int(red_avg / 100)
        green_avg = int(green_avg / 100)
        blue_avg = int(blue_avg / 100)

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

    def get_color_raw(self) -> (int | None, int | None, int | None):
        """
        Returns the color sensor's raw readout
        :return: red, green, blue
        """

        return self._red, self._green, self._blue

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

        r = (r - self._black_cal[0]) / (self._white_cal[0] - self._black_cal[0])
        g = (g - self._black_cal[1]) / (self._white_cal[1] - self._black_cal[1])
        b = (b - self._black_cal[2]) / (self._white_cal[2] - self._black_cal[2])

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

    def get_color(self, color_format: str = 'rgb') -> (float | None, float | None, float | None):
        """
        Returns the normalized color readout of the color sensor
        :param color_format: rgb or hsv only
        :return:
        """
        assert color_format in ['rgb', 'hsv']

        if None in list(self.get_color_raw()):
            return None, None, None

        if color_format == 'rgb':
            return self._normalize_color(*self.get_color_raw())
        elif color_format == 'hsv':
            return self.rgb2hsv(*self._normalize_color(*self.get_color_raw()))

    def get_color_label(self) -> str:
        """
        Returns the label of the color as recognized by the sensor
        :return:
        """
        return self.hsv2label(*self.get_color(color_format='hsv'))

    @staticmethod
    def hsv2label(h, s, v) -> str:
        """
        Returns the color label corresponding to the given normalized HSV color input
        :param h:
        :param s:
        :param v:
        :return:
        """

        if None in [h, s, v]:
            return 'UNDEFINED'

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
                elif 210 <= h < 250:
                    label = 'BLUE'
                elif 250 <= h < 280:
                    label = 'VIOLET'
                else:  # h<20 or h>=280 is more problematic
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

    def get_distance(self, unit: str = 'cm') -> (float | None, float | None, float | None, float | None, float | None):
        """
        Returns the distance readout of the TOF sensor
        :param unit: distance output unit
        :return: left_tof, center_left_tof, center_tof, center_right_tof, right_tof
        """

        return (convert_distance(self._left_tof, 'mm', unit),
                convert_distance(self._center_left_tof, 'mm', unit),
                convert_distance(self._center_tof, 'mm', unit),
                convert_distance(self._center_right_tof, 'mm', unit),
                convert_distance(self._right_tof, 'mm', unit))

    def get_distance_top(self, unit: str = 'cm') -> float | None:
        """
        Returns the obstacle top distance readout
        :param unit:
        :return:
        """
        return convert_distance(self._top_tof, 'mm', unit)

    def get_distance_bottom(self, unit: str = 'cm') -> float | None:
        """
        Returns the obstacle bottom distance readout
        :param unit:
        :return:
        """
        return convert_distance(self._bottom_tof, 'mm', unit)

    def get_version(self) -> str:
        """
        Returns the firmware version of the Alvik
        :return:
        """
        return f'{self._version[0]}.{self._version[1]}.{self._version[2]}'

    def print_status(self):
        """
        Prints the Alvik status
        :return:
        """
        for a in vars(self):
            if str(a).startswith('_'):
                continue
            print(f'{str(a).upper()} = {getattr(self, str(a))}')

    def on_touch_ok_pressed(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when touch button OK is pressed
        :param callback:
        :param args:
        :return:
        """
        self._touch_events.register_callback('on_ok_pressed', callback, args)

    def on_touch_cancel_pressed(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when touch button CANCEL is pressed
        :param callback:
        :param args:
        :return:
        """
        self._touch_events.register_callback('on_cancel_pressed', callback, args)

    def on_touch_center_pressed(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when touch button CENTER is pressed
        :param callback:
        :param args:
        :return:
        """
        self._touch_events.register_callback('on_center_pressed', callback, args)

    def on_touch_up_pressed(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when touch button UP is pressed
        :param callback:
        :param args:
        :return:
        """
        self._touch_events.register_callback('on_up_pressed', callback, args)

    def on_touch_left_pressed(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when touch button LEFT is pressed
        :param callback:
        :param args:
        :return:
        """
        self._touch_events.register_callback('on_left_pressed', callback, args)

    def on_touch_down_pressed(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when touch button DOWN is pressed
        :param callback:
        :param args:
        :return:
        """
        self._touch_events.register_callback('on_down_pressed', callback, args)

    def on_touch_right_pressed(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when touch button RIGHT is pressed
        :param callback:
        :param args:
        :return:
        """
        self._touch_events.register_callback('on_right_pressed', callback, args)

    def _start_touch_events_thread(self) -> None:
        """
        Starts the touch events thread
        :return:
        """
        if not self.__class__._touch_events_thread_running:
            self.__class__._touch_events_thread_running = True
            self.__class__._touch_events_thread_id = _thread.start_new_thread(self._update_touch_events, (50,))

    def _update_touch_events(self, delay_: int = 100):
        """
        Updates the touch state so that touch events can be generated
        :param delay_:
        :return:
        """
        while True:
            if self.is_on() and self._touch_byte is not None:
                self._touch_events.update_touch_state(self._touch_byte)
            if not ArduinoAlvik._touch_events_thread_running:
                break
            sleep_ms(delay_)

    @classmethod
    def _stop_touch_events_thread(cls):
        """
        Stops the touch events thread
        :return:
        """
        cls._touch_events_thread_running = False


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
            velocity = (velocity / 100) * MOTOR_MAX_RPM
        else:
            velocity = convert_rotational_speed(velocity, unit, 'rpm')

        self._packeter.packetC2B1F(ord('W'), self._label & 0xFF, ord('V'), velocity)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def get_speed(self, unit: str = 'rpm') -> float | None:
        """
        Returns the current RPM speed of the wheel
        :param unit: the unit of the output speed
        :return:
        """
        if unit == '%':
            speed = (self._speed / MOTOR_MAX_RPM) * 100 if self._speed is not None else None
        else:
            speed = convert_rotational_speed(self._speed, 'rpm', unit)
        return speed

    def get_position(self, unit: str = 'deg') -> float | None:
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


class _ArduinoAlvikEvents:
    """
    This is a generic events class
    """

    def __init__(self):
        self._callbacks = dict()

    def register_callback(self, event_name: str, callback: callable, args: tuple = None):
        """
        Registers a callback to execute on an event
        :param event_name:
        :param callback: the callable
        :param args: arguments tuple to pass to the callable. remember the comma! (value,)
        :return:
        """
        self._callbacks[event_name] = (callback, args,)

    def has_callbacks(self) -> bool:
        """
        True if the _callbacks dictionary has any callback registered
        :return:
        """
        return bool(self._callbacks)

    def execute_callback(self, event_name: str):
        """
        Executes the callback associated to the event_name
        :param event_name:
        :return:
        """
        if event_name not in self._callbacks.keys():
            return
        self._callbacks[event_name][0](*self._callbacks[event_name][1])


class _ArduinoAlvikTouchEvents(_ArduinoAlvikEvents):
    """
    This is the event class to handle touch button events
    """

    available_events = ['on_ok_pressed', 'on_cancel_pressed',
                        'on_center_pressed', 'on_left_pressed',
                        'on_right_pressed', 'on_up_pressed',
                        'on_down_pressed']

    def __init__(self):
        self._current_touch_state = 0
        super().__init__()

    @staticmethod
    def _is_ok_pressed(current_state, new_state) -> bool:
        """
        True if OK was pressed
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b00000010) and bool(new_state & 0b00000010)

    @staticmethod
    def _is_cancel_pressed(current_state, new_state) -> bool:
        """
        True if CANCEL was pressed
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b00000100) and bool(new_state & 0b00000100)

    @staticmethod
    def _is_center_pressed(current_state, new_state) -> bool:
        """
        True if CENTER was pressed
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b00001000) and bool(new_state & 0b00001000)

    @staticmethod
    def _is_up_pressed(current_state, new_state) -> bool:
        """
        True if UP was pressed
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b00010000) and bool(new_state & 0b00010000)

    @staticmethod
    def _is_left_pressed(current_state, new_state) -> bool:
        """
        True if LEFT was pressed
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b00100000) and bool(new_state & 0b00100000)

    @staticmethod
    def _is_down_pressed(current_state, new_state) -> bool:
        """
        True if DOWN was pressed
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b01000000) and bool(new_state & 0b01000000)

    @staticmethod
    def _is_right_pressed(current_state, new_state) -> bool:
        """
        True if RIGHT was pressed
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b10000000) and bool(new_state & 0b10000000)

    def update_touch_state(self, touch_state: int):
        """
        Updates the internal touch state and executes any possible callback
        :param touch_state:
        :return:
        """

        if self._is_ok_pressed(self._current_touch_state, touch_state):
            self.execute_callback('on_ok_pressed')

        if self._is_cancel_pressed(self._current_touch_state, touch_state):
            self.execute_callback('on_cancel_pressed')

        if self._is_center_pressed(self._current_touch_state, touch_state):
            self.execute_callback('on_center_pressed')

        if self._is_up_pressed(self._current_touch_state, touch_state):
            self.execute_callback('on_up_pressed')

        if self._is_left_pressed(self._current_touch_state, touch_state):
            self.execute_callback('on_left_pressed')

        if self._is_down_pressed(self._current_touch_state, touch_state):
            self.execute_callback('on_down_pressed')

        if self._is_right_pressed(self._current_touch_state, touch_state):
            self.execute_callback('on_right_pressed')

        self._current_touch_state = touch_state

    def register_callback(self, event_name: str, callback: callable, args: tuple = None):
        if event_name not in self.__class__.available_events:
            return
        super().register_callback(event_name, callback, args)


# UPDATE FIRMWARE METHOD #

def update_firmware(file_path: str):
    """

    :param file_path: path of your FW bin
    :return:
    """

    from sys import exit
    from .stm32_flash import (
        CHECK_STM32,
        STM32_endCommunication,
        STM32_startCommunication,
        STM32_NACK,
        STM32_eraseMEM,
        STM32_writeMEM, )

    if CHECK_STM32.value() is not 1:
        print("Turn on your Alvik to continue...")
        while CHECK_STM32.value() is not 1:
            sleep_ms(500)

    ans = STM32_startCommunication()
    if ans == STM32_NACK:
        print("Cannot establish connection with STM32")
        exit(-1)

    print('\nSTM32 FOUND')

    print('\nERASING MEM')
    STM32_eraseMEM(0xFFFF)

    print("\nWRITING MEM")
    STM32_writeMEM(file_path)
    print("\nDONE")
    print("\nLower Boot0 and reset STM32")

    STM32_endCommunication()
