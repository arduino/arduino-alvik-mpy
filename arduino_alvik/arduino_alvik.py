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

from .__init__ import __version__
from .__init__ import __required_firmware_version__


def writes_uart(method):
    def wrapper(*args, **kwargs):
        with ArduinoAlvik._write_lock:
            return method(*args, **kwargs)

    return wrapper


def reads_uart(method):
    def wrapper(*args, **kwargs):
        with ArduinoAlvik._read_lock:
            return method(*args, **kwargs)

    return wrapper


class _AlvikRLock:
    def __init__(self):
        """Alvik re-entrant Lock implementation"""
        self._lock = _thread.allocate_lock()
        self._owner = None
        self._count = 0

    def acquire(self):
        tid = _thread.get_ident()

        if self._owner == tid:
            self._count += 1
            return True

        self._lock.acquire()
        self._owner = tid
        self._count = 1
        return True

    def release(self):
        tid = _thread.get_ident()

        if self._owner != tid:
            raise RuntimeError("Cannot release an unowned lock")

        self._count -= 1
        if self._count == 0:
            self._owner = None
            self._lock.release()

    def locked(self):
        return self._lock.locked()

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.release()


class ArduinoAlvik:
    _update_thread_running = False
    _update_thread_id = None
    _events_thread_running = False
    _events_thread_id = None

    _write_lock = _AlvikRLock()
    _read_lock = _AlvikRLock()

    def __new__(cls):
        if not hasattr(cls, '_instance'):
            cls._instance = super(ArduinoAlvik, cls).__new__(cls)
        return cls._instance

    def __init__(self, stack_size = THREAD_STACK_SIZE):
        _thread.stack_size(stack_size)
        self.i2c = _ArduinoAlvikI2C(A4, A5)
        self._packeter = ucPack(200)
        self.left_wheel = _ArduinoAlvikWheel(self._packeter, ord('L'), alvik=self)
        self.right_wheel = _ArduinoAlvikWheel(self._packeter, ord('R'), alvik=self)
        self._servo_positions = list((None, None,))
        self.servo_A = _ArduinoAlvikServo(self._packeter, 'A', 0, self._servo_positions)
        self.servo_B = _ArduinoAlvikServo(self._packeter, 'B', 1, self._servo_positions)
        self._led_state = list((None,))
        self.left_led = self.DL1 = _ArduinoAlvikRgbLed(self._packeter, 'left', self._led_state,
                                                       rgb_mask=[0b00000100, 0b00001000, 0b00010000])
        self.right_led = self.DL2 = _ArduinoAlvikRgbLed(self._packeter, 'right', self._led_state,
                                                        rgb_mask=[0b00100000, 0b01000000, 0b10000000])
        self._battery_perc = None
        self._battery_is_charging = None
        self._touch_byte = None
        self._move_byte = None
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
        self._version = list(map(int, __version__.split('.')))
        self._fw_version = [None, None, None]
        self._required_fw_version = list(map(int, __required_firmware_version__.split('.')))
        self._touch_events = _ArduinoAlvikTouchEvents()
        self._move_events = _ArduinoAlvikMoveEvents()
        self._timer_events = _ArduinoAlvikTimerEvents(-1)

    def __del__(self):
        """
        This method is a stub. __del__ is not implemented in MicroPython (https://docs.micropython.org/en/latest/genrst/core_language.html#special-method-del-not-implemented-for-user-defined-classes)
        :return:
        """
        ...
        # self.__class__._instance = None

    @staticmethod
    def is_on() -> bool:
        """
        Returns true if robot is on
        :return:
        """
        return CHECK_STM32.value() == 1

    @staticmethod
    def _print_battery_status(percentage: float, is_charging) -> None:
        """
        Pretty prints the battery status
        :param percentage: SOC of the battery
        :param is_charging: True if the battery is charging
        :return:
        """
        print("\033[2K\033[1G", end='\r')
        if percentage > 97:
            marks_str = ' \U0001F50B'
        else:
            marks_str = ' \U0001FAAB'
        if is_charging:
            charging_str = ' \U0001F50C                                 '
        else:
            charging_str = ' \U000026A0 WARNING: battery is discharging!'
        word = marks_str + f" {percentage}% {charging_str} \t"
        print(word, end='')

    @staticmethod
    def _lengthy_op(self, iterations=10000000) -> int:
        result = 0
        for i in range(1, iterations):
            result += i * i
        return result

    def _idle(self, delay_=1, check_on_thread=False, blocking=False) -> None:
        """
        Alvik's idle mode behaviour
        :return:
        """
        NANO_CHK.value(1)
        self.i2c.set_single_thread(True)

        if blocking:
            self._lengthy_op(50000)
        else:
            sleep_ms(500)
        led_val = 0

        try:
            while not self.is_on():

                if check_on_thread and not self.__class__._update_thread_running:
                    break

                cmd = bytearray(1)
                cmd[0] = 0x06

                self.i2c.start()
                self.i2c.writeto(0x36, cmd)

                soc_raw = struct.unpack('h', self.i2c.readfrom(0x36, 2))[0]
                soc_perc = soc_raw * 0.00390625
                self._battery_is_charging = soc_perc > 0
                self._battery_perc = abs(soc_perc)
                self._print_battery_status(round(soc_perc), self._battery_is_charging)
                if blocking:
                    self._lengthy_op(10000)
                else:
                    sleep_ms(delay_)
                if soc_perc > 97:
                    LEDG.value(0)
                    LEDR.value(1)
                else:
                    LEDR.value(led_val)
                    LEDG.value(1)
                    led_val = (led_val + 1) % 2
            self.i2c.set_single_thread(False)
            if self.is_on():
                print("\n********** Alvik is on **********")
        except KeyboardInterrupt as e:
            self.stop()
            raise e
        except OSError as e:
            print(f'\nUnable to read SOC: {e}')
            raise e
        except Exception as e:
            print(f'\nUnhandled exception: {e} {type(e)}')
            raise e
        finally:
            LEDR.value(1)
            LEDG.value(1)
            NANO_CHK.value(0)
            self.i2c.set_single_thread(False)

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
            print("\033[2K\033[1G", end='\r')
            pre = ' ' * i
            between = ' ' * (i % 2 + 1)
            post = ' ' * 5
            frame = pre + snake + between + robot + post
            print(frame, end='')
            sleep_ms(200)

    def begin(self) -> int:
        """
        Begins all Alvik operations
        :return:
        """
        if not self.is_on():
            print("\n********** Please turn on your Arduino Alvik! **********\n")
            sleep_ms(1000)
            self.i2c.set_main_thread(_thread.get_ident())
            self._idle(1000)
        self._begin_update_thread()

        sleep_ms(100)

        self._reset_hw()
        self._flush_uart()
        self._snake_robot(1000)
        self._wait_for_ack()
        if not self._wait_for_fw_check():
            self.stop()
            raise Exception('\n********** PLEASE UPDATE ALVIK FIRMWARE (required: '+'.'.join(map(str,self._required_fw_version))+')! Check documentation **********\n')
        self._snake_robot(2000)
        self.set_illuminator(True)
        self.set_behaviour(1)
        self.set_behaviour(2)
        self._set_color_reference()
        if self._has_events_registered():
            print('\n********** Starting events thread **********\n')
            self._start_events_thread()
        self.set_servo_positions(90, 90)
        return 0

    def _has_events_registered(self) -> bool:
        """
        Returns True if Alvik has some events registered
        :return:
        """

        return any([
            self._touch_events.has_callbacks(),
            self._move_events.has_callbacks(),
            self._timer_events.has_callbacks()
            # more events check
        ])

    def _wait_for_ack(self) -> None:
        """
        Waits until receives 0x00 ack from robot
        :return:
        """
        self._waiting_ack = 0x00
        while self._last_ack != 0x00:
            sleep_ms(20)
        self._waiting_ack = None

    def _wait_for_fw_check(self, timeout=5) -> bool:
        """
        Waits until receives version from robot, check required version and return true if everything is ok
        :param timeout: wait for fw timeout in seconds
        :return:
        """
        start = ticks_ms()
        while self._fw_version == [None, None, None]:
            sleep_ms(20)
            if ticks_diff(ticks_ms(), start) > timeout * 1000:
                print("Could not get FW version")
                return False

        if self.check_firmware_compatibility():
            return True
        else:
            return False

    @staticmethod
    @reads_uart
    def _flush_uart():
        """
        Empties the UART buffer
        :return:
        """
        uart.read(uart.any())

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

    @writes_uart
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
            self._waiting_ack = None
            self._last_ack = 0x00
            sleep_ms(100)
            return True
        return False

    @writes_uart
    def set_behaviour(self, behaviour: int):
        """
        Sets the behaviour of Alvik
        :param behaviour: behaviour code
        :return:
        """
        self._packeter.packetC1B(ord('B'), behaviour & 0xFF)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    @writes_uart
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

    @writes_uart
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
        self._stop_events_thread()

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

    @writes_uart
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

    @writes_uart
    def set_wheels_position(self, left_angle: float, right_angle: float, unit: str = 'deg', blocking: bool = True):
        """
        Sets left/right motor angle
        :param left_angle:
        :param right_angle:
        :param unit: the speed unit of measurement (default: 'deg')
        :param blocking:
        :return:
        """
        left_angle = convert_angle(left_angle, unit, 'deg')
        right_angle = convert_angle(right_angle, unit, 'deg')
        self._packeter.packetC2F(ord('A'), left_angle, right_angle)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])
        self._waiting_ack = ord('P')
        if blocking:
            self._wait_for_target(idle_time=(max(left_angle, right_angle) / MOTOR_CONTROL_DEG_S))

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

    @writes_uart
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

    @writes_uart
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

    @writes_uart
    def set_servo_positions(self, a_position: int, b_position: int):
        """
        Sets A/B servomotor angle
        :param a_position: position of A servomotor (0-180)
        :param b_position: position of B servomotor (0-180)
        :return:
        """
        self._servo_positions[0] = a_position
        self._servo_positions[1] = b_position
        self._packeter.packetC2B(ord('S'), a_position & 0xFF, b_position & 0xFF)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def get_servo_positions(self) -> (int, int):
        """
        Returns the current servomotor positions
        :return: position of A/B servomotor (0-180)
        """

        return self._servo_positions[0], self._servo_positions[1]

    def get_ack(self) -> str:
        """
        Returns last acknowledgement
        :return:
        """
        return self._last_ack

    @writes_uart
    def send_ack(self, ack: str = 'K'):
        """
        Sends an ack message on UART
        :return:
        """
        self._packeter.packetC1B(ord('X'), ord(ack))
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    @writes_uart
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

        self.i2c.set_main_thread(_thread.get_ident())

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
                self.set_behaviour(2)
            if not ArduinoAlvik._update_thread_running:
                break
            self._read_message()
            sleep_ms(delay_)

    @reads_uart
    def _read_message(self) -> None:
        """
        Read a message from the uC
        :return: True if a message terminator was reached
        """
        buf = bytearray(uart.any())
        uart.readinto(buf)
        if len(buf):
            uart.readinto(buf)
            for b in buf:
                self._packeter.buffer.push(b)
                if b == self._packeter.end_index and self._packeter.checkPayload():
                    self._parse_message()

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
            _, battery_perc = self._packeter.unpacketC1F()
            self._battery_is_charging = battery_perc > 0
            self._battery_perc = abs(battery_perc)
        elif code == ord('d'):
            # distance sensor
            _, self._left_tof, self._center_tof, self._right_tof = self._packeter.unpacketC3I()
        elif code == ord('t'):
            # touch input
            _, self._touch_byte = self._packeter.unpacketC1B()
        elif code == ord('m'):
            # tilt/shake input
            _, self._move_byte = self._packeter.unpacketC1B()
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
            _, *self._fw_version = self._packeter.unpacketC3B()
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

    def is_battery_charging(self) -> bool:
        """
        Returns True if the device battery is charging
        :return:
        """
        return self._battery_is_charging

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

    @property
    def _move_bits(self) -> int:
        """
        Returns the shake/tilt state
        :return:
        """
        return (self._move_byte & 0xFF) if self._move_byte is not None else 0x80

    def get_shake(self) -> bool:
        """
        Returns true if Alvik is shaken
        :return:
        """
        return bool(self._move_bits & 0b00000001)

    def get_lifted(self) -> bool:
        """
        Returns true if Alvik is lifted
        :return:
        """
        return bool(self._move_bits & 0b00000010)

    def get_tilt(self) -> str:
        """
        Returns the tilt string eg: "X", "-Z" etc
        :return:
        """

        if bool(self._move_bits & 0b00000100):
            return "X"
        if bool(self._move_bits & 0b00001000):
            return "-X"
        if bool(self._move_bits & 0b00010000):
            return "Y"
        if bool(self._move_bits & 0b00100000):
            return "-Y"
        if bool(self._move_bits & 0b01000000):
            return "Z"
        if bool(self._move_bits & 0b10000000):
            return "-Z"

        return ""

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

    def get_version(self, version: str = 'fw') -> str:
        """
        Returns the version of the Alvik firmware or micropython library
        :param version:
        :return:
        """
        if version == 'fw' or version == 'FW' or version == 'firmware':
            return self.get_fw_version()
        elif version == 'lib' or version == 'LIB':
            return self.get_lib_version()
        else:
            return f'{None, None, None}'

    def get_lib_version(self) -> str:
        """
        Returns the micropython library version of the Alvik
        :return:
        """
        return f'{self._version[0]}.{self._version[1]}.{self._version[2]}'
    
    def get_fw_version(self) -> str:
        """
        Returns the firmware version of the Alvik Carrier
        :return:
        """
        return f'{self._fw_version[0]}.{self._fw_version[1]}.{self._fw_version[2]}'
    
    def get_required_fw_version(self) -> str:
        """
        Returns the required firmware version of the Alvik Carrier for this micropython library
        :return:
        """
        return f'{self._required_fw_version[0]}.{self._required_fw_version[1]}.{self._required_fw_version[2]}'
    
    def check_firmware_compatibility(self) -> bool:
        """
        Returns true if the library and the firmware are compatible
        :return:
        """
        return self._fw_version == self._required_fw_version

    def print_status(self):
        """
        Prints the Alvik status
        :return:
        """
        print('---ALVIK STATUS---')
        print(f'LIBRARY VERSION: {self._version}')
        print(f'REQUIRED FW VERSION: {self._required_fw_version}')
        print(f'FIRMWARE VERSION: {self._fw_version}')

        print('---SENSORS---')
        print(f'TOF: T:{self._top_tof} B:{self._bottom_tof} L:{self._left_tof} CL:{self._center_left_tof}' +
              f' C:{self._center_tof} CR:{self._center_right_tof} R:{self._right_tof}')
        print(f'LINE: L:{self._left_line} C:{self._center_line} R:{self._right_line}')
        print(f'ACC: X:{self._ax} Y:{self._ay} Z:{self._az}')
        print(f'GYR: X:{self._gx} Y:{self._gy} Z:{self._gz}')
        print(f'POS: X:{self._x} Y:{self._y} TH:{self._theta}')
        print(f'IMU: ROLL:{self._roll} PITCH:{self._pitch} YAW:{self._yaw}')
        print(f'COLOR: R:{self._red} G:{self._green} B:{self._blue}')
        print(f'BATT(%) {self._battery_perc}')

        print('---COMMUNICATION---')
        print(f'TOUCH BYTE: {self._touch_byte}')
        print(f'LAST ACK: {self._last_ack}')

        print('---MOTORS---')
        print(f'LINEAR VEL: {self._linear_velocity}')
        print(f'ANGULAR VEL: {self._angular_velocity}')

    def set_timer(self, mode: str, period: int, callback: callable, args: tuple = ()) -> None:
        """
        Register a timer callback
        :param mode: _ArduinoAlvikTimerEvents.PERIODIC or .ONE_SHOT
        :param period: period in milliseconds
        :param callback:
        :param args:
        :return:
        """

        self._timer_events = _ArduinoAlvikTimerEvents(period)
        self._timer_events.register_callback(mode, callback, args)

    @property
    def timer(self):
        """
        Gives access to the timer object
        :return:
        """
        return self._timer_events

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

    def on_shake(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when Alvik is shaken
        :param callback:
        :param args:
        :return:
        """
        self._move_events.register_callback('on_shake', callback, args)

    def on_lift(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when Alvik is lifted
        :param callback:
        :param args:
        :return:
        """
        self._move_events.register_callback('on_lift', callback, args)

    def on_drop(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when Alvik is dropped
        :param callback:
        :param args:
        :return:
        """
        self._move_events.register_callback('on_drop', callback, args)

    def on_x_tilt(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when Alvik is tilted on X-axis
        :param callback:
        :param args:
        :return:
        """
        self._move_events.register_callback('on_x_tilt', callback, args)

    def on_nx_tilt(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when Alvik is tilted on negative X-axis
        :param callback:
        :param args:
        :return:
        """
        self._move_events.register_callback('on_nx_tilt', callback, args)

    def on_y_tilt(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when Alvik is tilted on Y-axis
        :param callback:
        :param args:
        :return:
        """
        self._move_events.register_callback('on_y_tilt', callback, args)

    def on_ny_tilt(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when Alvik is tilted on negative Y-axis
        :param callback:
        :param args:
        :return:
        """
        self._move_events.register_callback('on_ny_tilt', callback, args)

    def on_z_tilt(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when Alvik is tilted on Z-axis
        :param callback:
        :param args:
        :return:
        """
        self._move_events.register_callback('on_z_tilt', callback, args)

    def on_nz_tilt(self, callback: callable, args: tuple = ()) -> None:
        """
        Register callback when Alvik is tilted on negative Z-axis
        :param callback:
        :param args:
        :return:
        """
        self._move_events.register_callback('on_nz_tilt', callback, args)

    def _start_events_thread(self) -> None:
        """
        Starts the touch events thread
        :return:
        """
        if not self.__class__._events_thread_running:
            self.__class__._events_thread_running = True
            self._timer_events.reset()                                     # resets the timer before starting
            self._move_events.reset(_ArduinoAlvikMoveEvents.NZ_TILT)       # resets the orientation to -Z tilted
            self.__class__._events_thread_id = _thread.start_new_thread(self._update_events, (50,))

    def _update_events(self, delay_: int = 100):
        """
        Updates the touch state so that touch events can be generated
        :param delay_:
        :return:
        """
        while True:
            if not ArduinoAlvik._events_thread_running:
                break

            if self.is_on():
                self._touch_events.update_state(self._touch_byte)
                self._move_events.update_state(self._move_byte)
                self._timer_events.update_state(ticks_ms())
                # MORE events update callbacks to be added

            sleep_ms(delay_)

    @classmethod
    def _stop_events_thread(cls):
        """
        Stops the touch events thread
        :return:
        """
        cls._events_thread_running = False


class _ArduinoAlvikI2C:

    _main_thread_id = None

    def __init__(self, sda: int, scl: int):
        """
        Alvik I2C wrapper
        :param sda:
        :param scl:
        """
        self._lock = _thread.allocate_lock()

        self._is_single_thread = False

        self.sda = sda
        self.scl = scl

    def set_main_thread(self, thread_id: int):
        """
        Sets the main thread of control. It will be the only thread allowed if set_single_thread is True
        """
        with self._lock:
            self.__class__._main_thread_id = thread_id

    def set_single_thread(self, value):
        """
        Sets the single thread mode on/off.
        In single mode only the main thread is allowed to access the bus
        """
        self._is_single_thread = value

    def is_accessible(self):
        """
        Returns True if bus is accessible by the current thread
        """
        return not self._is_single_thread or _thread.get_ident() == self.__class__._main_thread_id

    def start(self):
        """
        Bitbanging start condition
        :return:
        """
        _SDA = Pin(self.sda, Pin.OUT)
        _SDA.value(1)
        sleep_ms(100)
        _SDA.value(0)

    def init(self, scl, sda, freq=400_000) -> None:
        """
        init method just for call compatibility
        """
        print("AlvikWarning:: init Unsupported. Alvik defines/initializes its own I2C bus")

    def deinit(self):
        """
        deinit method just for call compatibility
        """
        print("AlvikWarning:: deinit Unsupported. Alvik defines/initializes its own I2C bus")

    def stop(self):
        """ Bitbanging stop condition (untested)
        :return:
        """
        _SDA = Pin(self.sda, Pin.OUT)
        _SDA.value(0)
        sleep_ms(100)
        _SDA.value(1)

    def scan(self) -> list[int]:
        """
        I2C scan method
        :return:
        """
        if not self.is_accessible():
            return []
        with self._lock:
            i2c = I2C(0, scl=Pin(self.scl, Pin.OUT), sda=Pin(self.sda, Pin.OUT))
            return i2c.scan()

    def readfrom(self, addr, nbytes, stop=True) -> bytes:
        """
        Wrapping i2c readfrom
        """
        if not self.is_accessible():
            return bytes(nbytes)
        with self._lock:
            i2c = I2C(0, scl=Pin(self.scl, Pin.OUT), sda=Pin(self.sda, Pin.OUT))
            return i2c.readfrom(addr, nbytes, stop)

    def writeto(self, addr, buf, stop=True) -> int:
        """
        Wrapping i2c writeto
        """
        if not self.is_accessible():
            return 0
        with self._lock:
            i2c = I2C(0, scl=Pin(self.scl, Pin.OUT), sda=Pin(self.sda, Pin.OUT))
            return i2c.writeto(addr, buf, stop)

    def readinto(self, buf, nack=True) -> None:
        """
        Wrapping i2c readinto
        """
        if not self.is_accessible():
            return
        with self._lock:
            i2c = I2C(0, scl=Pin(self.scl, Pin.OUT), sda=Pin(self.sda, Pin.OUT))
            return i2c.readinto(buf, nack)

    def write(self, buf) -> int:
        """
        Wrapping i2c write
        """
        if not self.is_accessible():
            return 0
        with self._lock:
            i2c = I2C(0, scl=Pin(self.scl, Pin.OUT), sda=Pin(self.sda, Pin.OUT))
            return i2c.write(buf)

    def readfrom_into(self, addr, buf, stop=True) -> None:
        """
        Wrapping i2c readfrom_into
        """
        if not self.is_accessible():
            return
        with self._lock:
            i2c = I2C(0, scl=Pin(self.scl, Pin.OUT), sda=Pin(self.sda, Pin.OUT))
            return i2c.readfrom_into(addr, buf, stop)

    def writevto(self, addr, vector, stop=True) -> int:
        """
        Wrapping i2c writevto
        """
        if not self.is_accessible():
            return 0
        with self._lock:
            i2c = I2C(0, scl=Pin(self.scl, Pin.OUT), sda=Pin(self.sda, Pin.OUT))
            return i2c.writevto(addr, vector, stop)

    def readfrom_mem(self, addr, memaddr, nbytes, addrsize=8) -> bytes:
        """
        Wrapping i2c readfrom_mem
        """
        if not self.is_accessible():
            return bytes(nbytes)
        with self._lock:
            i2c = I2C(0, scl=Pin(self.scl, Pin.OUT), sda=Pin(self.sda, Pin.OUT))
            return i2c.readfrom_mem(addr, memaddr, nbytes, addrsize=addrsize)

    def readfrom_mem_into(self, addr, memaddr, buf, addrsize=8) -> None:
        """
        Wrapping i2c readfrom_mem_into
        """
        if not self.is_accessible():
            return
        with self._lock:
            i2c = I2C(0, scl=Pin(self.scl, Pin.OUT), sda=Pin(self.sda, Pin.OUT))
            return i2c.readfrom_mem_into(addr, memaddr, buf, addrsize=addrsize)

    def writeto_mem(self, addr, memaddr, buf, addrsize=8) -> None:
        """
        Wrapping i2c writeto_mem
        """
        if not self.is_accessible():
            return
        with self._lock:
            i2c = I2C(0, scl=Pin(self.scl, Pin.OUT), sda=Pin(self.sda, Pin.OUT))
            return i2c.writeto_mem(addr, memaddr, buf, addrsize=addrsize)


class _ArduinoAlvikServo:

    def __init__(self, packeter: ucPack, label: str, servo_id: int, position: list[int | None]):
        self._packeter = packeter
        self._label = label
        self._id = servo_id
        self._position = position
    
    @writes_uart
    def set_position(self, position):
        """
        Sets the position of the servo
        :param position:
        :return:
        """
        self._position[self._id] = position
        self._packeter.packetC2B(ord('S'), self._position[0] & 0xFF, self._position[1] & 0xFF)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])

    def get_position(self) -> int:
        """
        Returns the position of the servo
        :return:
        """
        return self._position[self._id]


class _ArduinoAlvikWheel:

    def __init__(self, packeter: ucPack, label: int, wheel_diameter_mm: float = WHEEL_DIAMETER_MM,
                 alvik: ArduinoAlvik = None):
        self._packeter = packeter
        self._label = label
        self._wheel_diameter_mm = wheel_diameter_mm
        self._speed = None
        self._position = None
        self._alvik = alvik
    
    @writes_uart
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

    @writes_uart
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

    @writes_uart
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

    @writes_uart
    def set_position(self, position: float, unit: str = 'deg', blocking: bool = True):
        """
        Sets left/right motor speed
        :param position: the position of the motor
        :param unit: the unit of measurement
        :param blocking:
        :return:
        """
        position = convert_angle(position, unit, 'deg')
        self._packeter.packetC2B1F(ord('W'), self._label & 0xFF, ord('P'), position)
        uart.write(self._packeter.msg[0:self._packeter.msg_size])
        self._alvik._waiting_ack = ord('P')
        if blocking:
            self._alvik._wait_for_target(idle_time=(position / MOTOR_CONTROL_DEG_S))

    def is_target_reached(self):
        """
        Checks if the target position is reached
        :return:
        """
        return self._alvik.is_target_reached()


class _ArduinoAlvikRgbLed:
    def __init__(self, packeter: ucPack, label: str, led_state: list[int | None], rgb_mask: list[int]):
        self._packeter = packeter
        self.label = label
        self._rgb_mask = rgb_mask
        self._led_state = led_state

    @writes_uart
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

    available_events = []

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

        if event_name not in self.__class__.available_events:
            return
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

    def update_state(self, state):
        """
        Updates the internal state of the events handler
        :return:
        """
        pass


class _ArduinoAlvikTimerEvents(_ArduinoAlvikEvents):
    """
    Event class to handle timer events
    """

    available_events = ['periodic', 'one_shot']
    PERIODIC = 'periodic'
    ONE_SHOT = 'one_shot'

    def __init__(self, period: int):
        """
        Timer initialization
        :param period: Timer period in milliseconds
        """
        self._last_trigger = ticks_ms()
        self._period = period
        self._triggered = False
        self._stopped = False
        super().__init__()

    def is_triggered(self):
        """
        Returns the trigger state
        :return:
        """
        return self._triggered

    def is_stopped(self):
        """
        Return True if timer is stopped
        :return:
        """
        return self._stopped

    def set(self, start=None, period: int = None):
        """
        Sets the last trigger time
        :param start:
        :param period:
        :return:
        """
        self._last_trigger = start if start is not None else ticks_ms()
        if period is not None:
            self._period = period

    def reset(self, start=None, period: int = None):
        """
        Resets the timer. Use just before starting the events thread or if you want to restart the Timer
        :param start:
        :param period:
        :return:
        """
        self._last_trigger = start if start is not None else ticks_ms()
        if period is not None:
            self._period = period
        self._triggered = False

    def stop(self):
        """
        Stops the timer
        :return:
        """

        self._stopped = True

    def resume(self):
        """
        Resumes the timer
        :return:
        """
        self._stopped = False

    def get(self) -> int:
        """
        Returns the time passed since the last trigger in ms
        :return:
        """
        return ticks_diff(ticks_ms(), self._last_trigger)

    def register_callback(self, event_name: str, callback: callable, args: tuple = None):
        """
        Repeated calls to register_callback will overwrite the timer's behaviour. The Timer can be either PERIODIC
         or ONE_SHOT
        :param event_name:
        :param callback:
        :param args:
        :return:
        """
        self._callbacks = dict()
        super().register_callback(event_name, callback, args)

    def _is_period_expired(self, now=None) -> bool:
        """
        True if the timer period is expired
        :return:
        """

        if now is None:
            now = ticks_ms()
        return ticks_diff(now, self._last_trigger) > self._period

    def update_state(self, ticks):
        """
        Updates the internal state of the events handler and executes the related callback
        :return:
        """

        if list(self._callbacks.keys()) == [self.PERIODIC]:
            if self._is_period_expired(ticks):
                self._last_trigger = ticks
                if not self._stopped:
                    self.execute_callback(self.PERIODIC)
        elif list(self._callbacks.keys()) == [self.ONE_SHOT] and not self._triggered:
            if self._is_period_expired(ticks):
                self._last_trigger = ticks
                if not self._stopped:
                    self.execute_callback(self.ONE_SHOT)
                self._triggered = True


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

    def update_state(self, state: int | None):
        """
        Updates the internal touch state and executes any possible callback
        :param state:
        :return:
        """

        if state is None:
            return

        if self._is_ok_pressed(self._current_touch_state, state):
            self.execute_callback('on_ok_pressed')

        if self._is_cancel_pressed(self._current_touch_state, state):
            self.execute_callback('on_cancel_pressed')

        if self._is_center_pressed(self._current_touch_state, state):
            self.execute_callback('on_center_pressed')

        if self._is_up_pressed(self._current_touch_state, state):
            self.execute_callback('on_up_pressed')

        if self._is_left_pressed(self._current_touch_state, state):
            self.execute_callback('on_left_pressed')

        if self._is_down_pressed(self._current_touch_state, state):
            self.execute_callback('on_down_pressed')

        if self._is_right_pressed(self._current_touch_state, state):
            self.execute_callback('on_right_pressed')

        self._current_touch_state = state


class _ArduinoAlvikMoveEvents(_ArduinoAlvikEvents):
    """
    Event class to handle move events
    """

    available_events = ['on_shake', 'on_lift', 'on_drop', 'on_x_tilt', 'on_y_tilt', 'on_z_tilt',
                        'on_nx_tilt', 'on_ny_tilt', 'on_nz_tilt']

    NZ_TILT = 0x80

    def __init__(self):
        self._current_state = 0
        super().__init__()

    def reset(self, state: int = 0x00):
        """
        Sets the initial state
        :param state:
        :return:
        """
        self._current_state = state

    @staticmethod
    def _is_shaken(current_state, new_state) -> bool:
        """
        True if Alvik was shaken
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b00000001) and bool(new_state & 0b00000001)

    @staticmethod
    def _is_lifted(current_state, new_state) -> bool:
        """
        True if Alvik was lifted
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b00000010) and bool(new_state & 0b00000010)

    @staticmethod
    def _is_dropped(current_state, new_state) -> bool:
        """
        True if Alvik was dropped
        :param current_state:
        :param new_state:
        :return:
        """
        return bool(current_state & 0b00000010) and not bool(new_state & 0b00000010)

    @staticmethod
    def _is_x_tilted(current_state, new_state) -> bool:
        """
        True if Alvik is tilted on X-axis
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b00000100) and bool(new_state & 0b00000100)

    @staticmethod
    def _is_neg_x_tilted(current_state, new_state) -> bool:
        """
        True if Alvik is tilted on negative X-axis
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b00001000) and bool(new_state & 0b00001000)

    @staticmethod
    def _is_y_tilted(current_state, new_state) -> bool:
        """
        True if Alvik is tilted on Y-axis
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b00010000) and bool(new_state & 0b00010000)

    @staticmethod
    def _is_neg_y_tilted(current_state, new_state) -> bool:
        """
        True if Alvik is tilted on negative Y-axis
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b00100000) and bool(new_state & 0b00100000)

    @staticmethod
    def _is_z_tilted(current_state, new_state) -> bool:
        """
        True if Alvik is tilted on Z-axis
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b01000000) and bool(new_state & 0b01000000)

    @staticmethod
    def _is_neg_z_tilted(current_state, new_state) -> bool:
        """
        True if Alvik is tilted on negative Z-axis
        :param current_state:
        :param new_state:
        :return:
        """
        return not bool(current_state & 0b10000000) and bool(new_state & 0b10000000)

    def update_state(self, state: int | None):
        """
        Updates the internal state and executes any possible callback
        :param state:
        :return:
        """

        if state is None:
            return

        if self._is_shaken(self._current_state, state):
            self.execute_callback('on_shake')

        if self._is_lifted(self._current_state, state):
            self.execute_callback('on_lift')

        if self._is_dropped(self._current_state, state):
            self.execute_callback('on_drop')

        if self._is_x_tilted(self._current_state, state):
            self.execute_callback('on_x_tilt')

        if self._is_neg_x_tilted(self._current_state, state):
            self.execute_callback('on_nx_tilt')

        if self._is_y_tilted(self._current_state, state):
            self.execute_callback('on_y_tilt')

        if self._is_neg_y_tilted(self._current_state, state):
            self.execute_callback('on_ny_tilt')

        if self._is_z_tilted(self._current_state, state):
            self.execute_callback('on_z_tilt')

        if self._is_neg_z_tilted(self._current_state, state):
            self.execute_callback('on_nz_tilt')

        self._current_state = state


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

    def flash_toggle():
        i = 0

        while True:
            if i == 0:
                LEDR.value(1)
                LEDG.value(0)
            else:
                LEDR.value(0)
                LEDG.value(1)
            i = (i + 1) % 2
            yield

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
    toggle = flash_toggle()
    STM32_writeMEM(file_path, toggle)

    print("\nDONE")
    print("\nLower Boot0 and reset STM32")

    LEDR.value(1)
    LEDG.value(1)

    STM32_endCommunication()
