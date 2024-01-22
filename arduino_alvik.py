from uart import uart
import _thread
from time import sleep_ms

from ucPack import ucPack

from pinout_definitions import *
from constants import *


class ArduinoAlvik:

    def __init__(self):
        self.packeter = ucPack(200)
        self._update_thread_running = False
        self._update_thread_id = None
        self.l_speed = None
        self.r_speed = None
        self.battery_perc = None
        self.touch_bits = None
        self.behaviour = None
        self.led_state = None
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
        if not CHECK_STM32.value():
            print("\nTurn on your Arduino Alvik!\n")
            return -1
        self._begin_update_thread()
        sleep_ms(100)
        self._reset_hw()
        return 0

    def _begin_update_thread(self):
        """
        Runs robot background operations (e.g. threaded update)
        :return:
        """
        self._update_thread_running = True
        self._update_thread_id = _thread.start_new_thread(self._update, (1,))

    def _stop_update_thread(self):
        """
        Stops the background operations
        :return:
        """
        self._update_thread_running = False

    def stop(self):
        """
        Stops all Alvik operations
        :return:
        """
        # stop engines
        # turn off UI leds
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

    def get_speeds(self) -> (float, float):
        return self.l_speed, self.r_speed

    def set_speeds(self, left_speed: float, right_speed: float):
        """
        Sets left/right motor speed
        :param left_speed:
        :param right_speed:
        :return:
        """
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
        :return:
        """

        return self.roll, self.pitch, self.yaw

    def get_line_sensors(self) -> (int, int, int):
        """
        Returns the line sensors readout
        :return:
        """

        return self.left_line, self.center_line, self.right_line

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
        self.led_state = led_state & 0xFF
        self.packeter.packetC1B(ord('L'), self.led_state)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def set_builtin_led(self, value: bool):
        if self.led_state is None:
            self._set_leds(0x00)
        self.led_state = self.led_state | 0b00000001 if value else self.led_state & 0b11111110
        self._set_leds(self.led_state)

    def set_illuminator(self, value: bool):
        if self.led_state is None:
            self._set_leds(0x00)
        self.led_state = self.led_state | 0b00000010 if value else self.led_state & 0b11111101
        self._set_leds(self.led_state)

    def set_left_led_color(self, red: bool, green: bool, blue: bool):
        if self.led_state is None:
            self._set_leds(0x00)
        self.led_state = self.led_state | 0b00000100 if red else self.led_state & 0b11111011
        self.led_state = self.led_state | 0b00001000 if green else self.led_state & 0b11110111
        self.led_state = self.led_state | 0b00010000 if blue else self.led_state & 0b11101111
        self._set_leds(self.led_state)

    def set_right_led_color(self, red: bool, green: bool, blue: bool):
        if self.led_state is None:
            self._set_leds(0x00)
        self.led_state = self.led_state | 0b00100000 if red else self.led_state & 0b11011111
        self.led_state = self.led_state | 0b01000000 if green else self.led_state & 0b10111111
        self.led_state = self.led_state | 0b10000000 if blue else self.led_state & 0b01111111
        self._set_leds(self.led_state)

    def _update(self, delay_=1):
        """
        Updates the robot status reading/parsing messages from UART.
        This method is blocking and meant as a thread callback
        Use the method stop to terminate _update and exit the thread
        :param delay_: while loop delay
        :return:
        """
        while True:
            if not self._update_thread_running:
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
            _, self.l_speed, self.r_speed = self.packeter.unpacketC2F()
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
        return self.touch_bits

    def get_touch_any(self) -> bool:
        return bool(self.touch_bits & 0b00000001)

    def get_touch_ok(self) -> bool:
        return bool(self.touch_bits & 0b00000010)

    def get_touch_cancel(self) -> bool:
        return bool(self.touch_bits & 0b00000100)

    def get_touch_center(self) -> bool:
        return bool(self.touch_bits & 0b00001000)

    def get_touch_up(self) -> bool:
        return bool(self.touch_bits & 0b00010000)

    def get_touch_left(self) -> bool:
        return bool(self.touch_bits & 0b00100000)

    def get_touch_down(self) -> bool:
        return bool(self.touch_bits & 0b01000000)

    def get_touch_right(self) -> bool:
        return bool(self.touch_bits & 0b10000000)

    def get_color(self) -> (int, int, int):
        """
        Returns the RGB color readout
        :return:
        """

        return self.red, self.green, self.blue
        # return (int((self.red/COLOR_FULL_SCALE)*255),
        #         int((self.green/COLOR_FULL_SCALE)*255),
        #         int((self.blue/COLOR_FULL_SCALE)*255))

    def get_distance(self) -> (int, int, int, int, int, int):
        return self.left_tof, self.center_left_tof, self.center_tof, self.center_right_tof, self.right_tof

    def get_version(self) -> str:
        return f'{self.version[0]}.{self.version[1]}.{self.version[2]}'

    def print_status(self):
        for a in vars(self):
            if str(a).startswith('_'):
                continue
            print(f'{str(a).upper()} = {getattr(self, str(a))}')
