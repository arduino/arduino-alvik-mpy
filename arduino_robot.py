from uart import uart
import _thread
from time import sleep_ms
from ucPack import ucPack


class ArduinoRobot:

    def __init__(self):
        self.packeter = ucPack(200)
        self._update_thread_running = False
        self._update_thread_id = None
        self.l_speed = None
        self.r_speed = None
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

    def run(self):
        """
        Runs robot background operations (e.g. threaded update)
        :return:
        """
        self._update_thread_running = True
        self._update_thread_id = _thread.start_new_thread(self._update, (1,))

    def stop(self):
        """
        Stops the background operations
        :return:
        """
        self._update_thread_running = False

    def set_speed(self, left_speed, right_speed):
        """
        Sets left/right motor speed
        :param left_speed:
        :param right_speed:
        :return:
        """
        self.packeter.packetC2F(ord('J'), left_speed, right_speed)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def set_servo_angle(self, left_angle, right_angle):
        """
        Sets left/right motor angle
        :param left_angle:
        :param right_angle:
        :return:
        """
        self.packeter.packetC2B(ord('S'), left_angle, right_angle)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    # def send_ack(self):
    #     self.packeter.packetC1B(ord('X'), ACK_)
    #     uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def set_leds(self, led_state):
        """
        Sets the LEDs state
        :param led_state:
        :return:
        """
        self.packeter.packetC1B(ord('L'), led_state)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def _update(self, delay_=1):
        """
        Updates the robot status reading/parsing messages from UART.
        This method is blocking and meant as a thread callback
        Use the method stop to terminate _update and exit the thread
        :param delay_: while loop delay
        :param id_:
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
            _, self.red, self.green, self.blue = self.packeter.unpacketC3B()
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

    def get_version(self):
        return f'{self.version[0]}.{self.version[1]}.{self.version[2]}'

    def print_status(self):
        for a in vars(self):
            if str(a).startswith('_'):
                continue
            print(f'{str(a).upper()} = {getattr(self, str(a))}')

    def get_speed(self):
        pass
        #self.packeter.packetC2F(ord('J'), left_speed, right_speed)
