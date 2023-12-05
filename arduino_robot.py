from uart import uart
import _thread
from time import sleep_ms
from ucPack import ucPack


class ArduinoRobot:

    def __init__(self):
        self.packeter = ucPack(200)
        self.l_speed = 0
        self.r_speed = 0
        self.battery_perc = 0.0
        self.touch_bits = 0
        self.behaviour = 0
        self.red = 0
        self.green = 0
        self.blue = 0
        self.left_line = 0
        self.center_line = 0
        self.right_line = 0

    def run(self):
        _thread.start_new_thread(self.update, (1, 1))

    def set_speed(self, left_speed, right_speed):
        self.packeter.packetC2F(ord('J'), left_speed, right_speed)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def set_servo_angle(self, left_angle, right_angle):
        self.packeter.packetC2B(ord('S'), left_angle, right_angle)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    # def send_ack(self):
    #     self.packeter.packetC1B(ord('X'), ACK_)
    #     uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def set_leds(self, led_state):
        self.packeter.packetC1B(ord('L'), led_state)
        uart.write(self.packeter.msg[0:self.packeter.msg_size])

    def update(self, delay_, id_):
        while True:
            if self.read_message():
                self.parse_message()
            sleep_ms(delay_)

    def read_message(self) -> bool:
        while uart.any():
            b = uart.read(1)[0]
            self.packeter.buffer.push(b)
            if b == self.packeter.end_index:
                self.packeter.checkPayload()
                return True
        return False

    def parse_message(self) -> int:
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
            _, left_tof, center_tof, right_tof = self.packeter.unpacketC3I()
        elif code == ord('t'):
            # touch input
            _, self.touch_bits = self.packeter.unpacketC1B()
        elif code == ord('b'):
            # behaviour
            _, self.behaviour = self.packeter.unpacketC1B()
        elif code == ord('f'):
            # tof matrix
            # _, left, center_left, center,	center_right, right, bottom, top = self.packeter.unpacketC7I()
            pass
        else:
            return -1

        return 0

    def get_speed(self):
        pass
        #self.packeter.packetC2F(ord('J'), left_speed, right_speed)