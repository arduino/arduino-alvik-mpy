from arduino_robot import ArduinoRobot

robot = ArduinoRobot()
print("flashing blink fast example")
robot.flash_firmware("Blink_fast.bin")
print("done!")
robot.run()
