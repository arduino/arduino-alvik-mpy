from time import sleep_ms


from arduino_alvik import ArduinoAlvik

alvik = ArduinoAlvik()
alvik.begin()

sleep_ms(1000)

while True:
    try:
        out = alvik.i2c.scan()

        if len(out) == 0:
            print("\nNo device found on I2C")
        else:
            print("\nList of devices")
            for o in out:
                print(hex(o))

        sleep_ms(100)
    except KeyboardInterrupt as e:
        alvik.stop()
        break