from arduino_alvik import ArduinoAlvik
from time import sleep_ms


try:
    from modulino import ModulinoPixels
except ImportError as e:
    print("\nImportError: ModulinoPixels not installed")
    raise e

alvik = ArduinoAlvik()
alvik.begin()

pixels = ModulinoPixels(alvik.i2c)

if not pixels.connected:
    print("🤷 No pixel modulino found")
    sys.exit(-2)

while True:
    try:
        for i in range(0, 8):
            pixels.clear_all()
            pixels.set_rgb(i, 255, 0, 0, 100)
            pixels.show()
            sleep_ms(50)

        for i in range(7, -1, -1):
            pixels.clear_all()
            pixels.set_rgb(i, 255, 0, 0, 100)
            pixels.show()
            sleep_ms(50)

    except KeyboardInterrupt as e:
        alvik.stop()
        break
