python -m mpremote rm :arduino_robot.py
python -m mpremote rm :constants.py
python -m mpremote rm :pinout_definitions.py
python -m mpremote rm :uart.py

python -m mpremote cp arduino_robot.py :arduino_robot.py
python -m mpremote cp constants.py :constants.py
python -m mpremote cp pinout_definitions.py :pinout_definitions.py
python -m mpremote cp uart.py :uart.py

python -m mpremote reset