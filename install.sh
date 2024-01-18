#!/bin/bash

if command -v python3 &>/dev/null; then
    python_command="python3"
else
    # If python3 is not available, use python
    python_command="python"
fi

# Uncomment the following line on windows machines
# python_command="python"

$python_command -m mpremote rm :arduino_robot.py
$python_command -m mpremote rm :constants.py
$python_command -m mpremote rm :pinout_definitions.py
$python_command -m mpremote rm :uart.py

$python_command -m mpremote cp arduino_robot.py :arduino_robot.py
$python_command -m mpremote cp constants.py :constants.py
$python_command -m mpremote cp pinout_definitions.py :pinout_definitions.py
$python_command -m mpremote cp uart.py :uart.py

$python_command -m mpremote reset
