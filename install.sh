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

echo "Do you need flash firmware utilities? (Y/n)"
read -r needs_flash
needs_flash="${needs_flash:-y}"

if [ "$needs_flash" == "y" ]; then
    $python_command -m mpremote rm :firmware_updater.py
    $python_command -m mpremote rm :stm32_flash.py

    $python_command -m mpremote cp ./utilities/firmware_updater.py :firmware_updater.py
    $python_command -m mpremote cp stm32_flash.py :stm32_flash.py

    echo "Type the name of the firmware you want to upload. (Must be in the main folder. Press Enter to skip)"
    read -r firmware_file
    if [ "$firmware_file" != "" ]; then
        $python_command -m mpremote cp $firmware_file :$firmware_file
    fi
fi

$python_command -m mpremote reset
