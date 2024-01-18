#!/bin/bash

if command -v python3 &>/dev/null; then
    python_command="python3"
else
    # If python3 is not available, use python
    python_command="python"
fi

# Uncomment the following line on windows machines
# python_command="python"

echo "Installing flash firmware utilities..."

$python_command -m mpremote rm :firmware_updater.py
$python_command -m mpremote rm :stm32_flash.py

$python_command -m mpremote cp ./firmware_updater.py :firmware_updater.py
$python_command -m mpremote cp ./stm32_flash.py :stm32_flash.py

echo "Uploading your firmware file. There must be a file named firmware.bin in the same folder as this sh script. Press Enter to skip"

$python_command -m mpremote rm :firmware.bin
$python_command -m mpremote cp ./firmware.bin :firmware.bin

echo "Do want to flash the firmware right now? (y/N)"
read do_flash

if [ "$do_flash" == "y" ] || [ "$do_flash" == "Y" ]; then
    $python_command -m mpremote run firmware_updater.py
else
    echo "Firmware was not flashed on the remote device."
fi

$python_command -m mpremote reset
