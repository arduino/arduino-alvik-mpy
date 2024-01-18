@echo off

echo Installing flash firmware utilities...

python -m mpremote rm :firmware_updater.py
python -m mpremote rm :stm32_flash.py

python -m mpremote cp ./firmware_updater.py :firmware_updater.py
python -m mpremote cp ./stm32_flash.py :stm32_flash.py

echo Uploading your firmware file. There must be a file named firmware.bin in the same folder as this sh script. Press Enter to skip

python -m mpremote rm :firmware.bin
python -m mpremote cp ./firmware.bin :firmware.bin

set /p userInput=Do you want to flash the firmware right now? (y/N):

if /i "%userInput%"=="y" (
    python -m mpremote run firmware_updater.py
) else (
    echo Firmware was not flashed on the remote device.
)

python -m mpremote reset

pause
