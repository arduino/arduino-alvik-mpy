@echo off

set "port_string="

:parse_args
if "%1"=="" goto install
if /i "%1"=="-p" (
    set "port_string=connect %2"
    shift
    shift
    goto parse_args
)
if /i "%1"=="-h" (
    call :display_help
    exit /b 0
)

:install

python -m mpremote %port_string% fs mkdir lib
python -m mpremote %port_string% fs mkdir lib/arduino_alvik
python -m mpremote %port_string% fs cp arduino_alvik/__init__.py :lib/arduino_alvik/__init__.py
python -m mpremote %port_string% fs cp arduino_alvik/arduino_alvik.py :lib/arduino_alvik/arduino_alvik.py
python -m mpremote %port_string% fs cp arduino_alvik/constants.py :lib/arduino_alvik/constants.py
python -m mpremote %port_string% fs cp arduino_alvik/conversions.py :lib/arduino_alvik/conversions.py
python -m mpremote %port_string% fs cp arduino_alvik/pinout_definitions.py :lib/arduino_alvik/pinout_definitions.py
python -m mpremote %port_string% fs cp arduino_alvik/robot_definitions.py :lib/arduino_alvik/robot_definitions.py
python -m mpremote %port_string% fs cp arduino_alvik/uart.py :lib/arduino_alvik/uart.py

echo Installing dependencies
python -m mpremote %port_string% mip install github:arduino/ucPack-mpy

python -m mpremote %port_string% reset
exit /b 0

:display_help
echo Usage: %~nx0 [-p PORT]
echo Options:
echo   -p PORT     Specify the device port
echo   -h          Display this help message
exit /b 0