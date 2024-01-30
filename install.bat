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
python -m mpremote %port_string% fs rm :arduino_alvik.py
python -m mpremote %port_string% fs rm :constants.py
python -m mpremote %port_string% fs rm :pinout_definitions.py
python -m mpremote %port_string% fs rm :robot_definitions.py
python -m mpremote %port_string% fs rm :uart.py

python -m mpremote %port_string% fs cp arduino_alvik.py :arduino_alvik.py
python -m mpremote %port_string% fs cp constants.py :constants.py
python -m mpremote %port_string% fs cp pinout_definitions.py :pinout_definitions.py
python -m mpremote %port_string% fs cp robot_definitions.py :robot_definitions.py
python -m mpremote %port_string% fs cp uart.py :uart.py

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