@echo off

set "port_string="
set "filename="

:parse_args
if "%1"=="" goto check_params
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

:check_params
if "%1"=="" (
    echo Error: Filename parameter is mandatory.
    exit /b 1
) else (
    set "filename=%1"
)

echo Uploading %filename%

python -m mpremote %port_string% fs cp %filename% :firmware.bin

set /p userInput=Do you want to flash the firmware right now? (y/N):

if /i "%userInput%"=="y" (
    python -m mpremote %port_string% run ../examples/flash_firmware.py
) else (
    echo The firmware will not be written to the device.
)

python -m mpremote %port_string% reset
exit /b 0

:display_help
echo Usage: %~nx0 [-p PORT] FILENAME
echo Options:
echo   -p PORT     Specify the device port
echo   -h          Display this help message
exit /b 0
