#!/bin/bash

connect_string=""

# Display help message
function display_help {
    echo "Usage: $0 [-p PORT]"
    echo "Options:"
    echo "  -p PORT     Specify the device port"
    echo "  -h          Display this help message"
}

# Parse command-line options
while getopts ":p:h" opt; do
    case $opt in
        p)
            # If -p is provided, set the port string
            connect_string="connect $OPTARG"
            ;;
        h)
            display_help
            exit 0
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
    esac
done

if command -v python3 &>/dev/null; then
    python_command="python3"
else
    # If python3 is not available, use python
    python_command="python"
fi

# Uncomment the following line on windows machines
# python_command="python"

$python_command -m mpremote $connect_string fs mkdir lib/arduino_alvik
$python_command -m mpremote $connect_string fs cp arduino_alvik/__init__.py :lib/arduino_alvik/__init__.py
$python_command -m mpremote $connect_string fs cp arduino_alvik/arduino_alvik.py :lib/arduino_alvik/arduino_alvik.py
$python_command -m mpremote $connect_string fs cp arduino_alvik/constants.py :lib/arduino_alvik/constants.py
$python_command -m mpremote $connect_string fs cp arduino_alvik/conversions.py :lib/arduino_alvik/conversions.py
$python_command -m mpremote $connect_string fs cp arduino_alvik/pinout_definitions.py :lib/arduino_alvik/pinout_definitions.py
$python_command -m mpremote $connect_string fs cp arduino_alvik/robot_definitions.py :lib/arduino_alvik/robot_definitions.py
$python_command -m mpremote $connect_string fs cp arduino_alvik/uart.py :lib/arduino_alvik/uart.py

echo "Installing dependencies"
$python_command -m mpremote $connect_string mip install github:arduino/ucPack-mpy

$python_command -m mpremote $connect_string reset
