#!/bin/bash

connect_string=""

# Display help message
function display_help {
    echo "Usage: $0 [-p PORT] FILENAME"
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

shift $((OPTIND - 1))

if [ -z "$1" ]; then
    echo "Error: Filename parameter not provided."
    exit 1
fi

filename="$1"

if command -v python3 &>/dev/null; then
    python_command="python3"
else
    # If python3 is not available, use python
    python_command="python"
fi

# Uncomment the following line on windows machines
python_command="python"

echo "Installing flash firmware utilities..."

$python_command -m mpremote $connect_string fs rm :firmware_updater.py
$python_command -m mpremote $connect_string fs rm :stm32_flash.py

$python_command -m mpremote $connect_string fs cp ./firmware_updater.py :firmware_updater.py
$python_command -m mpremote $connect_string fs cp ./stm32_flash.py :stm32_flash.py

echo "Uploading $filename..."

$python_command -m mpremote $connect_string fs rm :firmware.bin
$python_command -m mpremote $connect_string fs cp ./$filename :firmware.bin

echo "Do want to flash the firmware right now? (y/N)"
read do_flash

if [ "$do_flash" == "y" ] || [ "$do_flash" == "Y" ]; then
    $python_command -m mpremote $connect_string run firmware_updater.py
else
    echo "Firmware was not flashed on the remote device."
fi

$python_command -m mpremote $connect_string reset
