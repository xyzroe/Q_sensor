#!/bin/bash

# Set the script to exit on any errors
set -e

# Description: This script creates an OTA file from the Q_sensor.bin file.
cd ./tools

# Install the required Python packages
python3 -m pip install -q zigpy

# Read the values from const.h
MANUFACTURER=$(grep -o '#define\s\+OTA_UPGRADE_MANUFACTURER\s\+0x[0-9a-fA-F]\+' ../main/const.h | awk '{print $3}')
IMAGE_TYPE=$(grep -o '#define\s\+OTA_UPGRADE_IMAGE_TYPE\s\+0x[0-9a-fA-F]\+' ../main/const.h | awk '{print $3}')
FILE_VERSION=$(grep -o '#define\s\+OTA_FW_VERSION\s\+0x[0-9a-fA-F]\+' ../main/const.h | awk '{print $3}')

# Check if the Q_sensor.bin file exists
if [ ! -f "../build/Q_sensor.bin" ]; then
    echo "Q_sensor.bin file not found!"
    exit 1
fi

# Print the values
echo "M: $MANUFACTURER | IT: $IMAGE_TYPE | FV: $FILE_VERSION";

# Create the output folder if it doesn't exist yet
mkdir -p ../output

# Create the OTA file
python3 create-ota.py -m "$MANUFACTURER" -i "$IMAGE_TYPE" -v "$FILE_VERSION" ../build/Q_sensor.bin ../output/Q_sensor.ota

# Copy the Q_sensor.bin file to the output folder
cp ../build/Q_sensor.bin ../output/Q_sensor.bin

echo "OTA file created successfully! Version: $FILE_VERSION"