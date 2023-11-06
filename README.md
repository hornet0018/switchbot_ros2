# Bluetooth Scanner Node for ROS2

## Overview

This Python script is designed to operate within the ROS2 (Robot Operating System Version 2) framework. It scans for sensor data from a specified Bluetooth device, encodes this data in JSON format, and publishes it to a ROS2 topic. The script specifically reads temperature, humidity, and battery level from a device with the predefined MAC address and publishes this information to the ROS2 topic `switchbot/data`.

## Prerequisites

Before running this node, ensure that you have:

- Python 3.5 or newer installed.
- An ROS2 environment set up.
- The `std_msgs` package available in your ROS2 installation.
- The `bluepy` library installed for Bluetooth communication.

## Installation

Ensure you have the correct ROS2 distribution installed. For setup and installation instructions, refer to the [ROS2 documentation](https://docs.ros.org/en/foxy/Installation.html).

Install the `bluepy` library using pip:

## Usage

To use this script, follow these steps:
1. Include this script in a package in your ROS2 workspace.
2. Make the script executable: `chmod +x bluetooth_scanner_node.py`
3. Build your ROS2 workspace: `colcon build && . install/setup.bash`
4. Run the node: `ros2 run [your_package_name] bluetooth_scanner_node`

Remember to replace `[your_package_name]` with the actual name of your ROS2 package.

## Features

- **MAC Address Filtering**: Targets only the device with the specified MAC address (`d6:03:21:1e:72:d1`).
- **Sensor Data Retrieval**: Extracts temperature, humidity, and battery level data.
- **Data Encoding and Publishing**: Encodes data into JSON and publishes it to `switchbot/data` on ROS2.
- **Periodic Scanning**: Scans for the Bluetooth device every second.

## Node Details

This node includes:
- `ScanDelegate`: Inherits from `bluepy.btle.DefaultDelegate`. Handles discovery events for Bluetooth devices.
- `BluetoothScannerNode`: A ROS2 node that initializes the scanner and handles data publishing.

## Notes

- This script is optimized for specific Bluetooth device types and may not work with others without modification.
- The `bluepy` library is only supported on Linux.
- Bluetooth scanning might require root permissions, depending on your system setup.

## License

This script is released under the MIT License.
