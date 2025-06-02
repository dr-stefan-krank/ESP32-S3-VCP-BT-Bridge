# ESP32-S3 Bluetooth Serial Bridge

This project implements a Bluetooth Low Energy (BLE) Serial Port Profile (SPP) bridge for the ESP32-S3. It allows you to connect to the ESP32-S3 over BLE and exchange serial data with a USB-Serial device attached to the ESP32-S3.
Its main use is for configuring devices like network switches without the need of an USB cable.

## Features
- BLE SPP server: Connect from a BLE client (e.g., PC, phone, or another ESP32) and exchange serial data.
- USB-Serial bridge: Data from BLE is forwarded to the USB-Serial device and vice versa.
- Visual feedback via WS2812 RGB LED for connection/activity states.

## Hardware Requirements
- ESP32-S3 development board (with PSRAM recommended)
- USB-Serial device (e.g., FTDI, CP210x, CH34x, etc.)
- WS2812 RGB LED connected to the configured GPIO (GPIO 48, integrated on the ESP32-S3-WROOM-1 devboard)
- On the back of the board should be a solder bridge that needs to be bridged to enable USB-OTG.

## Software Requirements
- ESP-IDF v5.0 or newer (tested with v5.3.2)
- Python 3 (for the `ble-serial` client)

## Building the Firmware

### Using ESP-IDF
1. Install ESP-IDF (see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
Import the sdkconfig or modify your own.

## Python BLE Serial Client
A reference client is provided in `ble/bin/ble-serial` (see the `ble-serial` script in the `ble` folder). This script creates a virtual serial port on your PC that connects to the ESP32-S3 over BLE and starts a console connection.

### Usage Example (Linux/macOS)
```sh
cd ble
./bin/ble-serial -d <BLE_MAC_ADDRESS> -r 0000abf2-0000-1000-8000-00805f9b34fb -w 0000abf1-0000-1000-8000-00805f9b34fb -m 517 --write-with-response
```
- `-d <BLE_MAC_ADDRESS>`: The MAC address of your ESP32-S3 (findable via BLE scan)

ble-serial will automatically start a raw terminal to the USB-COM Port.

#### Windows Usage (untested)
On Windows, you can try using the same script with a Windows COM port path, e.g.:
```sh
python ble-serial -d <BLE_MAC_ADDRESS> -p COM5 -r 0000abf2-0000-1000-8000-00805f9b34fb -w 0000abf1-0000-1000-8000-00805f9b34fb -m 517 --write-with-response
```
- Replace `COM5` with the desired COM port name. You may need to run the script in a Python environment with the required dependencies installed.
- Note: Windows support is untested. If you encounter issues, please report them.

#### Exiting the Client
- Type `~.` (tilde followed by a dot) at the beginning of a line to exit the client cleanly.

## LED Status and Color Codes
The WS2812 RGB LED provides visual feedback for the system state:

- **Yellow:** Both RX and TX activity (BLE or USB-Serial connected, data in both directions)
- **Red:** RX activity only
- **Green:** TX activity only
- **Blue:** USB-Serial connected, no BLE
- **Purple:** BLE connected, no data activity
- **Off:** No connection and no USB-Serial connected

The LED will update in real time to reflect the current connection and data transfer state.

## Troubleshooting
- For USB-Serial issues, check your cable and that the device is supported. If needed add the VID and PID to the drivers sources in the components folder.

