***This is a work in progress and has huge problems!***

The objective is to create bluetooth to serial bridge on the ESP32-S3 that uses USB-OTG to connect to USB Serial Adapters.
Mainly to connect to devices like network switches and firewalls who don't provide a real RS232 port anymore.

The project is based on the examples of the ESP-IDF 5.3.2.

Current state: Slow transfer works, but as soon as more data is transferred the buffers overflow and the ESP crashes.

To connect from a PC, you can use ble-serial:

https://github.com/Jakeler/ble-serial

Example:

ble-serial -d 34:85:18:XX:XX:XX -r 0000abf2-0000-1000-8000-00805f9b34fb -w 0000abf1-0000-1000-8000-00805f9b34fb
