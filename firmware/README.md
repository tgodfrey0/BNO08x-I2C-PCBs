# Firmware

The firmware is written in C for the Raspberry Pi Pico microcontroller.

The `adafruit-testing` folder contains very crude code used for I2C communication with an [Adafruit BNO085 breakout board](https://www.adafruit.com/product/4754#technical-details). This code is not inteded to be used for project with the breakout board, it was used to check the minimum number of pins needed to communicate with the BNO085.

All code can be compiled using CMake and a generator of your choice.
