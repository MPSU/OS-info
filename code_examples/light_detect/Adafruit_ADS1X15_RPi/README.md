# Adafruit_ADS1X15_RPi

Adafruit's ADS1X15 C++ library is compatible with various Arduinos. This fork modifies the library to be compatible with Raspberry Pi by using the wiringPi library to do the i2c communications instead of Arduino's wiring.h library. Note that there already exists an ADS1X15 Python library for RPi written by Adafruit: https://github.com/adafruit/Adafruit_Python_ADS1x15. In my case I needed to integrate the library with other C++ code so I made this fork.

[Tested on Raspberry Pi Zero W running Raspbian Jessie]

## Getting started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites
Install wiringPi:
  http://wiringpi.com/download-and-install/

Enable RPi's SPI and I2C interfaces:
```
sudo raspi-config
# then go to "Interfacing Options"
# and enable SPI and I2C
```

### Installing
Clone this repository:
```
cd ~
git clone https://github.com/hallgrimur1471/Adafruit_ADS1X15_RPi.git
```
Now compile, say, the differential_rpi.c example:
```
cd ~/Adafruit_ADS1X15_RPi
g++ -W -lwiringPi -o differential_rpi examples/differential_rpi.c Adafruit_ADS1015.cpp
```

And run it:
```
./differential_rpi
```

The program should now output ADS1X15's measurements.

Note that the differential_rpi.c example assumes you have the ADS1X15 circuit wired for differential measurements (see https://learn.adafruit.com/adafruit-4-channel-adc-breakouts/signal-connections).


### Original README.md from fork's origin:

Adafruit_ADS1015
================

Driver for TI's ADS1015: 12-bit Differential or Single-Ended ADC with PGA and Comparator
<!-- START COMPATIBILITY TABLE -->

## Compatibility

MCU                | Tested Works | Doesn't Work | Not Tested  | Notes
------------------ | :----------: | :----------: | :---------: | -----
Atmega328 @ 16MHz  |      X       |             |            | 
Atmega328 @ 12MHz  |      X       |             |            | 
Atmega32u4 @ 16MHz |      X       |             |            | Use SDA/SCL on pins D2 &amp; D3
Atmega32u4 @ 8MHz  |      X       |             |            | Use SDA/SCL on pins D2 &amp; D3
ESP8266            |      X       |             |            | SDA/SCL default to pins 4 &amp; 5 but any two pins can be assigned as SDA/SCL using Wire.begin(SDA,SCL)
Atmega2560 @ 16MHz |      X       |             |            | Use SDA/SCL on pins 20 &amp; 21
ATSAM3X8E          |      X       |             |            | Use SDA/SCL on pins 20 &amp; 21
ATSAM21D           |      X       |             |            | 
ATtiny85 @ 16MHz   |      X       |             |            | Use 0 for SDA, 2 for SCL
ATtiny85 @ 8MHz    |      X       |             |            | Use 0 for SDA, 2 for SCL
Intel Curie @ 32MHz |             |             |     X       | 
STM32F2            |             |             |     X       | 

  * ATmega328 @ 16MHz : Arduino UNO, Adafruit Pro Trinket 5V, Adafruit Metro 328, Adafruit Metro Mini
  * ATmega328 @ 12MHz : Adafruit Pro Trinket 3V
  * ATmega32u4 @ 16MHz : Arduino Leonardo, Arduino Micro, Arduino Yun, Teensy 2.0
  * ATmega32u4 @ 8MHz : Adafruit Flora, Bluefruit Micro
  * ESP8266 : Adafruit Huzzah
  * ATmega2560 @ 16MHz : Arduino Mega
  * ATSAM3X8E : Arduino Due
  * ATSAM21D : Arduino Zero, M0 Pro
  * ATtiny85 @ 16MHz : Adafruit Trinket 5V
  * ATtiny85 @ 8MHz : Adafruit Gemma, Arduino Gemma, Adafruit Trinket 3V

<!-- END COMPATIBILITY TABLE -->
