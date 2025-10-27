# FIRM PCB
This README aims to explain the hardware of FIRM in-depth and explain the design choices that went into designing our board.

## Overview

The purpose of creating our own PCB for FIRM was for a few main reasons. A big reason is customizabiltiy. When creating a flight controller from breakout boards, we are largely limited by the breakout boards that are supplied by major hobbyist companies like Adafruit and Sparkfun. A second reason is size. It's no surprise that when you only buy the sensor itself and not the entire breakout board, you can save a lot of wasted space that otherwise would be taken up by standard 0.1 inch pitch pinouts. The last major reason is cost. Hobbyist companies put huge mark-ups on their breakout boards, so we save a lot of money by designing and ordering the boards ourselves.

The schematic design is split up into 4 major components:
- Microcontroller: The heart of the board, supplies all of our pinouts and programmed to interface with the sensors
- Sensors: The hardware units that read pressure, temperature, acceleration, angular rate, magnetic field, etc.
- Logging/Storage: Encompasses our flash chip and the microSD card socket
- Power: The power input for the board through either input pins or USB, as well as the voltage regulator

## Microcontroller

Our MCU is the STM32F405RGT6. The name of this chip tells us a lot of information about it:

- "STM32": This is the 32-bit MCU family. The other main example is STM8 for 8-bit MCUs.
- "F": The type of the MCU, F is for foundational, sometimes high-performance though. Thier naming scheme isn't super consistant. Other common ones are G for mainstream, L for low-power, H for high-performance, and W for wireless.
- "4": The core, specifies the ARM Cortex chip used. STM32F4 MCU's use an ARM Cortex M4.
- "05": One of the models in the STM32F4 product lineup
- "R": Number of pinouts on the MCU. R either means 64 or 66. In our case, we have 64 pinouts.
- "G": Flash memory size, we have 1024 KBytes.
- "T": Package type, T is for LQFP packages.
- "6": Temperature rating, 6 denotes -40\deg C to 105\deg C.
[Heres a guide with more information about the naming scheme](https://www.digikey.com/en/maker/tutorials/2020/understanding-stm32-naming-conventions)

The main reason we chose this MCU in particular was due to its popularity, namely in drone flight controllers. We decided that having an MCU that was well-documented was going to be more important than an MCU that had the best specs. While specs do matter, we started this project with no STM32 experience and we knew development would go faster when there's already someone online that has encountered (and fixed) all the issues we run into.

The pinouts are also important for the MCU. We wanted to communicate through our sensors through SPI, which is much faster than the alternative communication protocol, I2C. Another supported protocol is UART, which will be used (in future FIRM iterations) for GPS, and also can be used for communicating with a main device such as an Arduino so that FIRM can transmit flight data real-time to it. It is important to note that many of the pins have alternate uses and can be configred via STM32CubeMX to do different functions.

## Sensors



