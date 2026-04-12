# FIRM
Filtered Inertial Rotation Module

FIRM is a project by the NCSU High Powered Rocketry Club to develop a flight computer for high powered rockets. The flight computer is designed to provide accurate attitude and position data during flight using an array of sensors and advanced filtering algorithms.

It is also designed to be modular and easily adaptable to different rocket configurations and mission requirements. The project will also have a custom PCB design to integrate all the components into a compact and lightweight package suitable for high-speed flight.

This repository contains both the embedded code (written in C) that is compiled directly on the STM32 microcontroller, as well as the client code (written in Rust) that will act as the interface between the user-facing API and the hardware.

## Hardware Components

The components used in this project include:

| Part Number    | Description       | Datasheet Link                                                                                      |
| -------------- | ----------------- | ------------------------------------------------------------------------------------------------- |
| ICM-45686      | 6 Axis IMU        | [Datasheet](https://www.mouser.com/catalog/specsheets/TDK_DS_000577_ICM_45686.pdf?srsltid=AfmBOooJ55_Jsy4PB-5CLV5vQtmUmcbglSfWs9S-cLOBSOXLc19UPaWf) |
| STM32F405RGT6  | Microcontroller   | [Datasheet](https://www.st.com/resource/en/datasheet/dm00037051.pdf)                              |
| MMC5983MA      | Magnetometer      | [Datasheet](https://media.digikey.com/pdf/Data%20Sheets/MEMSIC%20PDFs/MMC5983MA_RevA_4-3-19.pdf)  |
| BMP581         | Pressure Sensor   | [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf) |


## Project Setup

To set up the project for either embedded development in C or client development in Rust, you will need [uv](https://docs.astral.sh/uv/) installed, and clone the repository:
```bash
git clone https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM.git
```

