# FIRM
Filtered Inertial Rotation Module

FIRM is a project by the NCSU High Powered Rocketry Club to develop a flight computer for high powered rockets. The flight computer is designed to provide accurate attitude and position data during flight using an array of sensors and advanced filtering algorithms.

It is also designed to be modular and easily adaptable to different rocket configurations and mission requirements. The project will also have a custom PCB design to integrate all the components into a compact and lightweight package suitable for high-speed flight.

## Hardware Components

The components used in this project include:

| Part Number    | Description       | Datasheet Link                                                                                      |
| -------------- | ----------------- | ------------------------------------------------------------------------------------------------- |
| ICM-45686      | 6 Axis IMU        | [Datasheet](https://www.mouser.com/catalog/specsheets/TDK_DS_000577_ICM_45686.pdf?srsltid=AfmBOooJ55_Jsy4PB-5CLV5vQtmUmcbglSfWs9S-cLOBSOXLc19UPaWf) |
| STM32F405RGT6  | Microcontroller   | [Datasheet](https://www.st.com/resource/en/datasheet/dm00037051.pdf)                              |
| MMC5983MA      | Magnetometer      | [Datasheet](https://media.digikey.com/pdf/Data%20Sheets/MEMSIC%20PDFs/MMC5983MA_RevA_4-3-19.pdf)  |
| BMP581         | Pressure Sensor   | [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf) |


## Project Setup

To set up the project, follow these steps (assuming you have Git/Github Desktop, and [uv](https://docs.astral.sh/uv/) installed):

1. Clone the repository:
```bash
git clone https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM.git
```

2. Download the STM32CubeCLT from the [STMicroelectronics website](https://www.st.com/en/development-tools/stm32cubeclt.html) and install it on your machine.

3. Install the [STM32 VS Code Extension](https://marketplace.visualstudio.com/items?itemName=stmicroelectronics.stm32-vscode-extension).

4. Open the STM32 folder of the repo in VS Code and use the extension to import the folder with the
"Import CMake project" button.

5. Configure your workspace by accepting the default settings from the pop-up messages in VS Code.

6. Click the "Build" button on the bottom status bar to build the project.

7. Run `uv sync`.

8. Run `uv run pre-commit install` to set up the git hook for automatic code formatting, using `clang-format`.


## Building the project


If you are using STM32CubeIDE, you can build the project by clicking on the hammer icon or by going to `Project` > `Build All`.


## Third party licenses

Contains FATFS changes from https://github.com/MathewMorrow/STM32-SD-Logging-DMA (MIT Licensed)