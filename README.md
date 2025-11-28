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

4. Restart VS Code.

5. Open the STM32 folder of the repo in VS Code and use the extension to import the folder with the
"Import CMake project" button.

6. Configure your workspace by accepting the default settings from the pop-up messages in VS Code.

7. Click the "Build" button on the bottom status bar to build the project.

8. Run `uv sync`.

9. Run `uv run pre-commit install` to set up the git hook for automatic code formatting and linting, using `clang-format` & `clang-tidy`.


## Building the project

In VS Code, press `Ctrl+Shift+B` to bring up the build menu.
Then click "CMake: build". You should see the build output in the terminal.

## Flashing the firmware

To flash the firmware onto the STM32 microcontroller, follow these steps:

1. Connect the ST-Link Debugger to your computer, and jump the GND, SWDIO, and SWCLK pins from the ST-Link onto FIRM (see the back of the PCB for pin locations).

2. Power FIRM via a USB-C cable or another power source.

3. In VSCode, hit `Ctrl+Shift+P` to open the command palette. Then search for `Run Task` and select it. Then select the option which says to flash via SWD. 


### Running the debugger

Running the debugger is very similar to flashing the firmware.

Make sure you still have the ST-Link hooked onto FIRM, and then go to the Run and Debug tab on the left side of VS Code (or hit `Ctrl+Shift+D`).

Then simply click the green play button at the top of the sidebar to start a debugging session.


## Third party licenses

Contains FATFS changes from https://github.com/MathewMorrow/STM32-SD-Logging-DMA (MIT Licensed)
