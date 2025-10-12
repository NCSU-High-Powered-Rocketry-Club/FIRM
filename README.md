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

2. Download the STM32CubeIDE from the [STMicroelectronics website](https://www.st.com/en/development-tools/stm32cubeide.html) and install it on your machine.

3. Open STM32CubeIDE and import the project:
   - Go to `File` > `Import`.
   - Select `General` > `Existing Projects into Workspace` and click `Next`.
   - Browse to the cloned repository location and select it.
   - Click `Finish`.

4. You'll find our source files in the `Core/Src` directory and header files in the `Core/Inc` directory.

5. Run `uv sync`.

6. Run `pre-commit install` to set up the git hook for automatic code formatting and linting, using `clang-format` and `clang-tidy`. The linting configuration can be found in `.clang-tidy` at the repository root.


## Code Quality Tools

This project uses automated code quality tools:
- **clang-format**: Automatically formats C code according to the style defined in `.clang-format`
- **clang-tidy**: Performs static analysis to catch bugs, performance issues, and style violations. Configuration in `.clang-tidy` includes checks for:
  - Bug-prone code patterns (bugprone-*)
  - Static analysis (clang-analyzer-*)
  - Performance issues (performance-*)
  - Readability improvements (readability-*)
  - Portability concerns (portability-*)

Both tools run automatically on commit via pre-commit hooks. To run them manually:
```bash
uv run pre-commit run --all-files
```


## Building the project


If you are using STM32CubeIDE, you can build the project by clicking on the hammer icon or by going to `Project` > `Build All`.


## Third party licenses

Contains FATFS changes from https://github.com/MathewMorrow/STM32-SD-Logging-DMA (MIT Licensed)