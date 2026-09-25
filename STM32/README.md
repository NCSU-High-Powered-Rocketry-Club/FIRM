# Embedded FIRM
This contains all of the STM32-based embedded code that runs on FIRM itself.

## Environment Setup

1. Download the STM32CubeCLT from the [STMicroelectronics website](https://www.st.com/en/development-tools/stm32cubeclt.html) and install it on your machine.

2. Install the [STM32 VS Code Extension](https://marketplace.visualstudio.com/items?itemName=stmicroelectronics.stm32-vscode-extension).

3. Restart VS Code.

4. Open the STM32 folder of the repo in VS Code and use the extension to import the folder with the
"Import CMake project" button.

5. Configure your workspace by accepting the default settings from the pop-up messages in VS Code.

6. Click the "Build" button on the bottom status bar to build the project.

7. From the repository root, run `just sync`.

8. Run `uv run pre-commit install` to set up the git hook for automatic code formatting and linting, using `clang-format` & `clang-tidy`.

9. Run `cmake --preset firmware-debug` from the repository root once. The `clang-tidy` hook reads
   `build/firmware-debug/compile_commands.json`, which the VS Code extension's build (in
   `STM32/build/`) does not create.

## Unit tests

Firmware unit tests live in `tests/`. They are built for your computer with the repository's `host` CMake
preset and use [utest.h](../third_party/utest). From the repository root run `just test-firmware`
(or `ctest --preset host -L firmware`). Each test links only the production files listed for it in
`tests/CMakeLists.txt`; anything else it needs is faked in the test file or in `tests/support/`.


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

## Tracing

Tracing can be used to show how long each thread is running for.

To get a trace, debug the board with a ST-LINK. Pause execution, open up the `Debug Console`, and run

```
>dump binary value trace.bin trace_data
```

The `>` is required for the command to be interpreted as a GDB command. This will save the trace to `STM32/trace.bin`.

To format the trace, from the repository root run:

```bash
uv run firm-trace -i STM32/trace.bin -o trace.json
```


This will produce a json trace that can be visualized in [spall](https://gravitymoth.com/spall/spall.html) or [perfetto](https://ui.perfetto.dev)

## Third party licenses

Contains FATFS changes from https://github.com/MathewMorrow/STM32-SD-Logging-DMA (MIT Licensed)
