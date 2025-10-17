# STM32F429-Pong

## Prerequisites

This project is primarily developed for the STM32F429ZE microcontroller, and STM32CubeProg / ST-Link was used for flashing and debugging. To use those tools, install the [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html) toolset and make sure to add it to your PATH environment variable.

You'll also need Zephyr RTOS installed and configured: 

On Windows: 

- Install dependencies with `winget`:

`winget install Kitware.CMake Ninja-build.Ninja oss-winget.gperf python Git.Git oss-winget.dtc wget 7zip.7zip`

> Add 7zip to PATH environment variable.

- Create a python virtual environment and install `west`:

```bash
python -m venv zephyrproject\.venv
.\zephyrproject\.venv\Scripts\activate.ps1
pip install west
```

- Get the Zephyr source code and initialize the workspace:

```bash
west init zephyrproject
cd zephyrproject
west update
```

- Export the Zephyr CMake package:

```bash
west zephyr-export
```

- Install all other python dependencies:

```bash
cd zephyr
west packages pip --install
```

- Install the Zephyr SDK:

```bash
west zephyr-sdk install
```

For Linux or MacOS the instructions are pretty much the same, but check out the official [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

## Build and flash the application:

To use west commands outside the workspace directory, you need to set the `ZEPHYR_BASE` environment variable to point to the Zephyr installation directory.

On Windows (PowerShell):

```powershell
$env:ZEPHYR_BASE="C:\path\to\zephyrproject\zephyr"
```

Then, navigate to the project directory and build the application:

```bash
west build app -b bap6000 --pristine auto
```

This will build the app for the BAP6000 board (that's included in this project).

> You might need to set the `BOARD_ROOT` CMake variable if the board is not found:

```bash
west build app -b bap6000 --pristine auto -- -DBOARD_ROOT="./"
```

To flash the application to the board, use:

```bash
west flash
```

## Alternative - this repo as the manifest repository

You can also use this repository as the manifest repository for west. To do that, fist create a new folder for your Zephyr workspace, create a new python virtual environment and install west:


```sh
mkdir zephyr-wkspace
cd zephyr-wkspace
python -m venv .venv
source .venv/bin/activate  # On Windows (PS) use: .venv\Scripts\activate.ps1
pip install west
```

Then, clone this repository and initialize the west workspace:

```sh
git clone https://github.com/freitas-renato/STM32F429-Pong.git
west init -l STM32F429-Pong
west update
```

Install the west python dependencies:

```sh
west packages pip --install
```

If you haven't installed the Zephyr SDK yet, do it now and export the Zephyr CMake package:

```sh
west zephyr-sdk install
west zephyr-export
```

Now you can build and flash the application as described above.

```sh
west build STM32F429-Pong/app -b bap6000 --pristine auto
west flash
```

