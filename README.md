# Description

This repository contains the **MAX-M10x** GPS module driver (based on the UART interface).

# Dependencies

The driver relies on:

* An external `types.h` header file defining the **standard C types** of the targeted MCU.
* The **embedded utility functions** defined in the [embedded-utils](https://github.com/Ludovic-Lesur/embedded-utils) repository.

Here is the versions compatibility table:

| **maxm10x-driver** | **embedded-utils** |
|:---:|:---:|
| [sw1.1](https://github.com/Ludovic-Lesur/maxm10x-driver/releases/tag/sw1.1) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.0](https://github.com/Ludovic-Lesur/maxm10x-driver/releases/tag/sw1.0) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |

# Compilation flags

| **Flag name** | **Value** | **Description** |
|:---:|:---:|:---:|
| `MAXM10X_DRIVER_DISABLE_FLAGS_FILE` | `defined` / `undefined` | Disable the `maxm10x_driver_flags.h` header file inclusion when compilation flags are given in the project settings or by command line. |
| `MAXM10X_DRIVER_DISABLE` | `defined` / `undefined` | Disable the MAX-M10x driver. |
| `MAXM10X_DRIVER_GPIO_ERROR_BASE_LAST` | `<value>` | Last error base of the low level GPIO driver. |
| `MAXM10X_DRIVER_UART_ERROR_BASE_LAST` | `<value>` | Last error base of the low level UART driver. |
| `MAXM10X_DRIVER_DELAY_ERROR_BASE_LAST` | `<value>` | Last error base of the low level delay driver. |
| `MAXM10X_DRIVER_GPS_DATA_TIME` | `defined` / `undefined` | Enable or disable the time acquisition feature. |
| `MAXM10X_DRIVER_GPS_DATA_POSITION` | `defined` / `undefined` | Enable or disable the position acquisition feature. |
| `MAXM10X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE` | `0` / `1` / `2` | Altitude stability filter mode: `0` = disabled `1` = fixed `2` = dynamic.|
| `MAXM10X_DRIVER_ALTITUDE_STABILITY_THRESHOLD` | `<value>` | Altitude stability filter threshold (used when mode is `1`).
| `MAXM10X_DRIVER_VBCKP_CONTROL` | `defined` / `undefined` | Enable or disable the backup voltage pin control. |
| `MAXM10X_DRIVER_TIMEPULSE` | `defined` / `undefined` | Enable or disable the timepulse signal control. |

# Build

A static library can be compiled by command line with `cmake`.

```bash
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="<toolchain_file_path>" \
      -DTOOLCHAIN_PATH="<arm-none-eabi-gcc_path>" \
      -DTYPES_PATH="<types_file_path>" \
      -DEMBEDDED_UTILS_PATH="<embedded-utils_path>" \
      -DMAXM10X_DRIVER_GPIO_ERROR_BASE_LAST=0 \
      -DMAXM10X_DRIVER_UART_ERROR_BASE_LAST=0 \
      -DMAXM10X_DRIVER_DELAY_ERROR_BASE_LAST=0 \
      -DMAXM10X_DRIVER_GPS_DATA_TIME=ON \
      -DMAXM10X_DRIVER_GPS_DATA_POSITION=ON \
      -DMAXM10X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE=2 \
      -DMAXM10X_DRIVER_ALTITUDE_STABILITY_THRESHOLD=5 \
      -DMAXM10X_DRIVER_VBCKP_CONTROL=ON \
      -DMAXM10X_DRIVER_TIMEPULSE=ON \
      -G "Unix Makefiles" ..
make all
```
