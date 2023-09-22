# STM32F4xx grblHAL driver

A grblHAL driver for the STM32F401xC, STM32F407xx, STM32F411xE and STM32F446xx ARM processors.

Loosely based on code from robomechs [6-AXIS-USBCNC-GRBL](https://github.com/robomechs/6-AXIS-USBCNC-GRBL) port, updated for [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.htm) and the latest STM HAL drivers where appropriate.

# Quickstart

This driver can be built with the [Web Builder](http://svn.io-engineering.com:8080/?driver=STM32F4xx).
Alternatively it can be built with the STM32CubeIDE or [PlatformIO](https://platformio.org).  
For additional information on howto import the project, configure the driver and compile the firmware, see the Wiki-page [compiling grblHAL](https://github.com/grblHAL/core/wiki/Compiling-GrblHAL).

Install [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation.html) and then:

```shell
# Clone this repository
git clone https://github.com/grblHAL/STM32F4xx.git
cd STM32F4

# List available environments:
pio run --list-targets

# Pick an environment name (left column) matching your board variant and kick off the build with:
pio run --environment nucleo_f411re_protoneer

# To build firmware for all supported board variants, simply skip the `--environment` option:
pio run

# For e.g. Nucleo boards you can upload the firmware (via USB) using the built-in ST-Link programmer:
pio run -e nucleo_f411re_protoneer --target upload

# For help with PlatformIO, try using `-h`:
pio run -h
pio -h
```

The built firmware is stored inside directories named `.pio/build/<env name>/`, as `firmware.elf` (executable) or `firmware.bin` (binary image).

If the board exposes a USB mass storage device or has a microSD card, simply copy the `firmware.bin` to the root of this filesystem and reset the board.

If you want to customize the build, take a look at [platformio.ini](platformio.ini) where you can uncomment/customize additional `build_flags` and `lib_deps` in the `[common]` or `[env:some_board_variant]` sections.

Also see [Inc/my_machine.h](Inc/my_machine.h) but beware that settings in this file are ignored by PlatformIO (via the `-D OVERRIDE_MY_MACHINE=` build flag).  If they're not already present in the `platformio.ini` file, you will need to add them to the `build_flags` option in the appropriate section.


NOTE: As of Sep, 2021, the PlatformIO environments for the BigTreeTech SKR boards remain untested and might not work.  Feel free to drop a note of your success or failure with these boards on https://github.com/grblHAL/STM32F4xx/issues.


# Changelog

2021-19-09: Added build option to build a [BTT SKR 2](https://www.bigtree-tech.com/products/bigtreetech-skr-2.html) bootloader compatible binary.

This can be built by selecting _Release F407 8MHz 32K Bootloader_ from the build tool dropdown.  

_NOTE:_ A build configuration for a debug version has not been added as a programmer/debugger is required and flashing a "normal" debug build is usually appropriate when debugging.
This overwrites the bootloader that later has to be restored if a bootloader version is to be flashed again.
Alternatively a build configuration has to be added or modified to set the vector table offset by adding the symbol `VECT_TAB_OFFSET` with value `0x8008000` and configuring it to use the correct loader script.

Bootloader binaries for the SKR2 board can be found [here](https://github.com/GadgetAngel/BTT_SKR_13_14_14T_SD-DFU-Bootloader/tree/main/bootloader_bin/backed_up_original_bootloaders/SKR%20V2.0%20(SKR%202)).
Be aware that the [Bootloader_and_Firmware](https://github.com/GadgetAngel/BTT_SKR_13_14_14T_SD-DFU-Bootloader/tree/main/bootloader_bin/backed_up_original_bootloaders/SKR%20V2.0%20(SKR%202)/Bootloader_and_Firmware) folder contains Marlin firmware, not grblHAL.

2021-08-10: Added support for [BTT SKR 2](https://www.bigtree-tech.com/products/bigtreetech-skr-2.html), this is based on a STM32F407 processor with a 8MHz crystal.

2021-05-28: Added support for [BTT SKR PRO](https://www.bigtree-tech.com/products/bigtreetech-skr-pro-v1-2.html), this is based on a STM32F407 processor with a 25MHz crystal.


# Building grblHAL with Eclipse

Available driver options can be found [here](Inc/my_machine.h).

Select the processor to build for from the build tool dropdown to build. The `.bin` created file can be found in the folder with the same name as the menu name when the build is completed.  
![Config](media/STM32F4xx_config.png)

| Build configuration              | Targets                          | Linker script                |
|----------------------------------|----------------------------------|------------------------------|
| Release F401 Blackpill           | F401 Blackpill                   | STM32F401CCUX_FLASH.ld       | 
| Release F407 25MHz               | BT SKR PRO                       | STM32F407VGTX_FLASH.ld       |
| Release F407 8MHz                | BTT SKR 2.0, STM32F407 Discovery | STM32F407VGTX_FLASH.ld       |
| Release F407 8MHz 32K Bootloader | BTT SKR 2.0                      | STM32F407VGTX_BL32K_FLASH.ld |
| Release F411 Blackpill           | F411 Blackpill                   | STM32F411CEUX_FLASH.ld       |
| Release F411 Nucleo64            | NUCLEO-F446RE                    | STM32F411CEUX_FLASH.ld       |
| Release F446 8 MHz               | Generic                          | STM32F446RETX_FLASH.ld       |
| Release F446 Nucleo64            | NUCLEO-F446RE                    | STM32F446RETX_FLASH.ld       |

If the oscillator frequency is different from the default 25 MHz then add the symbol `HSE_VALUE` and set the value to the frequency in Hz. E.g. `8000000` for 8 Mhz.

A method for flashing the Nucleo F411 and Nucleo F446 is to drop the `.bin` file on the NODE_F4xxRE flash drive. Note that the file can be dragged from the IDE _Project Explorer_.

_NOTE:_ Internal flash page for parameters is not at the end of the flash memory due to size restrictions. This means each firmware upgrade will erase any saved parameters. 

---

If compiling for debugging, edit symbols in project properties _C\/C++ General > Paths and Symbols > Symbols_ to match your processor/board.

Remove symbols starting with `STM32F4`, starting with `NUCLEO_F4` and `HSE_VALUE`. Then add symbols:

#### STM32F401

* Add `STM32F401xC`.

#### STM32F407

* Add `STM32F407xx`.

#### STM32F411

* Add `STM32F411xE`.

* If compiling for the [Nucleo F411RE development board](https://www.st.com/en/evaluation-tools/nucleo-f411re.html) add `NUCLEO_F411`.

#### STM32F446

* Add  `STM32F446xx`.

* If compiling for the [Nucleo F446RE development board](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) add `NUCLEO_F446`.


# See also

CNC breakout boards:

[Minimal breakout for Blackpill](https://github.com/avizienis/Minimal-Black-Pill--STM32F4xx-BOB-for-grblHAL) by avizienis.

[CNC breakout for Nucleo-64](https://github.com/terjeio/CNC_Breakout_Nucleo64) by Terje Io.

---
2023-09-20
