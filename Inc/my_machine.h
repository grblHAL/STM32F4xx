/*
  my_machine.h - configuration for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used.
//#define BOARD_CNC3040
//#define BOARD_PROTONEER_3XX       // For use with Nucleo-F411RE and F446RE boards.
//#define BOARD_GENERIC_UNO         // For use with Nucleo-F411RE and F446RE boards.
//#define BOARD_CNC_BOOSTERPACK
//#define BOARD_MORPHO_CNC          // For use with Nucleo-F411RE and F446RE boards. Work in progress.
//#define BOARD_MORPHO_DAC_CNC      // For use with F446RE boards and a 32-bit I2S stereo DAC for X and Y output. **EXPERIMENTAL**
//#define BOARD_BLACKPILL           // For use with F411 BlackPill, with optional auto square support for one axis
//#define BOARD_BLACKPILL_ALT2      // For use with F411 BlackPill, with optional auto square support for one axis, I2C and spindle sync
//#define BOARD_DEVTRONIC_CNC       // For use with Devtronic CNC Controller (F411 BlackPill), with I2C and spindle sync
//#define BOARD_DEVTRONIC_CNC_V2    // For use with Devtronic CNC Controller V2 (F411 BlackPill), with I2C and spindle sync
//#define BOARD_MINI_BLACKPILL      // For use with F411 BlackPill, https://github.com/avizienis/Minimal-Black-Pill--STM32F4xx-BOB-for-grblHAL
//#define BOARD_BTT_SKR_PRO_1_1     // F407 based 3D Printer board
//#define BOARD_BTT_SKR_PRO_1_2     // F407 based 3D Printer board
//#define BOARD_BTT_SKR_20          // F407 based 3D Printer board
//#define BOARD_BTT_SKR_20_DAC      // F407 based 3D Printer board, uses analog output (DAC) for spindle speed control
//#define BOARD_STM32F407VET6_DEV   // F407 based industrial control board
//#define BOARD_FYSETC_S6           // F446 based 3D Printer board
//#define BOARD_FLEXI_HAL           // F446 CNC board
//#define BOARD_LONGBOARD32         // F412 Sienci SLB, onboard Trinamic drivers version
//#define BOARD_LONGBOARD32_EXT     // F412 Sienci SLB, external drivers version
//#define BOARD_STM32F401_UNI       // F401 CNC board
//#define BOARD_MKS_ROBIN_NANO_30   // F407 based 3D Printer board - untested!
//#define BOARD_MKS_EAGLE           // F407 based 3D Printer board
//#define BOARD_MY_MACHINE          // Add my_machine_map.h before enabling this!

// WARNING: BOARD_BTT_SKR_20 may fry your Trinamic drivers due to bad hardware design.
//          The risk goes away if Q1 (HY1904C2) is shorted between source (S) and drain (D).
//          See this issue for details: https://github.com/bigtreetech/SKR-2/issues/63
//          Note: grblHAL does not check for reversed drivers.

#if defined(NUCLEO_F401) || defined(NUCLEO_F411) || defined(NUCLEO_F446)
#define IS_NUCLEO_DEVKIT 64
#elif defined(NUCLEO144_F446)
#define IS_NUCLEO_DEVKIT 144
#else
#define IS_NUCLEO_DEVKIT 0
#endif

// Configuration
// Uncomment to enable.

#if IS_NUCLEO_DEVKIT != 64 && !defined(USB_SERIAL_CDC) // The Nucleo boards has an off-chip UART to USB interface.
#define USB_SERIAL_CDC            1 // Serial communication via native USB.
#endif
//#define BLUETOOTH_ENABLE        2 // Set to 2 for HC-05 module. Requires and claims one auxiliary input pin.
// Spindle selection:
// Up to four specific spindle drivers can be instantiated at a time
// depending on N_SPINDLE and N_SYS_SPINDLE definitions in grbl/config.h.
// If none are specified the default PWM spindle is instantiated.
// Spindle definitions can be found in grbl/spindle_control.h.
// More here https://github.com/grblHAL/Plugins_spindle
//#define SPINDLE0_ENABLE         SPINDLE_ALL
//#define SPINDLE0_ENABLE         SPINDLE_PWM0_NODIR
//#define SPINDLE2_ENABLE         SPINDLE_NONE
//#define SPINDLE3_ENABLE         SPINDLE_NONE
//#define SPINDLE_OFFSET          1 // Uncomment to enable settings for laser spindle XY-offset.
// **********************
//#define MODBUS_ENABLE           1 // Set to 1 for auto direction, 2 for direction signal on auxiliary output pin.
//#define ETHERNET_ENABLE         1 // Ethernet streaming. Uses networking plugin.
//#define _WIZCHIP_            5500 // Enables ethernet via WIZnet breakout connected via SPI. Set to 5500 for W5500 chip, 5105 for W5100S.
//#define WEBUI_ENABLE            3 // Enable ESP3D-WEBUI plugin along with networking and SD card plugins.
                                    // NOTE: Only fully compatible with F412 and F429 MCUs.
//#define WEBUI_AUTH_ENABLE       1 // Enable ESP3D-WEBUI authentication.
//#define SDCARD_ENABLE           1 // Run gcode programs from SD card. Set to 2 to enable YModem upload.
//#define MPG_ENABLE              1 // Enable MPG interface. Requires a serial stream and means to switch between normal and MPG mode.
                                    // 1: Mode switching is by handshake pin.
                                    // 2: Mode switching is by the CMD_MPG_MODE_TOGGLE (0x8B) command character.
//#define KEYPAD_ENABLE           1 // 1: uses a I2C keypad for input.
                                    // 2: uses a serial stream for input. If MPG_ENABLE is set > 0 the serial stream is shared with the MPG.
//#define DISPLAY_ENABLE          1 // Set to 1 for I2C display protocol, 2 for I2C LED protocol.
//#define MACROS_ENABLE           1 // Macros plugin. For macros that can be triggered by keypad plugin or auxiliary inputs.
                                    // Set to 1 for aux input triggers, 2 for keypad triggers or 3 for both.
//#define N_MACROS                1 // Number of macros to enable, max 8. If commented out default is 4 when triggered by aux inputs, 8 otherwise.
//#define ODOMETER_ENABLE         1 // Odometer plugin.
//#define PLASMA_ENABLE           1 // Plasma (THC) plugin. To be completed.
//#define EMBROIDERY_ENABLE       1 // Embroidery plugin. To be completed.
//#define PPI_ENABLE              1 // Laser PPI plugin. To be completed.
//#define LASER_COOLANT_ENABLE    1 // Laser coolant plugin. To be completed.
//#define LB_CLUSTERS_ENABLE      1 // LaserBurn cluster support.
//#define TRINAMIC_ENABLE      2130 // Trinamic TMC2130 stepper driver support.
//#define TRINAMIC_ENABLE      5160 // Trinamic TMC5160 stepper driver support.
//#define TRINAMIC_ENABLE      2209 // Trinamic TMC2209 stepper driver support.
//#define TRINAMIC_ENABLE      2660 // Trinamic TMC2660 stepper driver support.
//#define TRINAMIC_R_SENSE      110 // R sense resistance in milliohms, 2130 and 2209 default is 110, 5160 is 75.
//#define TRINAMIC_I2C            1 // Trinamic I2C - SPI bridge interface.
//#define TRINAMIC_DEV            1 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code.
//#define FANS_ENABLE             1 // Enable fan control via M106/M107. Enables fans plugin.
//#define EEPROM_ENABLE          16 // I2C EEPROM/FRAM support. Set to 16 for 2K, 32 for 4K, 64 for 8K, 128 for 16K and 256 for 32K capacity.
//#define EEPROM_IS_FRAM          1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.
//#define SPINDLE_SYNC_ENABLE     1 // Enable spindle sync support (G33, G76). !! NOTE: Alpha quality - enable only for test or verification.
                                    // Currently available for BOARD_PROTONEER_3XX, BOARD_BLACKPILL*, BOARD_MORPHO_CNC and BOARD_STM32F401_UNI.
//#define ESTOP_ENABLE            0 // When enabled only real-time report requests will be executed when the reset pin is asserted.
                                    // Note: if left commented out the default setting is determined from COMPATIBILITY_LEVEL.
//#define MCP3221_ENABLE       0x4D // Enable MCP3221 I2C ADC input with address 0x4D (0b01001101).
//#define RGB_LED_ENABLE          2 // Set to 1 to enable strip length settings $536 and $537, set to 2 to also enable M150 LED strip control.
//#define PWM_SERVO_ENABLE        1 // Enable M280 PWM servo support, requires at least one PWM capable auxiliary output.
//#define BLTOUCH_ENABLE          1 // Enable M401/M402 BLTouch support. Requires and claims one auxiliary PWM servo output.
//#define EVENTOUT_ENABLE         1 // Enable binding events (triggers) to control auxiliary outputs.
//#define ESP_AT_ENABLE           1 // Enable support for Telnet communication via UART connected ESP32 running ESP-AT.
//#define FEED_OVERRIDE_ENABLE    1 // Enable M200 feed override control.
//#define HOMING_PULLOFF_ENABLE   1 // Enable per axis homing pulloff distance settings.

// Optional control signals:
// These will be assigned to aux input pins. Use the $pins command to check which pins are assigned.
// NOTE: If not enough pins are available assignment will silently fail.
//#define PROBE_ENABLE            0 // Default enabled, remove comment to disable probe input.
//#define SAFETY_DOOR_ENABLE      1
//#define MOTOR_FAULT_ENABLE      1
//#define MOTOR_WARNING_ENABLE    1
//#define PROBE_DISCONNECT_ENABLE 1
//#define STOP_DISABLE_ENABLE     1
//#define BLOCK_DELETE_ENABLE     1
//#define SINGLE_BLOCK_ENABLE     1
//#define LIMITS_OVERRIDE_ENABLE  1

/**/
// If the selected board map supports more than three motors ganging and/or auto-squaring
// of axes can be enabled here.
//#define X_GANGED            1
//#define X_AUTO_SQUARE       1
//#define Y_GANGED            1
//#define Y_AUTO_SQUARE       1
//#define Z_GANGED            1
//#define Z_AUTO_SQUARE       1
// For ganged axes the limit switch input (if available) can be configured to act as a max travel limit switch.
// NOTE: If board map already has max limit inputs defined this configuration will be ignored.
//#define X_GANGED_LIM_MAX    1
//#define Y_GANGED_LIM_MAX    1
//#define Z_GANGED_LIM_MAX    1
//

#ifdef _WIZCHIP_
#define ETHERNET_ENABLE 1
#endif

#if ETHERNET_ENABLE || WEBUI_ENABLE
#define TELNET_ENABLE       1 // Telnet daemon - requires Ethernet streaming enabled.
#define WEBSOCKET_ENABLE    1 // Websocket daemon - requires Ethernet streaming enabled.
//#define MDNS_ENABLE         1 // mDNS daemon.
//#define SSDP_ENABLE         1 // SSDP daemon - requires HTTP enabled.
//#define MQTT_ENABLE         1 // MQTT client API, only enable if needed by plugin code.
#if SDCARD_ENABLE || WEBUI_ENABLE
//#define FTP_ENABLE         1 // Ftp daemon - requires SD card enabled.
//#define HTTP_ENABLE         1 // http daemon - requires SD card enabled.
//#define WEBDAV_ENABLE       1 // webdav protocol - requires http daemon and SD card enabled.
#endif

// The following symbols have the default values as shown, uncomment and change as needed.
//#define NETWORK_HOSTNAME        "grblHAL"
//#define NETWORK_IPMODE          1 // 0 = static, 1 = DHCP, 2 = AutoIP
//#define NETWORK_IP              "192.168.5.1"
//#define NETWORK_GATEWAY         "192.168.5.1"
//#define NETWORK_MASK            "255.255.255.0"
//#define NETWORK_FTP_PORT        21
//#define NETWORK_TELNET_PORT     23
//#define NETWORK_HTTP_PORT       80
#if HTTP_ENABLE
//#define NETWORK_WEBSOCKET_PORT  81
#else
//#define NETWORK_WEBSOCKET_PORT  80
#endif
#endif
