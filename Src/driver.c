/*

  driver.c - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2022 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "driver.h"
#include "serial.h"

#include "grbl/protocol.h"
#include "grbl/motor_pins.h"
#include "grbl/pin_bits_masks.h"
#include "grbl/state_machine.h"
#include "grbl/machine_limits.h"

#if I2C_ENABLE
#include "i2c.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "ff.h"
#include "diskio.h"
#endif

#if USB_SERIAL_CDC
#include "usb_serial.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if PPI_ENABLE
#include "laser/ppi.h"
#endif

#if KEYPAD_ENABLE == 2
#include "keypad/keypad.h"
#endif

#if FLASH_ENABLE
#include "flash.h"
#endif

#if ETHERNET_ENABLE
  #include "enet.h"
  #if TELNET_ENABLE
    #include "networking/TCPStream.h"
  #endif
  #if WEBSOCKET_ENABLE
    #include "networking/WsStream.h"
  #endif
#endif

#if !I2C_STROBE_ENABLE
#define I2C_STROBE_BIT 0
#endif

#if !SPINDLE_SYNC_ENABLE
#define SPINDLE_INDEX_BIT 0
#else
#ifndef SPINDLE_INDEX_BIT
#define SPINDLE_INDEX_BIT (1<<SPINDLE_INDEX_PIN)
#endif
#ifndef SPINDLE_PULSE_BIT
#define SPINDLE_PULSE_BIT (1<<SPINDLE_PULSE_PIN)
#endif
#endif

#if !SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_BIT 0
#endif

#ifdef MPG_MODE_PIN
#define MPG_MODE_BIT (1<<MPG_MODE_PIN)
#else
#define MPG_MODE_BIT 0
#endif

#if CONTROL_MASK != (RESET_BIT+FEED_HOLD_BIT+CYCLE_START_BIT+SAFETY_DOOR_BIT)
#error Interrupt enabled input pins must have unique pin numbers!
#endif

#define DRIVER_IRQMASK (LIMIT_MASK|CONTROL_MASK|I2C_STROBE_BIT|SPINDLE_INDEX_BIT|MPG_MODE_BIT)

#ifdef Z_LIMIT_POLL
#if (DRIVER_IRQMASK-Z_LIMIT_BIT) != (LIMIT_MASK-Z_LIMIT_BIT+CONTROL_MASK+I2C_STROBE_BIT+SPINDLE_INDEX_BIT+MPG_MODE_BIT)
#error Interrupt enabled input pins must have unique pin numbers!
#endif
#else
#if DRIVER_IRQMASK != (LIMIT_MASK+CONTROL_MASK+I2C_STROBE_BIT+SPINDLE_INDEX_BIT+MPG_MODE_BIT)
#error Interrupt enabled input pins must have unique pin numbers!
#endif
#endif

typedef union {
    uint8_t mask;
    struct {
        uint8_t limits :1,
                door   :1,
                unused :6;
    };
} debounce_t;

#if (!VFD_SPINDLE || N_SPINDLE > 1) && defined(SPINDLE_ENABLE_PIN)

#define DRIVER_SPINDLE

#if defined(SPINDLE_PWM_TIMER_N)
static bool pwmEnabled = false;
static spindle_pwm_t spindle_pwm;
static void spindle_set_speed (uint_fast16_t pwm_value);
#endif

#if SPINDLE_SYNC_ENABLE && !defined(SPINDLE_PWM_TIMER_N)
#undef SPINDLE_SYNC_ENABLE
#endif

#elif defined(SPINDLE_PWM_TIMER_N)
#undef SPINDLE_PWM_TIMER_N
#endif

static periph_signal_t *periph_pins = NULL;

static input_signal_t inputpin[] = {
#if ESTOP_ENABLE
    { .id = Input_EStop,          .port = RESET_PORT,         .pin = RESET_PIN,           .group = PinGroup_Control },
#else
    { .id = Input_Reset,          .port = RESET_PORT,         .pin = RESET_PIN,           .group = PinGroup_Control },
#endif
    { .id = Input_FeedHold,       .port = FEED_HOLD_PORT,     .pin = FEED_HOLD_PIN,       .group = PinGroup_Control },
    { .id = Input_CycleStart,     .port = CYCLE_START_PORT,   .pin = CYCLE_START_PIN,     .group = PinGroup_Control },
#if SAFETY_DOOR_ENABLE
    { .id = Input_SafetyDoor,     .port = SAFETY_DOOR_PORT,   .pin = SAFETY_DOOR_PIN,     .group = PinGroup_Control },
#endif
#ifdef PROBE_PIN
    { .id = Input_Probe,          .port = PROBE_PORT,         .pin = PROBE_PIN,           .group = PinGroup_Probe },
#endif
#ifdef I2C_STROBE_PIN
    { .id = Input_KeypadStrobe,   .port = I2C_STROBE_PORT,    .pin = I2C_STROBE_PIN,      .group = PinGroup_Keypad },
#endif
#ifdef MPG_MODE_PIN
    { .id = Input_MPGSelect,      .port = MPG_MODE_PORT,      .pin = MPG_MODE_PIN,        .group = PinGroup_MPG },
#endif
// Limit input pins must be consecutive in this array
    { .id = Input_LimitX,         .port = X_LIMIT_PORT,       .pin = X_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_2,       .port = X2_LIMIT_PORT,      .pin = X2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
    { .id = Input_LimitY,         .port = Y_LIMIT_PORT,       .pin = Y_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef Y2_LIMIT_PIN
    { .id = Input_LimitY_2,       .port = Y2_LIMIT_PORT,      .pin = Y2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
    { .id = Input_LimitZ,         .port = Z_LIMIT_PORT,       .pin = Z_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef Z2_LIMIT_PIN
    { .id = Input_LimitZ_2,       .port = Z2_LIMIT_PORT,      .pin = Z2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
#ifdef A_LIMIT_PIN
    { .id = Input_LimitA,         .port = A_LIMIT_PORT,       .pin = A_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef B_LIMIT_PIN
    { .id = Input_LimitB,         .port = B_LIMIT_PORT,       .pin = B_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef C_LIMIT_PIN
    { .id = Input_LimitC,         .port = C_LIMIT_PORT,       .pin = C_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#if SPINDLE_SYNC_ENABLE
    { .id = Input_SpindleIndex,   .port = SPINDLE_INDEX_PORT, .pin = SPINDLE_INDEX_PIN,   .group = PinGroup_SpindleIndex },
#endif
// Aux input pins must be consecutive in this array
#ifdef AUXINPUT0_PIN
    { .id = Input_Aux0,           .port = AUXINPUT0_PORT,     .pin = AUXINPUT0_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT1_PIN
    { .id = Input_Aux1,           .port = AUXINPUT1_PORT,     .pin = AUXINPUT1_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT2_PIN
    { .id = Input_Aux2,           .port = AUXINPUT2_PORT,     .pin = AUXINPUT2_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT3_PIN
    { .id = Input_Aux3,           .port = AUXINPUT3_PORT,     .pin = AUXINPUT3_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT4_PIN
    { .id = Input_Aux4,           .port = AUXINPUT4_PORT,     .pin = AUXINPUT4_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT5_PIN
    { .id = Input_Aux5,           .port = AUXINPUT5_PORT,     .pin = AUXINPUT5_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT6_PIN
    { .id = Input_Aux6,           .port = AUXINPUT6_PORT,     .pin = AUXINPUT6_PIN,       .group = PinGroup_AuxInput }
#endif
};

static output_signal_t outputpin[] = {
    { .id = Output_StepX,           .port = X_STEP_PORT,            .pin = X_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
    { .id = Output_StepY,           .port = Y_STEP_PORT,            .pin = Y_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
    { .id = Output_StepZ,           .port = Z_STEP_PORT,            .pin = Z_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#ifdef A_AXIS
    { .id = Output_StepA,           .port = A_STEP_PORT,            .pin = A_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef B_AXIS
    { .id = Output_StepB,           .port = B_STEP_PORT,            .pin = B_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef C_AXIS
    { .id = Output_StepC,           .port = C_STEP_PORT,            .pin = C_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef X2_STEP_PIN
    { .id = Output_StepX_2,         .port = X2_STEP_PORT,           .pin = X2_STEP_PIN,             .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef Y2_STEP_PIN
    { .id = Output_StepY_2,         .port = Y2_STEP_PORT,           .pin = Y2_STEP_PIN,             .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef Z2_STEP_PIN
    { .id = Output_StepZ_2,         .port = Z2_STEP_PORT,           .pin = Z2_STEP_PIN,             .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
    { .id = Output_DirX,            .port = X_DIRECTION_PORT,       .pin = X_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
    { .id = Output_DirY,            .port = Y_DIRECTION_PORT,       .pin = Y_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
    { .id = Output_DirZ,            .port = Z_DIRECTION_PORT,       .pin = Z_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#ifdef A_AXIS
    { .id = Output_DirA,            .port = A_DIRECTION_PORT,       .pin = A_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,            .port = B_DIRECTION_PORT,       .pin = B_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,            .port = C_DIRECTION_PORT,       .pin = C_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef X2_DIRECTION_PIN
    { .id = Output_DirX_2,          .port = X2_DIRECTION_PORT,      .pin = X2_DIRECTION_PIN,        .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef Y2_DIRECTION_PIN
    { .id = Output_DirY_2,          .port = Y2_DIRECTION_PORT,      .pin = Y2_DIRECTION_PIN,        .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef Z2_DIRECTION_PIN
    { .id = Output_DirZ_2,          .port = Z2_DIRECTION_PORT,      .pin = Z2_DIRECTION_PIN,        .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef STEPPERS_POWER_PORT
    { .id = Output_StepperPower,    .port = STEPPERS_POWER_PORT,    .pin = STEPPERS_POWER_PIN,      .group = PinGroup_StepperPower },
#endif
#if !TRINAMIC_MOTOR_ENABLE
#ifdef STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnable,   .port = STEPPERS_ENABLE_PORT,   .pin = STEPPERS_ENABLE_PIN,     .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef X_ENABLE_PORT
    { .id = Output_StepperEnableX,  .port = X_ENABLE_PORT,          .pin = X_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Y_ENABLE_PORT
    { .id = Output_StepperEnableY,  .port = Y_ENABLE_PORT,          .pin = Y_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Z_ENABLE_PORT
    { .id = Output_StepperEnableZ,  .port = Z_ENABLE_PORT,          .pin = Z_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef A_ENABLE_PORT
    { .id = Output_StepperEnableA,  .port = A_ENABLE_PORT,          .pin = A_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef B_ENABLE_PORT
    { .id = Output_StepperEnableB,  .port = B_ENABLE_PORT,          .pin = B_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef C_ENABLE_PORT
    { .id = Output_StepperEnableC,  .port = C_ENABLE_PORT,          .pin = C_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef X2_ENABLE_PIN
    { .id = Output_StepperEnableX,  .port = X2_ENABLE_PORT,         .pin = X2_ENABLE_PIN,           .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Y2_ENABLE_PIN
    { .id = Output_StepperEnableY,  .port = Y2_ENABLE_PORT,         .pin = Y2_ENABLE_PIN,           .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Z2_ENABLE_PIN
    { .id = Output_StepperEnableZ,  .port = Z2_ENABLE_PORT,         .pin = Z2_ENABLE_PIN,           .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#endif // TRINAMIC_MOTOR_ENABLE
#ifdef MOTOR_CS_PIN
    { .id = Output_MotorChipSelect,     .port = MOTOR_CS_PORT,      .pin = MOTOR_CS_PIN,            .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSX_PIN
    { .id = Output_MotorChipSelectX,    .port = MOTOR_CSX_PORT,     .pin = MOTOR_CSX_PIN,           .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSY_PIN
    { .id = Output_MotorChipSelectY,    .port = MOTOR_CSY_PORT,     .pin = MOTOR_CSY_PIN,           .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSZ_PIN
    { .id = Output_MotorChipSelectZ,    .port = MOTOR_CSZ_PORT,     .pin = MOTOR_CSZ_PIN,           .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM3_PIN
    { .id = Output_MotorChipSelectM3,   .port = MOTOR_CSM3_PORT,    .pin = MOTOR_CSM3_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM4_PIN
    { .id = Output_MotorChipSelectM4,   .port = MOTOR_CSM4_PORT,    .pin = MOTOR_CSM4_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM5_PIN
    { .id = Output_MotorChipSelectM5,   .port = MOTOR_CSM5_PORT,    .pin = MOTOR_CSM5_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CS_PIN
    { .id = Output_MotorChipSelect,     .port = MOTOR_CS_PORT,      .pin = MOTOR_CS_PIN,            .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_UARTX_PIN
    { .id = Bidirectional_MotorUARTX,   .port = MOTOR_UARTX_PORT,   .pin = MOTOR_UARTX_PIN,         .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTY_PIN
    { .id = Bidirectional_MotorUARTY,   .port = MOTOR_UARTY_PORT,   .pin = MOTOR_UARTY_PIN,         .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTZ_PIN
    { .id = Bidirectional_MotorUARTZ,   .port = MOTOR_UARTZ_PORT,   .pin = MOTOR_UARTZ_PIN,         .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM3_PIN
    { .id = Bidirectional_MotorUARTM3,  .port = MOTOR_UARTM3_PORT,  .pin = MOTOR_UARTM3_PIN,        .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM4_PIN
    { .id = Bidirectional_MotorUARTM4,  .port = MOTOR_UARTM4_PORT,  .pin = MOTOR_UARTM4_PIN,        .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM5_PIN
    { .id = Bidirectional_MotorUARTM5,  .port = MOTOR_UARTM5_PORT,  .pin = MOTOR_UARTM5_PIN,        .group = PinGroup_MotorUART },
#endif
#ifdef DRIVER_SPINDLE
#ifdef SPINDLE_ENABLE_PIN
    { .id = Output_SpindleOn,       .port = SPINDLE_ENABLE_PORT,    .pin = SPINDLE_ENABLE_PIN,      .group = PinGroup_SpindleControl },
#endif
#ifdef SPINDLE_DIRECTION_PIN
    { .id = Output_SpindleDir,      .port = SPINDLE_DIRECTION_PORT, .pin = SPINDLE_DIRECTION_PIN,   .group = PinGroup_SpindleControl },
#endif
#endif // DRIVER_SPINDLE
    { .id = Output_CoolantFlood,    .port = COOLANT_FLOOD_PORT,     .pin = COOLANT_FLOOD_PIN,       .group = PinGroup_Coolant },
#ifdef COOLANT_MIST_PIN
    { .id = Output_CoolantMist,     .port = COOLANT_MIST_PORT,      .pin = COOLANT_MIST_PIN,        .group = PinGroup_Coolant },
#endif
#ifdef SD_CS_PORT
    { .id = Output_SdCardCS,        .port = SD_CS_PORT,             .pin = SD_CS_PIN,               .group = PinGroup_SdCard },
#endif
#ifdef AUXOUTPUT0_PORT
    { .id = Output_Aux0,            .port = AUXOUTPUT0_PORT,        .pin = AUXOUTPUT0_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT1_PORT
    { .id = Output_Aux1,            .port = AUXOUTPUT1_PORT,        .pin = AUXOUTPUT1_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT2_PORT
    { .id = Output_Aux2,            .port = AUXOUTPUT2_PORT,        .pin = AUXOUTPUT2_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT3_PORT
    { .id = Output_Aux3,            .port = AUXOUTPUT3_PORT,        .pin = AUXOUTPUT3_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT4_PORT
    { .id = Output_Aux4,            .port = AUXOUTPUT4_PORT,        .pin = AUXOUTPUT4_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT5_PORT
    { .id = Output_Aux5,            .port = AUXOUTPUT5_PORT,        .pin = AUXOUTPUT5_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT6_PORT
    { .id = Output_Aux6,            .port = AUXOUTPUT6_PORT,        .pin = AUXOUTPUT6_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT7_PORT
    { .id = Output_Aux7,            .port = AUXOUTPUT7_PORT,        .pin = AUXOUTPUT7_PIN,          .group = PinGroup_AuxOutput }
#endif
};

extern __IO uint32_t uwTick;
static uint32_t pulse_length, pulse_delay, aux_irq = 0;;
static bool IOInitDone = false;
static axes_signals_t next_step_outbits;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static debounce_t debounce;
#ifdef PROBE_PIN
static probe_state_t probe = {
    .connected = On
};
#endif

#if I2C_STROBE_ENABLE

static driver_irq_handler_t i2c_strobe = { .type = IRQ_I2C_Strobe };

static bool irq_claim (irq_type_t irq, uint_fast8_t id, irq_callback_ptr handler)
{
    bool ok;

    if((ok = irq == IRQ_I2C_Strobe && i2c_strobe.callback == NULL))
        i2c_strobe.callback = handler;

    return ok;
}

#endif

#include "grbl/stepdir_map.h"

#ifdef SQUARING_ENABLED
static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};
#endif

#if SPINDLE_SYNC_ENABLE

#include "grbl/spindle_sync.h"

static spindle_data_t spindle_data;
static spindle_encoder_t spindle_encoder = {
    .tics_per_irq = 4
};
static spindle_sync_t spindle_tracker;
static volatile bool spindleLock = false;
#if RPM_COUNTER_N == 2
static volatile uint32_t rpm_timer_ovf = 0;
#define RPM_TIMER_COUNT (RPM_TIMER->CNT | (rpm_timer_ovf << 16))
#else
#define RPM_TIMER_COUNT RPM_TIMER->CNT
#endif

static void stepperPulseStartSynchronized (stepper_t *stepper);
static void spindleDataReset (void);
static spindle_data_t *spindleGetData (spindle_data_request_t request);

#endif

static void driver_delay (uint32_t ms, delay_callback_ptr callback)
{
    if((delay.ms = ms) > 0) {
        if(!(delay.callback = callback)) {
            while(delay.ms)
                grbl.on_execute_delay(state_get());
        }
    } else {
        delay.callback = NULL;
        if(callback)
            callback();
    }
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
#if !TRINAMIC_MOTOR_ENABLE
  #ifdef STEPPERS_ENABLE_PORT
    DIGITAL_OUT(STEPPERS_ENABLE_PORT, STEPPERS_ENABLE_PIN, enable.x);
  #else
    DIGITAL_OUT(X_ENABLE_PORT, X_ENABLE_PIN, enable.x);
   #ifdef X2_ENABLE_PIN
    DIGITAL_OUT(X2_ENABLE_PORT, X2_ENABLE_PIN, enable.x);
   #endif
    DIGITAL_OUT(Y_ENABLE_PORT, Y_ENABLE_PIN, enable.y);
   #ifdef Y2_ENABLE_PIN
    DIGITAL_OUT(Y2_ENABLE_PORT, Y2_ENABLE_PIN, enable.y);
   #endif
    DIGITAL_OUT(Z_ENABLE_PORT, Z_ENABLE_PIN, enable.z);
   #ifdef Z2_ENABLE_PIN
    DIGITAL_OUT(Z2_ENABLE_PORT, Z2_ENABLE_PIN, enable.z);
   #endif
   #ifdef A_ENABLE_PORT
    DIGITAL_OUT(A_ENABLE_PORT, A_ENABLE_PIN, enable.a);
   #endif
   #ifdef B_ENABLE_PORT
    DIGITAL_OUT(B_ENABLE_PORT, B_ENABLE_PIN, enable.b);
   #endif
   #ifdef C_ENABLE_PORT
    DIGITAL_OUT(C_ENABLE_PORT, C_ENABLE_PIN, enable.c);
   #endif
  #endif
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});

    STEPPER_TIMER->ARR = 5000; // delay to allow drivers time to wake up
    STEPPER_TIMER->EGR = TIM_EGR_UG;
    STEPPER_TIMER->CR1 |= TIM_CR1_CEN;
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    STEPPER_TIMER->CR1 &= ~TIM_CR1_CEN;
    STEPPER_TIMER->CNT = 0;
}

// Sets up stepper driver interrupt timeout, "Normal" version
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    STEPPER_TIMER->ARR = cycles_per_tick < (1UL << 20) ? cycles_per_tick : 0x000FFFFFUL;
}

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
#ifdef SQUARING_ENABLED

inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_outbits_1)
{
    axes_signals_t step_outbits_2;
    step_outbits_2.mask = (step_outbits_1.mask & motors_2.mask) ^ settings.steppers.step_invert.mask;

#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits_1.mask = (step_outbits_1.mask & motors_1.mask) ^ settings.steppers.step_invert.mask;

    DIGITAL_OUT(X_STEP_PORT, X_STEP_PIN, step_outbits_1.x);
 #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_outbits_2.x);
 #endif
    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_PIN, step_outbits_1.y);
 #ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_outbits_2.y);
 #endif
    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_PIN, step_outbits_1.z);
 #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_outbits_2.z);
 #endif
 #ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PORT, A_STEP_PIN, step_outbits_1.a);
 #endif
 #ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PORT, B_STEP_PIN, step_outbits_1.b);
 #endif
 #ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PORT, C_STEP_PIN, step_outbits_1.c);
 #endif

#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | step_outmap[step_outbits_1.value & motors_1.mask];
 #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_outbits_2.x);
 #endif
 #ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_outbits_2.y);
 #endif
 #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_outbits_2.z);
 #endif

#else // STEP_OUTMODE == GPIO_SHIFTx
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | (((step_outbits_1.mask & motors_1.mask) ^ settings.steppers.step_invert.mask) << STEP_OUTMODE);
 #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_outbits_2.x);
 #endif
 #ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_outbits_2.y);
 #endif
 #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_outbits_2.z);
 #endif
#endif
}

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_1.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
    motors_2.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
}

#else // SQUARING DISABLED

inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_outbits)
{
#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits.mask ^= settings.steppers.step_invert.mask;
    DIGITAL_OUT(X_STEP_PORT, X_STEP_PIN, step_outbits.x);
  #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_outbits.x);
  #endif
    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_PIN, step_outbits.y);
  #ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_outbits.y);
  #endif
    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_PIN, step_outbits.z);
  #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_outbits.z);
  #endif
  #ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PORT, A_STEP_PIN, step_outbits.a);
  #endif
  #ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PORT, B_STEP_PIN, step_outbits.b);
  #endif
  #ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PORT, C_STEP_PIN, step_outbits.c);
  #endif
#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | step_outmap[step_outbits.value];
  #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_outbits.x ^ settings.steppers.step_invert.x);
  #endif
  #ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_outbits.y ^ settings.steppers.step_invert.y);
  #endif
  #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_outbits.z ^ settings.steppers.step_invert.z);
  #endif
#else // STEP_OUTMODE == GPIO_SHIFTx
 #if N_GANGED
   step_outbits.mask ^= settings.steppers.step_invert.mask;
   STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | (step_outbits.mask << STEP_OUTMODE);
  #ifdef X2_STEP_PIN
   DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_outbits.x);
  #endif
  #ifdef Y2_PIN
   DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_outbits.y);
  #endif
  #ifdef Z2_STEP_PIN
   DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_outbits.z);
  #endif
 #else
   STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | ((step_outbits.value << STEP_OUTMODE) ^ settings.steppers.step_invert.value);
 #endif
#endif
}

#endif

#ifdef GANGING_ENABLED

static axes_signals_t getGangedAxes (bool auto_squared)
{
    axes_signals_t ganged = {0};

    if(auto_squared) {
        #if X_AUTO_SQUARE
            ganged.x = On;
        #endif

        #if Y_AUTO_SQUARE
            ganged.y = On;
        #endif

        #if Z_AUTO_SQUARE
            ganged.z = On;
        #endif
    } else {
        #if X_GANGED
            ganged.x = On;
        #endif

        #if Y_GANGED
            ganged.y = On;
        #endif

        #if Z_GANGED
            ganged.z = On;
        #endif
    }

    return ganged;
}

#endif

// Set stepper direction output pins
// NOTE: see note for stepperSetStepOutputs()
inline static __attribute__((always_inline)) void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
#if DIRECTION_OUTMODE == GPIO_BITBAND
    dir_outbits.mask ^= settings.steppers.dir_invert.mask;
    DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_PIN, dir_outbits.x);
    DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_PIN, dir_outbits.y);
    DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_PIN, dir_outbits.z);
 #ifdef GANGING_ENABLED
    dir_outbits.mask ^= settings.steppers.ganged_dir_invert.mask;
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_PIN, dir_outbits.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_PIN, dir_outbits.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_PIN, dir_outbits.z);
  #endif
 #endif
 #ifdef A_AXIS
    DIGITAL_OUT(A_DIRECTION_PORT, A_DIRECTION_PIN, dir_outbits.a);
 #endif
 #ifdef B_AXIS
    DIGITAL_OUT(B_DIRECTION_PORT, B_DIRECTION_PIN, dir_outbits.b);
 #endif
 #ifdef C_AXIS
    DIGITAL_OUT(C_DIRECTION_PORT, C_DIRECTION_PIN, dir_outbits.c);
 #endif
#elif DIRECTION_OUTMODE == GPIO_MAP
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | dir_outmap[dir_outbits.value];
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_PIN, (dir_outbits.x ^ settings.steppers.dir_invert.x) ^ settings.steppers.ganged_dir_invert.x;
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_PIN, (dir_outbits.y ^ settings.steppers.dir_invert.y) ^ settings.steppers.ganged_dir_invert.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_PIN, (dir_outbits.z ^ settings.steppers.dir_invert.z) ^ settings.steppers.ganged_dir_invert.z;
  #endif
#else // DIRECTION_OUTMODE = SHIFTx
 #ifdef GANGING_ENABLED
    dir_outbits.mask ^= settings.steppers.dir_invert.mask;
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | (dir_outbits.mask << DIRECTION_OUTMODE);
    dir_outbits.mask ^= settings.steppers.ganged_dir_invert.mask;
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_PIN, dir_outbits.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_PIN, dir_outbits.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_PIN, dir_outbits.z);
  #endif
 #else
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | ((dir_outbits.mask ^ settings.steppers.dir_invert.mask) << DIRECTION_OUTMODE);
 #endif
#endif
}

// Sets stepper direction and pulse pins and starts a step pulse.
static void stepperPulseStart (stepper_t *stepper)
{
#if SPINDLE_SYNC_ENABLE
    if(stepper->new_block && stepper->exec_segment->spindle_sync) {
        spindle_tracker.stepper_pulse_start_normal = hal.stepper.pulse_start;
        hal.stepper.pulse_start = stepperPulseStartSynchronized;
        hal.stepper.pulse_start(stepper);
        return;
    }
#endif

    if(stepper->dir_change)
        stepperSetDirOutputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
        stepperSetStepOutputs(stepper->step_outbits);
        PULSE_TIMER->EGR = TIM_EGR_UG;
        PULSE_TIMER->CR1 |= TIM_CR1_CEN;
    }
}

// Start a stepper pulse, delay version.
// Note: delay is only added when there is a direction change and a pulse to be output.
static void stepperPulseStartDelayed (stepper_t *stepper)
{
#if SPINDLE_SYNC_ENABLE
    if(stepper->new_block && stepper->exec_segment->spindle_sync) {
        spindle_tracker.stepper_pulse_start_normal = hal.stepper.pulse_start;
        hal.stepper.pulse_start = stepperPulseStartSynchronized;
        hal.stepper.pulse_start(stepper);
        return;
    }
#endif

    if(stepper->dir_change) {

        stepperSetDirOutputs(stepper->dir_outbits);

        if(stepper->step_outbits.value) {
            next_step_outbits = stepper->step_outbits; // Store out_bits
            PULSE_TIMER->ARR = pulse_delay;
            PULSE_TIMER->EGR = TIM_EGR_UG;
            PULSE_TIMER->CR1 |= TIM_CR1_CEN;
        }

        return;
    }

    if(stepper->step_outbits.value) {
        stepperSetStepOutputs(stepper->step_outbits);
        PULSE_TIMER->EGR = TIM_EGR_UG;
        PULSE_TIMER->CR1 |= TIM_CR1_CEN;
    }
}

#if SPINDLE_SYNC_ENABLE

// Spindle sync version: sets stepper direction and pulse pins and starts a step pulse.
// Switches back to "normal" version if spindle synchronized motion is finished.
// TODO: add delayed pulse handling...
static void stepperPulseStartSynchronized (stepper_t *stepper)
{
    static bool sync = false;
    static float block_start;

    if(stepper->new_block) {
        if(!stepper->exec_segment->spindle_sync) {
            hal.stepper.pulse_start = spindle_tracker.stepper_pulse_start_normal;
            hal.stepper.pulse_start(stepper);
            return;
        }
        sync = true;
        stepperSetDirOutputs(stepper->dir_outbits);
        spindle_tracker.programmed_rate = stepper->exec_block->programmed_rate;
        spindle_tracker.steps_per_mm = stepper->exec_block->steps_per_mm;
        spindle_tracker.segment_id = 0;
        spindle_tracker.prev_pos = 0.0f;
        block_start = spindleGetData(SpindleData_AngularPosition)->angular_position * spindle_tracker.programmed_rate;
        pidf_reset(&spindle_tracker.pid);
#ifdef PID_LOG
        sys.pid_log.idx = 0;
        sys.pid_log.setpoint = 100.0f;
#endif
    }

    if(stepper->step_outbits.value) {
        stepperSetStepOutputs(stepper->step_outbits);
        PULSE_TIMER->EGR = TIM_EGR_UG;
        PULSE_TIMER->CR1 |= TIM_CR1_CEN;
    }

    if(spindle_tracker.segment_id != stepper->exec_segment->id) {

        spindle_tracker.segment_id = stepper->exec_segment->id;

        if(!stepper->new_block) {  // adjust this segments total time for any positional error since last segment

            float actual_pos;

            if(stepper->exec_segment->cruising) {

                float dt = (float)hal.f_step_timer / (float)(stepper->exec_segment->cycles_per_tick * stepper->exec_segment->n_step);
                actual_pos = spindleGetData(SpindleData_AngularPosition)->angular_position * spindle_tracker.programmed_rate;

                if(sync) {
                    spindle_tracker.pid.sample_rate_prev = dt;
//                    spindle_tracker.block_start += (actual_pos - spindle_tracker.block_start) - spindle_tracker.prev_pos;
//                    spindle_tracker.block_start += spindle_tracker.prev_pos;
                    sync = false;
                }

                actual_pos -= block_start;
                int32_t step_delta = (int32_t)(pidf(&spindle_tracker.pid, spindle_tracker.prev_pos, actual_pos, dt) * spindle_tracker.steps_per_mm);
                int32_t ticks = (((int32_t)stepper->step_count + step_delta) * (int32_t)stepper->exec_segment->cycles_per_tick) / (int32_t)stepper->step_count;

                stepper->exec_segment->cycles_per_tick = (uint32_t)max(ticks, spindle_tracker.min_cycles_per_tick >> stepper->exec_segment->amass_level);

                stepperCyclesPerTick(stepper->exec_segment->cycles_per_tick);
           } else
                actual_pos = spindle_tracker.prev_pos;

#ifdef PID_LOG
            if(sys.pid_log.idx < PID_LOG) {

                sys.pid_log.target[sys.pid_log.idx] = spindle_tracker.prev_pos;
                sys.pid_log.actual[sys.pid_log.idx] = actual_pos; // - spindle_tracker.prev_pos;

            //    spindle_tracker.log[sys.pid_log.idx] = STEPPER_TIMER->BGLOAD << stepper->amass_level;
            //    spindle_tracker.pos[sys.pid_log.idx] = stepper->exec_segment->cycles_per_tick  stepper->amass_level;
            //    spindle_tracker.pos[sys.pid_log.idx] = stepper->exec_segment->cycles_per_tick * stepper->step_count;
            //    STEPPER_TIMER->BGLOAD = STEPPER_TIMER->LOAD;

             //   spindle_tracker.pos[sys.pid_log.idx] = spindle_tracker.prev_pos;

                sys.pid_log.idx++;
            }
#endif
        }

        spindle_tracker.prev_pos = stepper->exec_segment->target_position;
    }
}

#endif

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
    if(on) {
        EXTI->PR |= LIMIT_MASK;     // Clear any pending limit interrupts
        EXTI->IMR |= LIMIT_MASK;    // and enable
    } else
        EXTI->IMR &= ~LIMIT_MASK;
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

    signals.min.mask = settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
    signals.min2.mask = settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
    signals.max.mask = settings.limits.invert.mask;
#endif

#if LIMIT_INMODE == GPIO_BITBAND
    signals.min.x = DIGITAL_IN(X_LIMIT_PORT, X_LIMIT_PIN);
    signals.min.y = DIGITAL_IN(Y_LIMIT_PORT, Y_LIMIT_PIN);
    signals.min.z = DIGITAL_IN(Z_LIMIT_PORT, Z_LIMIT_PIN);
  #ifdef A_LIMIT_PIN
    signals.min.a = DIGITAL_IN(A_LIMIT_PORT, A_LIMIT_PIN);
  #endif
  #ifdef B_LIMIT_PIN
    signals.min.b = DIGITAL_IN(B_LIMIT_PORT, B_LIMIT_PIN);
  #endif
  #ifdef C_LIMIT_PIN
    signals.min.c = DIGITAL_IN(C_LIMIT_PORT, C_LIMIT_PIN);
  #endif
#elif LIMIT_INMODE == GPIO_MAP
    uint32_t bits = LIMIT_PORT->IDR;
    signals.min.x = !!(bits & X_LIMIT_BIT);
    signals.min.y = !!(bits & Y_LIMIT_BIT);
    signals.min.z = !!(bits & Z_LIMIT_BIT);
  #ifdef A_LIMIT_PIN
    signals.min.a = !!(bits & A_LIMIT_BIT);
  #endif
  #ifdef B_LIMIT_PIN
    signals.min.b = !!(bits & B_LIMIT_BIT);
  #endif
  #ifdef C_LIMIT_PIN
    signals.min.c = !!(bits & C_LIMIT_BIT);
  #endif
#else
    signals.min.value = (uint8_t)((LIMIT_PORT->IDR & LIMIT_MASK) >> LIMIT_INMODE);
#endif

#ifdef X2_LIMIT_PIN
    signals.min2.x = DIGITAL_IN(X2_LIMIT_PORT, X2_LIMIT_PIN);
#endif
#ifdef Y2_LIMIT_PIN
    signals.min2.y = DIGITAL_IN(Y2_LIMIT_PORT, Y2_LIMIT_PIN);
#endif
#ifdef Z2_LIMIT_PIN
    signals.min2.z = DIGITAL_IN(Z2_LIMIT_PORT, Z2_LIMIT_PIN);
#endif

#ifdef X_LIMIT_PIN_MAX
    signals.max.x = DIGITAL_IN(X_LIMIT_PORT_MAX, X_LIMIT_PIN_MAX);
#endif
#ifdef Y_LIMIT_PIN_MAX
    signals.max.y = DIGITAL_IN(Y_LIMIT_PORT_MAX, Y_LIMIT_PIN_MAX);
#endif
#ifdef Z_LIMIT_PIN_MAX
    signals.max.z = DIGITAL_IN(Z_LIMIT_PORT_MAX, Z_LIMIT_PIN_MAX);
#endif

    if (settings.limits.invert.mask) {
        signals.min.value ^= settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
        signals.min2.mask ^= settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
        signals.max.value ^= settings.limits.invert.mask;
#endif
    }

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.mask = settings.control_invert.mask;

#if CONTROL_INMODE == GPIO_BITBAND
#if ESTOP_ENABLE
    signals.e_stop = DIGITAL_IN(RESET_PORT, RESET_PIN);
#else
    signals.reset = DIGITAL_IN(RESET_PORT, RESET_PIN);
#endif
    signals.feed_hold = DIGITAL_IN(FEED_HOLD_PORT, FEED_HOLD_PIN);
    signals.cycle_start = DIGITAL_IN(CYCLE_START_PORT, CYCLE_START_PIN);
  #ifdef SAFETY_DOOR_PIN
    signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PORT, SAFETY_DOOR_PIN);
  #endif
#elif CONTROL_INMODE == GPIO_MAP
    uint32_t bits = CONTROL_PORT->IDR;
#if ESTOP_ENABLE
    signals.e_stop = !!(bits & RESET_BIT);
#else
    signals.reset = !!(bits & RESET_BIT);
#endif
    signals.feed_hold = !!(bits & FEED_HOLD_BIT);
    signals.cycle_start = !!(bits & CYCLE_START_BIT);
  #ifdef SAFETY_DOOR_PIN
    signals.safety_door_ajar = !!(bits & SAFETY_DOOR_BIT);
  #endif
#else
    signals.value = (uint8_t)((CONTROL_PORT->IDR & CONTROL_MASK) >> CONTROL_INMODE);
  #ifndef SAFETY_DOOR_PIN
    signals.safety_door_ajar = settings.control_invert.safety_door_ajar;
  #endif
  #if ESTOP_ENABLE
    signals.e_stop = signals.reset;
    signals.reset = settings.control_invert.reset;
  #endif
#endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

    return signals;
}

#ifdef PROBE_PIN

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    probe.triggered = Off;
    probe.is_probing = probing;
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    state.triggered = DIGITAL_IN(PROBE_PORT, PROBE_PIN) ^ probe.inverted;

    return state;
}

#endif

#ifdef DRIVER_SPINDLE

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (void)
{
#ifdef SPINDLE_ENABLE_PIN
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, settings.spindle.invert.on);
#endif
}

inline static void spindle_on (void)
{
#ifdef SPINDLE_ENABLE_PIN
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, !settings.spindle.invert.on);
#endif
#if SPINDLE_SYNC_ENABLE
    spindleDataReset();
#endif
}

#ifdef SPINDLE_DIRECTION_PIN
inline static void spindle_dir (bool ccw)
{
    DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, ccw ^ settings.spindle.invert.ccw);
}
#endif

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{
    if (!state.on)
        spindle_off();
    else {
#ifdef SPINDLE_DIRECTION_PIN
        spindle_dir(state.ccw);
#endif
        spindle_on();
    }
}

// Variable spindle control functions

#ifdef SPINDLE_PWM_TIMER_N

// Sets spindle speed
static void spindle_set_speed (uint_fast16_t pwm_value)
{
    if (pwm_value == spindle_pwm.off_value) {
        pwmEnabled = false;
        if(settings.spindle.flags.enable_rpm_controlled)
            spindle_off();
        if(spindle_pwm.always_on) {
            SPINDLE_PWM_TIMER_CCR = spindle_pwm.off_value;
#if SPINDLE_PWM_TIMER_N == 1
            SPINDLE_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
#endif
            SPINDLE_PWM_TIMER_CCR = pwm_value;
        } else
#if SPINDLE_PWM_TIMER_N == 1
            SPINDLE_PWM_TIMER->BDTR &= ~TIM_BDTR_MOE; // Set PWM output low
#else
            SPINDLE_PWM_TIMER_CCR = 0;
#endif
    } else {
        if(!pwmEnabled) {
            spindle_on();
            pwmEnabled = true;
        }
        SPINDLE_PWM_TIMER_CCR = pwm_value;
#if SPINDLE_PWM_TIMER_N == 1
        SPINDLE_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
#endif
    }
}

static uint_fast16_t spindleGetPWM (float rpm)
{
    return spindle_compute_pwm_value(&spindle_pwm, rpm, false);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_state_t state, float rpm)
{
#ifdef SPINDLE_DIRECTION_PIN
    if(state.on)
        spindle_dir(state.ccw);
#endif
    if(!settings.spindle.flags.enable_rpm_controlled) {
        if(state.on)
            spindle_on();
        else
            spindle_off();
    }

    spindle_set_speed(state.on ? spindle_compute_pwm_value(&spindle_pwm, rpm, false) : spindle_pwm.off_value);

#if SPINDLE_SYNC_ENABLE
    if(settings.spindle.at_speed_tolerance > 0.0f) {
        float tolerance = rpm * settings.spindle.at_speed_tolerance / 100.0f;
        spindle_data.rpm_low_limit = rpm - tolerance;
        spindle_data.rpm_high_limit = rpm + tolerance;
    }
    spindle_data.rpm_programmed = spindle_data.rpm = rpm;
#endif
}

bool spindleConfig (void)
{
    static spindle_settings_t spindle = {0};

    if(hal.spindle.rpm_max > 0.0f && memcmp(&spindle, &settings.spindle, sizeof(spindle_settings_t))) {

        RCC_ClkInitTypeDef clock;
        uint32_t latency, prescaler = settings.spindle.pwm_freq > 4000.0f ? 1 : (settings.spindle.pwm_freq > 200.0f ? 12 : 25);

        HAL_RCC_GetClockConfig(&clock, &latency);
        memcpy(&spindle, &settings.spindle, sizeof(spindle_settings_t));

    #if SPINDLE_PWM_TIMER_N == 1
        if((hal.spindle.cap.variable = !settings.spindle.flags.pwm_disable && spindle_precompute_pwm_values(&spindle_pwm, (HAL_RCC_GetPCLK2Freq() * (clock.APB2CLKDivider == 0 ? 1 : 2)) / prescaler))) {
    #else
        if((hal.spindle.cap.variable = !settings.spindle.flags.pwm_disable && spindle_precompute_pwm_values(&spindle_pwm, (HAL_RCC_GetPCLK1Freq() * (clock.APB1CLKDivider == 0 ? 1 : 2)) / prescaler))) {
    #endif

            hal.spindle.set_state = spindleSetStateVariable;

            SPINDLE_PWM_TIMER->CR1 &= ~TIM_CR1_CEN;

            TIM_Base_InitTypeDef timerInitStructure = {
                .Prescaler = prescaler - 1,
                .CounterMode = TIM_COUNTERMODE_UP,
                .Period = spindle_pwm.period - 1,
                .ClockDivision = TIM_CLOCKDIVISION_DIV1,
                .RepetitionCounter = 0
            };

            TIM_Base_SetConfig(SPINDLE_PWM_TIMER, &timerInitStructure);

            SPINDLE_PWM_TIMER->CCER &= ~SPINDLE_PWM_CCER_EN;
            SPINDLE_PWM_TIMER_CCMR &= ~SPINDLE_PWM_CCMR_OCM_CLR;
            SPINDLE_PWM_TIMER_CCMR |= SPINDLE_PWM_CCMR_OCM_SET;
            SPINDLE_PWM_TIMER_CCR = 0;
    #if SPINDLE_PWM_TIMER_N == 1
            SPINDLE_PWM_TIMER->BDTR |= TIM_BDTR_OSSR|TIM_BDTR_OSSI;
    #endif
            if(settings.spindle.invert.pwm) {
                SPINDLE_PWM_TIMER->CCER |= SPINDLE_PWM_CCER_POL;
                SPINDLE_PWM_TIMER->CR2 |= SPINDLE_PWM_CR2_OIS;
            } else {
                SPINDLE_PWM_TIMER->CCER &= ~SPINDLE_PWM_CCER_POL;
                SPINDLE_PWM_TIMER->CR2 &= ~SPINDLE_PWM_CR2_OIS;
            }
            SPINDLE_PWM_TIMER->CCER |= SPINDLE_PWM_CCER_EN;
            SPINDLE_PWM_TIMER->CR1 |= TIM_CR1_CEN;

        } else {
            if(pwmEnabled)
                hal.spindle.set_state((spindle_state_t){0}, 0.0f);

            hal.spindle.set_state = spindleSetState;
        }
    }

    spindle_update_caps(hal.spindle.cap.variable ? &spindle_pwm : NULL);

#if SPINDLE_SYNC_ENABLE
    hal.spindle.cap.at_speed = hal.spindle.get_data == spindleGetData;
#endif

    return true;
}

#if PPI_ENABLE

static void spindlePulseOn (uint_fast16_t pulse_length)
{
    PPI_TIMER->ARR = pulse_length;
    PPI_TIMER->EGR = TIM_EGR_UG;
    PPI_TIMER->CR1 |= TIM_CR1_CEN;
    spindle_on();
}

#endif

#if SPINDLE_SYNC_ENABLE

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    bool stopped;
    uint32_t pulse_length, rpm_timer_delta;

    spindle_encoder_counter_t encoder;

//    while(spindle_encoder.spin_lock);

    __disable_irq();

    memcpy(&encoder, &spindle_encoder.counter, sizeof(spindle_encoder_counter_t));

    pulse_length = spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;
    rpm_timer_delta = RPM_TIMER_COUNT - spindle_encoder.timer.last_pulse;

    // if 16 bit RPM timer and RPM_TIMER_COUNT < spindle_encoder.timer.last_pulse then what?

    __enable_irq();

    // If no spindle pulses during last 250 ms assume RPM is 0
    if((stopped = ((pulse_length == 0) || (rpm_timer_delta > spindle_encoder.maximum_tt)))) {
        spindle_data.rpm = 0.0f;
        rpm_timer_delta = (uint16_t)(((uint16_t)RPM_COUNTER->CNT - (uint16_t)encoder.last_count)) * pulse_length;
    }

    switch(request) {

        case SpindleData_Counters:
            spindle_data.index_count = encoder.index_count;
            spindle_data.pulse_count = encoder.pulse_count + (uint32_t)((uint16_t)RPM_COUNTER->CNT - (uint16_t)encoder.last_count);
            spindle_data.error_count = spindle_encoder.error_count;
            break;

        case SpindleData_RPM:
            if(!stopped)
                spindle_data.rpm = spindle_encoder.rpm_factor / (float)pulse_length;
            break;

        case SpindleData_AngularPosition:
            spindle_data.angular_position = (float)encoder.index_count +
                    ((float)((uint16_t)encoder.last_count - (uint16_t)encoder.last_index) +
                              (pulse_length == 0 ? 0.0f : (float)rpm_timer_delta / (float)pulse_length)) *
                                spindle_encoder.pulse_distance;
            break;
    }

    return &spindle_data;
}

static void spindleDataReset (void)
{
    while(spindleLock);

    uint32_t timeout = uwTick + 1000; // 1 second

    uint32_t index_count = spindle_encoder.counter.index_count + 2;
    if(spindleGetData(SpindleData_RPM)->rpm > 0.0f) { // wait for index pulse if running

        while(index_count != spindle_encoder.counter.index_count && uwTick <= timeout);

//        if(uwTick > timeout)
//            alarm?
    }


    RPM_TIMER->EGR |= TIM_EGR_UG; // Reload RPM timer
    RPM_COUNTER->CR1 &= ~TIM_CR1_CEN;

#if RPM_COUNTER_N == 2
    rpm_timer_ovf = 0;
#endif

    spindle_encoder.timer.last_index =
    spindle_encoder.timer.last_index = RPM_TIMER_COUNT;

    spindle_encoder.timer.pulse_length =
    spindle_encoder.counter.last_count =
    spindle_encoder.counter.last_index =
    spindle_encoder.counter.pulse_count =
    spindle_encoder.counter.index_count =
    spindle_encoder.error_count = 0;

    RPM_COUNTER->EGR |= TIM_EGR_UG;
    RPM_COUNTER->CCR1 = spindle_encoder.tics_per_irq;
    RPM_COUNTER->CR1 |= TIM_CR1_CEN;
}

#endif // SPINDLE_SYNC_ENABLE

#endif // SPINDLE_PWM_TIMER_N

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {settings.spindle.invert.mask};

#ifdef SPINDLE_ENABLE_PIN
    state.on = DIGITAL_IN(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN);
#endif
#ifdef SPINDLE_DIRECTION_PIN
    state.ccw = DIGITAL_IN(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN);
#endif
    state.value ^= settings.spindle.invert.mask;

#if SPINDLE_SYNC_ENABLE
    float rpm = spindleGetData(SpindleData_RPM)->rpm;
    state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (rpm >= spindle_data.rpm_low_limit && rpm <= spindle_data.rpm_high_limit);
    state.encoder_error = spindle_encoder.error_count > 0;
#endif

    return state;
}

#endif // DRIVER_SPINDLE

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;
    DIGITAL_OUT(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, mode.flood);
#ifdef COOLANT_MIST_PIN
    DIGITAL_OUT(COOLANT_MIST_PORT, COOLANT_MIST_PIN, mode.mist);
#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = (coolant_state_t){settings.coolant_invert.mask};

    state.flood = DIGITAL_IN(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN);
#ifdef COOLANT_MIST_PIN
    state.mist  = DIGITAL_IN(COOLANT_MIST_PORT, COOLANT_MIST_PIN);
#endif
    state.value ^= settings.coolant_invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    *ptr |= bits;
    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    __enable_irq();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    __enable_irq();
    return prev;
}

#if MPG_MODE == 1

static void mpg_select (sys_state_t state)
{
    stream_mpg_enable(DIGITAL_IN(MPG_MODE_PORT, MPG_MODE_PIN) == 0);
}

static void mpg_enable (sys_state_t state)
{
    if(sys.mpg_mode != (DIGITAL_IN(MPG_MODE_PORT, MPG_MODE_PIN) == 0))
        stream_mpg_enable(true);
}

#endif

static uint32_t getElapsedTicks (void)
{
    return uwTick;
}

// Configures peripherals when settings are initialized or changed
void settings_changed (settings_t *settings)
{
#if USE_STEPDIR_MAP
    stepdirmap_init(settings);
#endif

    if(IOInitDone) {

        GPIO_InitTypeDef GPIO_Init = {
            .Speed = GPIO_SPEED_FREQ_HIGH
        };

        stepperSetStepOutputs((axes_signals_t){0});
        stepperSetDirOutputs((axes_signals_t){0});

#ifdef SQUARING_ENABLED
        hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif

#ifdef SPINDLE_PWM_TIMER_N
        if(hal.spindle.config == spindleConfig)
            spindleConfig();
#endif

#if SPINDLE_SYNC_ENABLE

        if((hal.spindle.get_data = (hal.spindle.cap.at_speed = settings->spindle.ppr > 0) ? spindleGetData : NULL) &&
             (spindle_encoder.ppr != settings->spindle.ppr || pidf_config_changed(&spindle_tracker.pid, &settings->position.pid))) {

            hal.spindle.reset_data = spindleDataReset;
            hal.spindle.set_state((spindle_state_t){0}, 0.0f);

            pidf_init(&spindle_tracker.pid, &settings->position.pid);

#if RPM_COUNTER_N == 2
            float timer_resolution = 1.0f / 50000.0f; // 50 us resolution
#else
            float timer_resolution = 1.0f / 1000000.0f; // 1 us resolution
#endif
            spindle_tracker.min_cycles_per_tick = hal.f_step_timer / (uint32_t)(settings->axis[Z_AXIS].max_rate * settings->axis[Z_AXIS].steps_per_mm / 60.0f);
            spindle_encoder.ppr = settings->spindle.ppr;
            spindle_encoder.tics_per_irq = max(1, spindle_encoder.ppr / 32);
            spindle_encoder.pulse_distance = 1.0f / spindle_encoder.ppr;
            spindle_encoder.maximum_tt = (uint32_t)(2.0f / timer_resolution) / spindle_encoder.tics_per_irq;
            spindle_encoder.rpm_factor = 60.0f / ((timer_resolution * (float)spindle_encoder.ppr));
            spindleDataReset();
        }

#endif

        pulse_length = (uint32_t)(10.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            pulse_delay = (uint32_t)(10.0f * (settings->steppers.pulse_delay_microseconds - 1.0f));
            if(pulse_delay < 2)
                pulse_delay = 2;
            else if(pulse_delay == pulse_length)
                pulse_delay++;
            hal.stepper.pulse_start = &stepperPulseStartDelayed;
        } else {
            pulse_delay = 0;
            hal.stepper.pulse_start = &stepperPulseStart;
        }

        PULSE_TIMER->ARR = pulse_length;
        PULSE_TIMER->EGR = TIM_EGR_UG;

        /*************************
         *  Control pins config  *
         *************************/

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<0)
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<1)
        HAL_NVIC_DisableIRQ(EXTI1_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<2)
        HAL_NVIC_DisableIRQ(EXTI2_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<3)
        HAL_NVIC_DisableIRQ(EXTI3_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<4)
        HAL_NVIC_DisableIRQ(EXTI4_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & 0x03E0
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & 0xFC00
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
#endif

        bool pullup;
        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
        input_signal_t *input;

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        do {

            pullup = false;
            input = &inputpin[--i];
            if(input->group != PinGroup_AuxInput) {
                input->irq_mode = IRQ_Mode_None;
                input->bit = 1 << input->pin;
            }

            switch(input->id) {

                case Input_Reset:
                    pullup = !settings->control_disable_pullup.reset;
                    input->irq_mode = control_fei.reset ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_FeedHold:
                    pullup = !settings->control_disable_pullup.feed_hold;
                    input->irq_mode = control_fei.feed_hold ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_CycleStart:
                    pullup = !settings->control_disable_pullup.cycle_start;
                    input->irq_mode = control_fei.cycle_start ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_SafetyDoor:
                    pullup = !settings->control_disable_pullup.safety_door_ajar;
                    input->irq_mode = control_fei.safety_door_ajar ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_Probe:
                    pullup = hal.driver_cap.probe_pull_up;
                    break;

                case Input_LimitX:
                case Input_LimitX_2:
                case Input_LimitX_Max:
                    pullup = !settings->limits.disable_pullup.x;
                    input->irq_mode = limit_fei.x ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitY:
                case Input_LimitY_2:
                case Input_LimitY_Max:
                    pullup = !settings->limits.disable_pullup.y;
                    input->irq_mode = limit_fei.y ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitZ:
                case Input_LimitZ_2:
                case Input_LimitZ_Max:
                    pullup = !settings->limits.disable_pullup.z;
                    input->irq_mode = limit_fei.z ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitA:
                case Input_LimitA_Max:
                    pullup = !settings->limits.disable_pullup.a;
                    input->irq_mode = limit_fei.a ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitB:
                case Input_LimitB_Max:
                    pullup = !settings->limits.disable_pullup.b;
                    input->irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitC:
                case Input_LimitC_Max:
                    pullup = !settings->limits.disable_pullup.c;
                    input->irq_mode = limit_fei.c ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_ModeSelect:
                    pullup = true;
                    input->irq_mode = IRQ_Mode_Change;
                    break;

                case Input_KeypadStrobe:
                    pullup = true;
                    input->irq_mode = IRQ_Mode_Change;
                    break;

                case Input_SpindleIndex:
                    pullup = true;
                    input->irq_mode = IRQ_Mode_Falling;
                    break;

                default:
                    break;
            }

            if(input->group == PinGroup_AuxInput) {
                pullup = true;
                if(input->cap.irq_mode != IRQ_Mode_None) {
                    aux_irq |= input->bit;
                    // Map interrupt to pin
                    uint32_t extireg = SYSCFG->EXTICR[input->pin >> 2] & ~(0b1111 << ((input->pin & 0b11) << 2));
                    extireg |= ((uint32_t)(GPIO_GET_INDEX(input->port)) << ((input->pin & 0b11) << 2));
                    SYSCFG->EXTICR[input->pin >> 2] = extireg;
                }
            }

            GPIO_Init.Pin = 1 << input->pin;
            GPIO_Init.Pull = pullup ? GPIO_PULLUP : GPIO_NOPULL;

            switch(input->irq_mode) {
                case IRQ_Mode_Rising:
                    GPIO_Init.Mode = GPIO_MODE_IT_RISING;
                    break;
                case IRQ_Mode_Falling:
                    GPIO_Init.Mode = GPIO_MODE_IT_FALLING;
                    break;
                case IRQ_Mode_Change:
                    GPIO_Init.Mode = GPIO_MODE_IT_RISING_FALLING;
                    break;
                default:
                    GPIO_Init.Mode = GPIO_MODE_INPUT;
                    break;
            }
            HAL_GPIO_Init(input->port, &GPIO_Init);

            input->debounce = false;

        } while(i);

        uint32_t irq_mask = DRIVER_IRQMASK|aux_irq;

        __HAL_GPIO_EXTI_CLEAR_IT(irq_mask);

        if(irq_mask & (1<<0)) {
            HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(EXTI0_IRQn);
        }
        if(irq_mask & (1<<1)) {
            HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(EXTI1_IRQn);
        }
        if(irq_mask & (1<<2)) {
            HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(EXTI2_IRQn);
        }
        if(irq_mask & (1<<3)) {
            HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(EXTI3_IRQn);
        }
        if(irq_mask & (1<<4)) {
            HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(EXTI4_IRQn);
        }
        if(irq_mask & 0x03E0) {
            HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        }
        if(irq_mask & 0xFC00) {
            HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
            HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        }

        hal.limits.enable(settings->limits.flags.hard_enabled, false);
    }
}

static char *port2char (GPIO_TypeDef *port)
{
    static char name[3] = "P?";

    name[1] = 'A' + GPIO_GET_INDEX(port);

    return name;
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {0};
    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.pin = inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        pin.port = low_level ? (void *)inputpin[i].port : (void *)port2char(inputpin[i].port);
        pin.description = inputpin[i].description;
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;

        pin_info(&pin, data);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.pin = outputpin[i].pin;
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.port = low_level ? (void *)outputpin[i].port : (void *)port2char(outputpin[i].port);
        pin.description = outputpin[i].description;

        pin_info(&pin, data);
    };

    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.port = low_level ? ppin->pin.port : (void *)port2char(ppin->pin.port);
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description;

        pin_info(&pin, data);

        ppin = ppin->next;
    } while(ppin);
}

void registerPeriphPin (const periph_pin_t *pin)
{
    periph_signal_t *add_pin = malloc(sizeof(periph_signal_t));

    if(!add_pin)
        return;

    memcpy(&add_pin->pin, pin, sizeof(periph_pin_t));
    add_pin->next = NULL;

    if(periph_pins == NULL) {
        periph_pins = add_pin;
    } else {
        periph_signal_t *last = periph_pins;
        while(last->next)
            last = last->next;
        last->next = add_pin;
    }
}

void setPeriphPinDescription (const pin_function_t function, const pin_group_t group, const char *description)
{
    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        if(ppin->pin.function == function && ppin->pin.group == group) {
            ppin->pin.description = description;
            ppin = NULL;
        } else
            ppin = ppin->next;
    } while(ppin);
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    //    Interrupt_disableSleepOnIsrExit();

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM9_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_Init = {
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Mode = GPIO_MODE_OUTPUT_PP
    };

    /*************************
     *  Output signals init  *
     *************************/

    uint32_t i;

    // Switch on stepper driver power before enabling other output pins
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        if(outputpin[i].group == PinGroup_StepperPower) {
            GPIO_Init.Pin = 1 << outputpin[i].pin;
            GPIO_Init.Mode = outputpin[i].mode.open_drain ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;
            HAL_GPIO_Init(outputpin[i].port, &GPIO_Init);
            DIGITAL_OUT(outputpin[i].port, outputpin[i].pin, 1);
        }
    }

    hal.delay_ms(100, NULL);

    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        if(outputpin[i].group != PinGroup_StepperPower) {
            GPIO_Init.Pin = 1 << outputpin[i].pin;
            GPIO_Init.Mode = outputpin[i].mode.open_drain ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;
            HAL_GPIO_Init(outputpin[i].port, &GPIO_Init);

            if(outputpin[i].group == PinGroup_MotorChipSelect || outputpin[i].group == PinGroup_MotorUART)
                DIGITAL_OUT(outputpin[i].port, outputpin[i].pin, 1);
        }
    }

    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;

 // Stepper init

    STEPPER_TIMER_CLOCK_ENA();
    STEPPER_TIMER->CR1 &= ~TIM_CR1_CEN;
    STEPPER_TIMER->SR &= ~TIM_SR_UIF;
    STEPPER_TIMER->CNT = 0;
    STEPPER_TIMER->DIER |= TIM_DIER_UIE;

    HAL_NVIC_SetPriority(STEPPER_TIMER_IRQn, 1, 0);
    NVIC_EnableIRQ(STEPPER_TIMER_IRQn);

 // Single-shot 100 ns per tick

    PULSE_TIMER_CLOCK_ENA();
    PULSE_TIMER->CR1 |= TIM_CR1_OPM|TIM_CR1_DIR|TIM_CR1_CKD_1|TIM_CR1_ARPE|TIM_CR1_URS;
    PULSE_TIMER->PSC = hal.f_step_timer / 10000000UL - 1;
    PULSE_TIMER->SR &= ~(TIM_SR_UIF|TIM_SR_CC1IF);
    PULSE_TIMER->CNT = 0;
    PULSE_TIMER->DIER |= TIM_DIER_UIE;

    HAL_NVIC_SetPriority(PULSE_TIMER_IRQn, 0, 1);
    NVIC_EnableIRQ(PULSE_TIMER_IRQn);

 // Control pins init

    if(hal.driver_cap.software_debounce) {
        // Single-shot 0.1 ms per tick
        DEBOUNCE_TIMER_CLOCK_ENA();
        DEBOUNCE_TIMER->CR1 |= TIM_CR1_OPM|TIM_CR1_DIR|TIM_CR1_CKD_1|TIM_CR1_ARPE|TIM_CR1_URS;
        DEBOUNCE_TIMER->PSC = hal.f_step_timer / 10000UL - 1;
        DEBOUNCE_TIMER->SR &= ~TIM_SR_UIF;
        DEBOUNCE_TIMER->ARR = 400; // 40 ms timeout
        DEBOUNCE_TIMER->DIER |= TIM_DIER_UIE;

        HAL_NVIC_EnableIRQ(DEBOUNCE_TIMER_IRQn); // Enable debounce interrupt
    }

  // Spindle init

#ifdef SPINDLE_PWM_TIMER_N

    SPINDLE_PWM_CLOCK_ENA();

    GPIO_Init.Pin = (1 << SPINDLE_PWM_PIN);
    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Alternate = SPINDLE_PWM_AF;
    HAL_GPIO_Init(SPINDLE_PWM_PORT, &GPIO_Init);

    static const periph_pin_t pwm = {
        .function = Output_SpindlePWM,
        .group = PinGroup_SpindlePWM,
        .port = SPINDLE_PWM_PORT,
        .pin = SPINDLE_PWM_PIN,
        .mode = { .mask = PINMODE_OUTPUT }
    };

    hal.periph_port.register_pin(&pwm);

#endif

#if SDCARD_ENABLE

    DIGITAL_OUT(SD_CS_PORT, SD_CS_PIN, 1);

    sdcard_init();

#endif

#if PPI_ENABLE

    // Single-shot 1 us per tick
    PPI_TIMER_CLOCK_ENA();
    PPI_TIMER->CR1 |= TIM_CR1_OPM|TIM_CR1_DIR|TIM_CR1_CKD_1|TIM_CR1_ARPE|TIM_CR1_URS;
    PPI_TIMER->PSC = hal.f_step_timer / 1000000UL - 1;
    PPI_TIMER->SR &= ~(TIM_SR_UIF|TIM_SR_CC1IF);
    PPI_TIMER->CNT = 0;
    PPI_TIMER->DIER |= TIM_DIER_UIE;

    HAL_NVIC_EnableIRQ(PPI_TIMER_IRQn);

#endif

#if SPINDLE_SYNC_ENABLE

    RPM_TIMER_CLOCK_ENA();
    RPM_TIMER->CR1 = TIM_CR1_CKD_1|TIM_CR1_URS;
#if RPM_COUNTER_N == 2
    RPM_TIMER->PSC = hal.f_step_timer / 50000UL - 1;
    RPM_TIMER->DIER |= TIM_DIER_UIE;
#else
    RPM_TIMER->PSC = hal.f_step_timer / 1000000UL - 1;
#endif
    RPM_TIMER->CR1 |= TIM_CR1_CEN;

#if RPM_COUNTER_N == 2
    HAL_NVIC_EnableIRQ(RPM_TIMER_IRQn);
    HAL_NVIC_SetPriority(RPM_COUNTER_IRQn, 1, 1);
#endif

//    RPM_COUNTER->SMCR = TIM_SMCR_SMS_0|TIM_SMCR_SMS_1|TIM_SMCR_SMS_2|TIM_SMCR_ETF_2|TIM_SMCR_ETF_3|TIM_SMCR_TS_0|TIM_SMCR_TS_1|TIM_SMCR_TS_2;
    RPM_COUNTER_CLOCK_ENA();
    RPM_COUNTER->SMCR = TIM_SMCR_ECE;
    RPM_COUNTER->PSC = 0;
    RPM_COUNTER->ARR = 65535;
    RPM_COUNTER->DIER = TIM_DIER_CC1IE;

    HAL_NVIC_EnableIRQ(RPM_COUNTER_IRQn);

    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    GPIO_Init.Pin = SPINDLE_PULSE_BIT;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
#if RPM_COUNTER_N == 2
    GPIO_Init.Alternate = GPIO_AF1_TIM2;
#else // RPM_COUNTER_N = 2
    GPIO_Init.Alternate = GPIO_AF2_TIM3;
#endif
    HAL_GPIO_Init(SPINDLE_PULSE_PORT, &GPIO_Init);

    static const periph_pin_t ssp = {
        .function = Input_SpindlePulse,
        .group = PinGroup_SpindlePulse,
        .port = SPINDLE_PULSE_PORT,
        .pin = SPINDLE_PULSE_PIN,
        .mode = { .mask = PINMODE_NONE }
    };

    hal.periph_port.register_pin(&ssp);

#endif

    IOInitDone = settings->version == 21;

    hal.settings_changed(settings);

#if PPI_ENABLE
    ppi_init();
#endif

#if ETHERNET_ENABLE
    enet_start();
#endif

    return IOInitDone;
}

#if RTC_ENABLE

static bool rtc_started = false;

static RTC_HandleTypeDef hrtc = {
    .Instance = RTC,
    .Init.HourFormat = RTC_HOURFORMAT_24,
    .Init.AsynchPrediv = 127,
    .Init.SynchPrediv = 255,
    .Init.OutPut = RTC_OUTPUT_DISABLE,
    .Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH,
    .Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN
};

static bool set_rtc_time (struct tm *time)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    if(!rtc_started) {

        RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {
            .PeriphClockSelection = RCC_PERIPHCLK_RTC,
            .RTCClockSelection = RCC_RTCCLKSOURCE_LSE
        };

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) == HAL_OK) {
            __HAL_RCC_RTC_ENABLE();
            rtc_started = HAL_RTC_Init(&hrtc) == HAL_OK;
        }
    }

    if(rtc_started) {

        sTime.Hours = time->tm_hour;
        sTime.Minutes = time->tm_min;
        sTime.Seconds = time->tm_sec;
        sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        sTime.StoreOperation = RTC_STOREOPERATION_RESET;
        if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK) {
            sDate.WeekDay = RTC_WEEKDAY_MONDAY;
            sDate.Month = time->tm_mon + 1;
            sDate.Date = time->tm_mday;
            sDate.Year = time->tm_year - 100;
            HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
        }
    }

    return rtc_started;
}

static bool get_rtc_time (struct tm *time)
{
    bool ok = false;

    if(rtc_started) {

        RTC_TimeTypeDef sTime = {0};
        RTC_DateTypeDef sDate = {0};

        if((ok = HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK &&
                  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) == HAL_OK)) {

            time->tm_hour = sTime.Hours;
            time->tm_min = sTime.Minutes;
            time->tm_sec = sTime.Seconds;
            time->tm_mon = sDate.Month - 1;
            time->tm_mday = sDate.Date;
            time->tm_year = sDate.Year + 100;
        }
    }

    return ok;
}

#endif

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: grblHAL is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void)
{
#ifdef HAS_BOOTLOADER
    extern uint8_t _FLASH_VectorTable;
    __disable_irq();
    SCB->VTOR = (uint32_t)&_FLASH_VectorTable;
    __DSB();
    __enable_irq();
#endif

#if MPG_MODE == 1

    // Drive MPG mode input pin low until setup complete

    GPIO_InitTypeDef GPIO_Init = {
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pin = 1 << MPG_MODE_PIN,
        .Mode = GPIO_MODE_OUTPUT_PP
    };

    HAL_GPIO_Init(MPG_MODE_PORT, &GPIO_Init);

    DIGITAL_OUT(MPG_MODE_PORT, MPG_MODE_PIN, 1);

#endif

#ifdef STM32F446xx
    hal.info = "STM32F446";
#elif defined(STM32F411xE)
    hal.info = "STM32F411";
#elif defined(STM32F407xx)
    hal.info = "STM32F407";
#else
    hal.info = "STM32F401CC";
#endif
    hal.driver_version = "221014";
    hal.driver_url = GRBL_URL "/STM32F4xx";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
#ifdef BOARD_URL
    hal.board = BOARD_URL;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = HAL_RCC_GetPCLK2Freq();
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = &driver_delay;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
    hal.stepper.motor_iterator = motor_iterator;
#ifdef GANGING_ENABLED
    hal.stepper.get_ganged = getGangedAxes;
#endif
#ifdef SQUARING_ENABLED
    hal.stepper.disable_motors = StepperDisableMotors;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

#ifdef PROBE_PIN
    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;
#endif

    hal.control.get_state = systemGetState;

    hal.irq_enable = __enable_irq;
    hal.irq_disable = __disable_irq;
#if I2C_STROBE_ENABLE
    hal.irq_claim = irq_claim;
#endif
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = getElapsedTicks;
    hal.enumerate_pins = enumeratePins;
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;

#if RTC_ENABLE
    hal.rtc.get_datetime = get_rtc_time;
    hal.rtc.set_datetime = set_rtc_time;
#endif

    serialRegisterStreams();

#if USB_SERIAL_CDC
    stream_connect(usbInit());
#elif !defined(UART_INSTANCE)
    stream_connect(serialInit(BAUD_RATE));
#endif

#if I2C_ENABLE
    i2c_init();
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#elif FLASH_ENABLE
    hal.nvs.type = NVS_Flash;
    hal.nvs.memcpy_from_flash = memcpy_from_flash;
    hal.nvs.memcpy_to_flash = memcpy_to_flash;
#else
    hal.nvs.type = NVS_None;
#endif

#ifdef DRIVER_SPINDLE

    static const spindle_ptrs_t spindle = {
 #ifdef SPINDLE_PWM_TIMER_N
        .type = SpindleType_PWM,
        .cap.variable = On,
        .cap.laser = On,
        .cap.pwm_invert = On,
        .config = spindleConfig,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindle_set_speed,
  #if PPI_ENABLE
        .pulse_on = spindlePulseOn,
  #endif
 #else
        .type = SpindleType_Basic,
 #endif
 #ifdef SPINDLE_DIRECTION_PIN
        .cap.direction = On,
 #endif
        .set_state = spindleSetState,
        .get_state = spindleGetState
    };

#ifdef SPINDLE_PWM_TIMER_N
    spindle_register(&spindle, "PWM");
#else
    spindle_register(&spindle, "Basic");
#endif

#endif // DRIVER_SPINDLE

// driver capabilities, used for announcing and negotiating (with the core) driver functionality

#if ESTOP_ENABLE
    hal.signals_cap.e_stop = On;
#endif
#ifdef SAFETY_DOOR_PIN
    hal.signals_cap.safety_door_ajar = On;
#endif

#if SPINDLE_SYNC_ENABLE
    hal.driver_cap.spindle_sync = On;
#endif
#ifdef COOLANT_MIST_PIN
    hal.driver_cap.mist_control = On;
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
#ifdef PROBE_PIN
    hal.driver_cap.probe_pull_up = On;
#endif

    uint32_t i;
    input_signal_t *input;
    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0};

    for(i = 0 ; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        input = &inputpin[i];
        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;
            input->id = (pin_function_t)(Input_Aux0 + aux_inputs.n_pins++);
            input->bit = 1 << input->pin;
            input->cap.pull_mode = PullMode_UpDown;
            input->cap.irq_mode = (DRIVER_IRQMASK & input->bit) ? IRQ_Mode_None : IRQ_Mode_Edges;
        }
    }

    output_signal_t *output;
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            output->id = (pin_function_t)(Output_Aux0 + aux_outputs.n_pins++);
        }
    }

#ifdef HAS_IOPORTS
    ioports_init(&aux_inputs, &aux_outputs);
#endif

#ifdef HAS_BOARD_INIT
    board_init();
#endif

#if defined(UART_INSTANCE) && USB_SERIAL_CDC == 0
    if(!stream_connect_instance(UART_INSTANCE, BAUD_RATE))
        while(true); // Cannot boot if no communication channel is available!
#endif

#if MPG_MODE == 1
  #if KEYPAD_ENABLE == 2
    if((hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL), false, keypad_enqueue_keycode)))
        protocol_enqueue_rt_command(mpg_enable);
  #else
    if((hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL), false, NULL)))
        protocol_enqueue_rt_command(mpg_enable);
  #endif
#elif MPG_MODE == 2
    hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL), false, keypad_enqueue_keycode);
#elif KEYPAD_ENABLE == 2
    stream_open_instance(KEYPAD_STREAM, 115200, keypad_enqueue_keycode);
#endif

#if ETHERNET_ENABLE
    enet_init();
#endif

#include "grbl/plugins_init.h"

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 10;
}

/* interrupt handlers */

// Main stepper driver
void STEPPER_TIMER_IRQHandler (void)
{
    if ((STEPPER_TIMER->SR & TIM_SR_UIF) != 0)                  // check interrupt source
    {
        STEPPER_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag
        hal.stepper.interrupt_callback();
    }
}

/* The Stepper Port Reset Interrupt: This interrupt handles the falling edge of the step
   pulse. This should always trigger before the next general stepper driver interrupt and independently
   finish, if stepper driver interrupts is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is
   added to Grbl.
*/

// This interrupt is used only when STEP_PULSE_DELAY is enabled. Here, the step pulse is
// initiated after the STEP_PULSE_DELAY time period has elapsed. The ISR TIMER2_OVF interrupt
// will then trigger after the appropriate settings.pulse_microseconds, as in normal operation.
// The new timing between direction, step pulse, and step complete events are setup in the
// st_wake_up() routine.

// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
void PULSE_TIMER_IRQHandler (void)
{
    PULSE_TIMER->SR &= ~TIM_SR_UIF;                 // Clear UIF flag

    if (PULSE_TIMER->ARR == pulse_delay)            // Delayed step pulse?
    {
        PULSE_TIMER->ARR = pulse_length;
        stepperSetStepOutputs(next_step_outbits);   // begin step pulse
        PULSE_TIMER->EGR = TIM_EGR_UG;
        PULSE_TIMER->CR1 |= TIM_CR1_CEN;
    } else
        stepperSetStepOutputs((axes_signals_t){0}); // end step pulse
}

// Debounce timer interrupt handler
void DEBOUNCE_TIMER_IRQHandler (void)
{
    DEBOUNCE_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag;

    if(debounce.limits) {
        debounce.limits = Off;
        limit_signals_t state = limitsGetState();
        if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
            hal.limits.interrupt_callback(state);
    }

    if(debounce.door) {
        debounce.door = Off;
        control_signals_t state = systemGetState();
        if(state.safety_door_ajar)
            hal.control.interrupt_callback(state);
    }
}

#if PPI_ENABLE

// PPI timer interrupt handler
void PPI_TIMER_IRQHandler (void)
{
    PPI_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag;

    spindle_off();
}

#endif

#if SPINDLE_SYNC_ENABLE

void RPM_COUNTER_IRQHandler (void)
{
    spindle_encoder.spin_lock = true;

    __disable_irq();
    uint32_t tval = RPM_TIMER_COUNT;
    uint16_t cval = RPM_COUNTER->CNT;
    __enable_irq();

    RPM_COUNTER->SR = ~TIM_SR_CC1IF;
    RPM_COUNTER->CCR1 = (uint16_t)(RPM_COUNTER->CCR1 + spindle_encoder.tics_per_irq);

    spindle_encoder.counter.pulse_count += (uint16_t)(cval - (uint16_t)spindle_encoder.counter.last_count);
    spindle_encoder.counter.last_count = cval;
    spindle_encoder.timer.pulse_length = tval - spindle_encoder.timer.last_pulse;
    spindle_encoder.timer.last_pulse = tval;

    spindle_encoder.spin_lock = false;
}

#if RPM_COUNTER_N == 2

void RPM_TIMER_IRQHandler (void)
{
    RPM_TIMER->SR &= ~TIM_SR_UIF;

    rpm_timer_ovf++;
}

#endif

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<0)

void EXTI0_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<0);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<0)
        hal.control.interrupt_callback(systemGetState());
  #if SAFETY_DOOR_BIT & (1<<0)
        if(hal.driver_cap.software_debounce) {
            debounce.door = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
  #endif
#elif defined(I2C_STROBE_ENABLE) && I2C_STROBE_BIT & (1<<0)
        if(i2c_strobe.callback)
            i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
#elif LIMIT_MASK & (1<<0)
        if(hal.driver_cap.software_debounce) {
            debounce.limits = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
            hal.limits.interrupt_callback(limitsGetState());
#elif AUXINPUT_MASK & (1<<0)
        ioports_event(ifg);
#endif

    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<1)

void EXTI1_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<1);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<1)
  #if SAFETY_DOOR_BIT & (1<<1)
        if(hal.driver_cap.software_debounce) {
            debounce.door = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
  #endif
        hal.control.interrupt_callback(systemGetState());
#elif LIMIT_MASK & (1<<1)
        if(hal.driver_cap.software_debounce) {
            debounce.limits = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
            hal.limits.interrupt_callback(limitsGetState());
#elif AUXINPUT_MASK & (1<<1)
        ioports_event(ifg);
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<2)

void EXTI2_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<2);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<2)
  #if SAFETY_DOOR_BIT & (1<<2)
        if(hal.driver_cap.software_debounce) {
            debounce.door = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
 #endif
        hal.control.interrupt_callback(systemGetState());
#elif LIMIT_MASK & (1<<2)
        if(hal.driver_cap.software_debounce) {
            debounce.limits = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
            hal.limits.interrupt_callback(limitsGetState());
#elif AUXINPUT_MASK & (1<<2)
        ioports_event(ifg);
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<3)

void EXTI3_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<3);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<3)
  #if SAFETY_DOOR_BIT & (1<<3)
        if(hal.driver_cap.software_debounce) {
            debounce.door = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
  #endif
          hal.control.interrupt_callback(systemGetState());
#elif LIMIT_MASK & (1<<3)
        if(hal.driver_cap.software_debounce) {
            debounce.limits = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
            hal.limits.interrupt_callback(limitsGetState());
#elif AUXINPUT_MASK & (1<<3)
        ioports_event(ifg);
#elif SPINDLE_INDEX_PIN == 3
        if(ifg & SPINDLE_INDEX_BIT) {

            if(spindle_encoder.counter.index_count && (uint16_t)(RPM_COUNTER->CNT - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
                spindle_encoder.error_count++;

            spindle_encoder.counter.last_index = RPM_COUNTER->CNT;
            spindle_encoder.timer.last_index = RPM_TIMER_COUNT;
            spindle_encoder.counter.index_count++;
        }
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<4)

void EXTI4_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<4);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<4)
  #if SAFETY_DOOR_BIT & (1<<4)
        if(hal.driver_cap.software_debounce) {
            debounce.door = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
  #endif
        hal.control.interrupt_callback(systemGetState());
#elif LIMIT_MASK & (1<<4)
        if(hal.driver_cap.software_debounce) {
            debounce.limits = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
            hal.limits.interrupt_callback(limitsGetState());
#elif AUXINPUT_MASK & (1<<4)
        ioports_event(ifg);
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK|MPG_MODE_BIT) & (0x03E0)

void EXTI9_5_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(0x03E0);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);

#if CONTROL_MASK & 0x03E0
        if(ifg & CONTROL_MASK) {
  #if SAFETY_DOOR_BIT & 0x03E0
            if((ifg & SAFETY_DOOR_BIT) && hal.driver_cap.software_debounce) {
                debounce.door = On;
                DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
                DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
            } else
  #endif
                hal.control.interrupt_callback(systemGetState());
        }
#endif
#if LIMIT_MASK & 0x03E0
        if(ifg & LIMIT_MASK) {
            if(hal.driver_cap.software_debounce) {
                debounce.limits = On;
                DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
                DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
            } else
                hal.limits.interrupt_callback(limitsGetState());
        }
#endif
#if I2C_STROBE_ENABLE && (I2C_STROBE_BIT & 0x03E0)
        if((ifg & I2C_STROBE_BIT) && i2c_strobe.callback)
            i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
#endif
#if MPG_MODE_BIT && (MPG_MODE_BIT & 0x03E0)
        if(ifg & MPG_MODE_BIT)
            protocol_enqueue_rt_command(mpg_select);
#endif
#if AUXINPUT_MASK & 0x03E0
        if(ifg & aux_irq)
            ioports_event(ifg & aux_irq);
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (0xFC00)

void EXTI15_10_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(0xFC00);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);

#ifdef SPINDLE_INDEX_PORT
        if(ifg & SPINDLE_INDEX_BIT) {

            if(spindle_encoder.counter.index_count && (uint16_t)(RPM_COUNTER->CNT - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
                spindle_encoder.error_count++;

            spindle_encoder.counter.last_index = RPM_COUNTER->CNT;
            spindle_encoder.timer.last_index = RPM_TIMER_COUNT;
            spindle_encoder.counter.index_count++;
        }
#endif
#if CONTROL_MASK & 0xFC00
        if(ifg & CONTROL_MASK) {
  #if SAFETY_DOOR_BIT & 0xFC00
            if((ifg & SAFETY_DOOR_BIT) && hal.driver_cap.software_debounce) {
                debounce.door = On;
                DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
                DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
            } else
  #endif
                hal.control.interrupt_callback(systemGetState());
        }
#endif
#if LIMIT_MASK & 0xFC00
        if(ifg & LIMIT_MASK) {
            if(hal.driver_cap.software_debounce) {
                debounce.limits = On;
                DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
                DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
            } else
                hal.limits.interrupt_callback(limitsGetState());
        }
#endif
#if I2C_STROBE_ENABLE && (I2C_STROBE_BIT & 0xFC00)
        if((ifg & I2C_STROBE_BIT) && i2c_strobe.callback)
            i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
#endif
#if MPG_MODE_BIT && (MPG_MODE_BIT & 0xFC00)
        if(ifg & MPG_MODE_BIT)
            protocol_enqueue_rt_command(mpg_select);
#endif
#if AUXINPUT_MASK & 0xFC00
        if(ifg & aux_irq)
            ioports_event(ifg & aux_irq);
#endif
    }
}

#endif

// Interrupt handler for 1 ms interval timer
void Driver_IncTick (void)
{
#ifdef Z_LIMIT_POLL
    static bool z_limit_state = false;
    if(settings.limits.flags.hard_enabled) {
        bool z_limit = DIGITAL_IN(Z_LIMIT_PORT, Z_LIMIT_PIN) ^ settings.limits.invert.z;
        if(z_limit_state != z_limit) {
            if((z_limit_state = z_limit)) {
                if((debounce.limits = hal.driver_cap.software_debounce)) {
                    DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
                    DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
                } else
                    hal.limits.interrupt_callback(limitsGetState());
            }
        }
    }
#endif

#if SDCARD_ENABLE
    static uint32_t fatfs_ticks = 10;
    if(!(--fatfs_ticks)) {
        disk_timerproc();
        fatfs_ticks = 10;
    }
#endif

    if(delay.ms && !(--delay.ms)) {
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
}
