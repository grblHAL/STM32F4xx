/*
 * my_plugin.c
 *
 *  Created on: Dec 10, 2021
 *      Author: tai
 */

#include "main.h"
#include "driver.h"
#include "serial.h"

#include "grbl/limits.h"
#include "grbl/protocol.h"
#include "grbl/motor_pins.h"
#include "grbl/pin_bits_masks.h"
#include "grbl/stepdir_map.h"

#define PU1_Pin GPIO_PIN_2
#define PU1_GPIO_Port GPIOA
#define DR1_Pin GPIO_PIN_3
#define DR1_GPIO_Port GPIOA
#define LedG_Pin GPIO_PIN_6
#define LedG_GPIO_Port GPIOC
#define LedB_Pin GPIO_PIN_7
#define LedB_GPIO_Port GPIOC
#define LedR_Pin GPIO_PIN_8
#define LedR_GPIO_Port GPIOC

#define led1		  GPIOC,GPIO_PIN_6	 //GR
#define led2		  GPIOC,GPIO_PIN_7	 //BL
#define led3		  GPIOC,GPIO_PIN_8	 //RD
#define led1on		HAL_GPIO_WritePin(led1,GPIO_PIN_SET)
#define led2on		HAL_GPIO_WritePin(led2,GPIO_PIN_SET)
#define led3on		HAL_GPIO_WritePin(led3,GPIO_PIN_SET)
#define led1off		HAL_GPIO_WritePin(led1,GPIO_PIN_RESET)
#define led2off		HAL_GPIO_WritePin(led2,GPIO_PIN_RESET)
#define led3off		HAL_GPIO_WritePin(led3,GPIO_PIN_RESET)
#define led1toggle  HAL_GPIO_TogglePin(led1);
#define led2toggle  HAL_GPIO_TogglePin(led2);
#define led3toggle  HAL_GPIO_TogglePin(led3);

#define  X0		GPIOE,GPIO_PIN_0
#define  X1		GPIOE,GPIO_PIN_1
#define  X2		GPIOE,GPIO_PIN_2
#define  X3		GPIOE,GPIO_PIN_3
#define  X4		GPIOE,GPIO_PIN_4
#define  X5		GPIOE,GPIO_PIN_5
#define  X6		GPIOE,GPIO_PIN_6
#define  X7		GPIOE,GPIO_PIN_7
#define  X8		GPIOE,GPIO_PIN_8
#define  X9		GPIOE,GPIO_PIN_9
#define  X10	GPIOE,GPIO_PIN_10
#define  X11	GPIOE,GPIO_PIN_11

#define X0R       HAL_GPIO_ReadPin(X0)
#define X1R       HAL_GPIO_ReadPin(X1)
#define X2R       HAL_GPIO_ReadPin(X2)
#define X3R       HAL_GPIO_ReadPin(X3)
#define X4R       HAL_GPIO_ReadPin(X4)
#define X5R       HAL_GPIO_ReadPin(X5)
#define X6R       HAL_GPIO_ReadPin(X6)
#define X7R       HAL_GPIO_ReadPin(X7)
#define X8R       HAL_GPIO_ReadPin(X8)
#define X9R       HAL_GPIO_ReadPin(X9)
#define X10R      HAL_GPIO_ReadPin(X10)
#define X11R      HAL_GPIO_ReadPin(X11)

#define  X0set	HAL_GPIO_WritePin(X0, GPIO_PIN_SET);
#define  X1set	HAL_GPIO_WritePin(X1, GPIO_PIN_SET);
#define  X2set	HAL_GPIO_WritePin(X2, GPIO_PIN_SET);
#define  X3set	HAL_GPIO_WritePin(X3, GPIO_PIN_SET);
#define  X4set	HAL_GPIO_WritePin(X4, GPIO_PIN_SET);
#define  X5set	HAL_GPIO_WritePin(X5, GPIO_PIN_SET);
#define  X6set	HAL_GPIO_WritePin(X6, GPIO_PIN_SET);
#define  X7set	HAL_GPIO_WritePin(X7, GPIO_PIN_SET);
#define  X8set	HAL_GPIO_WritePin(X8, GPIO_PIN_SET);
#define  X9set	HAL_GPIO_WritePin(X9, GPIO_PIN_SET);
#define  X10set	HAL_GPIO_WritePin(X10, GPIO_PIN_SET);
#define  X11set	HAL_GPIO_WritePin(X11, GPIO_PIN_SET);

#define  X0clr 	HAL_GPIO_WritePin(X0, GPIO_PIN_RESET);
#define  X1clr 	HAL_GPIO_WritePin(X1, GPIO_PIN_RESET);
#define  X2clr 	HAL_GPIO_WritePin(X2, GPIO_PIN_RESET);
#define  X3clr 	HAL_GPIO_WritePin(X3, GPIO_PIN_RESET);
#define  X4clr 	HAL_GPIO_WritePin(X4, GPIO_PIN_RESET);
#define  X5clr 	HAL_GPIO_WritePin(X5, GPIO_PIN_RESET);
#define  X6clr 	HAL_GPIO_WritePin(X6, GPIO_PIN_RESET);
#define  X7clr 	HAL_GPIO_WritePin(X7, GPIO_PIN_RESET);
#define  X8clr 	HAL_GPIO_WritePin(X8, GPIO_PIN_RESET);
#define  X9clr 	HAL_GPIO_WritePin(X9, GPIO_PIN_RESET);
#define  X10clr HAL_GPIO_WritePin(X10, GPIO_PIN_RESET);
#define  X11clr HAL_GPIO_WritePin(X11, GPIO_PIN_RESET);

#define  X0tog 	HAL_GPIO_TogglePin(X0);
#define  X1tog 	HAL_GPIO_TogglePin(X1);
#define  X2tog 	HAL_GPIO_TogglePin(X2);
#define  X3tog 	HAL_GPIO_TogglePin(X3);
#define  X4tog 	HAL_GPIO_TogglePin(X4);
#define  X5tog 	HAL_GPIO_TogglePin(X5);
#define  X6tog 	HAL_GPIO_TogglePin(X6);
#define  X7tog 	HAL_GPIO_TogglePin(X7);
#define  X8tog 	HAL_GPIO_TogglePin(X8);
#define  X9tog 	HAL_GPIO_TogglePin(X9);
#define  X10tog HAL_GPIO_TogglePin(X10);
#define  X11tog HAL_GPIO_TogglePin(X11);

TIM_HandleTypeDef htim2;

static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime;

stepper_t stepper =
{ 0 };

static bool encoder_interrupt_busy = false;

static void MX_TIM2_Init(void);
static void encoder_handler(sys_state_t state);

// Add info about our plugin to the $I report.
static void on_report_my_options(bool newopt)
{
	on_report_options(newopt);

	if (!newopt) hal.stream.write("[PLUGIN:Blink LED v1.00]" ASCII_EOL);
}

static void blink_led(sys_state_t state)
{
//	static bool led_on = false;
	static uint32_t ms = 0;

	if (hal.get_elapsed_ticks() >= ms)
	{
		ms = hal.get_elapsed_ticks() + 1000; //ms

//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Green
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7); // Blue
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // Red

// Alternative Method
//		static bool led_on = false;
//		static uint32_t ms = 0;
//
//		if (hal.get_elapsed_ticks() >= ms)
//		{
//			ms = hal.get_elapsed_ticks() + 500; //ms
//			led_on = !led_on;
//			if (led_on)
//			GPIOC->ODR |= GPIO_PIN_13;
//			else
//			GPIOC->ODR &= ~GPIO_PIN_13;
//		}

	}

	on_execute_realtime(state);
}

//#define SLOW_JOG
#define FAST_JOG

volatile int step = 0;
volatile uint32_t counter = 0;
volatile int16_t count = 0;
volatile int16_t prev_count = 0;
volatile int16_t delta_count = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(htim);

//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	counter = __HAL_TIM_GET_COUNTER(htim);
	count = (int16_t) (counter / 4);
	delta_count = count - prev_count;

	if ((delta_count < 0) || (delta_count > 0))
	{
		protocol_enqueue_rt_command(encoder_handler);

	}
}

static void MX_TIM2_Init(void)
{
	TIM_Encoder_InitTypeDef sConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
}

void output_init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	//X0-X11
	GPIO_InitStruct.Pin = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

////PU1,DR1-PU8-DR8
//	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
//			| GPIO_PIN_7;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
//	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
////CS0
//	GPIO_InitStruct.Pin = GPIO_PIN_15;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
////KEY
//	GPIO_InitStruct.Pin = GPIO_PIN_12;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void encoder_handler(sys_state_t state)
{

	if (encoder_interrupt_busy == true) return;

	if ((delta_count < 0) || (delta_count > 0))
	{
		encoder_interrupt_busy = true;
	}

	if (delta_count > 0)
	{

#ifdef SLOW_JOG

		stepper.step_outbits.x = 1;
		stepper.dir_outbits.x = 1;
		stepper.dir_change = 1; // set when dir_outbits is changed.
		hal.stepper.pulse_start(&stepper);
		//		hal.stepper.
		sys.position[0] = sys.position[0] + (stepper.dir_outbits.x ? -1 : 1);
		sync_position();
#endif
#ifdef FAST_JOG
		for (uint16_t step = 0; step < 100; step++)
		{
			stepper.step_outbits.x = 1;
			stepper.dir_outbits.x = 1;
			stepper.dir_change = 1; // set when dir_outbits is changed.
			hal.stepper.pulse_start(&stepper);

			sys.position[0] = sys.position[0] + (stepper.dir_outbits.x ? -1 : 1);
			sync_position();

			hal.delay_ms(1, NULL);

		}
		//		stepper.step_outbits.x = 1;
		//		stepper.dir_outbits.x = 1;
		//		stepper.dir_change = 1; // set when dir_outbits is changed.
		//		hal.stepper.pulse_start(&stepper);
		//		sys.position[0] = sys.position[0] + (stepper.dir_outbits.x ? -1 : 1);
		//		sync_position();

#endif

	}
	else if (delta_count < 0)
	{

#ifdef SLOW_JOG
		stepper.step_outbits.x = 1;
		stepper.dir_outbits.x = 0;
		stepper.dir_change = 1; // set when dir_outbits is changed.
		hal.stepper.pulse_start(&stepper);
		sys.position[0] = sys.position[0] + (stepper.dir_outbits.x ? -1 : 1);
		sync_position();
#endif
#ifdef FAST_JOG
		for (uint16_t step = 0; step < 100; step++)
		{
			stepper.step_outbits.x = 1;
			stepper.dir_outbits.x = 0;
			stepper.dir_change = 1; // set when dir_outbits is changed.
			hal.stepper.pulse_start(&stepper);

			sys.position[0] = sys.position[0] + (stepper.dir_outbits.x ? -1 : 1);
			sync_position();

			hal.delay_ms(1, NULL);

		}

#endif

	}
	prev_count = count;
	encoder_interrupt_busy = false;
}

/************************************
 *
 * SERIAL PORT
 *
 ************************************/

/************************************
 *
 * MAIN CODE
 *
 ************************************/
void board_init(void)
{
	// Add info about our plugin to the $I report.
	on_report_options = grbl.on_report_options;
	grbl.on_report_options = on_report_my_options;

	// Add blink LED function to grblHAL foreground process
	led_init();
	on_execute_realtime = grbl.on_execute_realtime;
	grbl.on_execute_realtime = blink_led;

	// Enable F4MPV5 outputs
	output_init();
	// Enable Steppers
	X0set
	;

	// Enable Encoder Timer
//	MX_TIM2_Init();
//	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);

}

