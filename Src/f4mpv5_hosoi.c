/*
 * my_plugin.c
 *
 *  Created on: Dec 10, 2021
 *      Author: tai
 */

#include <string.h>

#include "main.h"
#include "driver.h"
#include "serial.h"

#include "grbl/hal.h"
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
//#define UART_PORT GPIOD
//#define UART_TX_PIN 8
//#define UART_RX_PIN 9
const io_stream_t* serial3Init(uint32_t baud_rate);
const io_stream_t* serial4Init(uint32_t baud_rate);

static stream_rx_buffer_t rxbuf3 =
{ 0 };
static stream_tx_buffer_t txbuf3 =
{ 0 };
static enqueue_realtime_command_ptr enqueue_realtime_command3 = protocol_enqueue_realtime_command;

#define USART_GRBL 							USART3
#define USART_GRBL_IRQHandler 	USART3_IRQHandler

static io_stream_properties_t serial[] =
{
{
		.type = StreamType_Serial,
		.instance = 0,
		.flags.claimable = On,
		.flags.claimed = Off,
		.flags.connected = On,
		.flags.can_set_baud =
		On,
		.claim = serialInit },
#ifdef SERIAL2_MOD
{
	.type = StreamType_Serial,
	.instance = 1,
	.flags.claimable = On,
	.flags.claimed = Off,
	.flags.connected = On,
	.flags.can_set_baud = On,
	.flags.modbus_ready = On,
	.claim = serial2Init
}
#endif

		{
				.type = StreamType_Serial,
				.instance = 2,
				.flags.claimable = On,
				.flags.claimed = Off,
				.flags.connected = On,
				.flags.can_set_baud =
				On,
				.claim = serial3Init },

		{
				.type = StreamType_Serial,
				.instance = 3,
				.flags.claimable = On,
				.flags.claimed = Off,
				.flags.connected = On,
				.flags.can_set_baud =
				On,
				.flags.modbus_ready = On,
				.claim = serial4Init }

};

//
// Returns number of free characters in serial input buffer
//
static uint16_t serial3RxFree(void)
{
	uint16_t tail = rxbuf3.tail, head = rxbuf3.head;

	return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serial3RxFlush(void)
{
	rxbuf3.tail = rxbuf3.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serial3RxCancel(void)
{
	rxbuf3.data[rxbuf3.head] = ASCII_CAN;
	rxbuf3.tail = rxbuf3.head;
	rxbuf3.head = BUFNEXT(rxbuf3.head, rxbuf3);
}

//
// Writes a character to the serial output stream
//
static bool serial3PutC(const char c)
{
	uint16_t next_head = BUFNEXT(txbuf3.head, txbuf3);    // Get pointer to next free slot in buffer

	while (txbuf3.tail == next_head)
	{                    // While TX buffer full
		if (!hal.stream_blocking_callback())             // check if blocking for space,
		return false;                               // exit if not (leaves TX buffer in an inconsistent state)
	}
	txbuf3.data[txbuf3.head] = c;                         // Add data to buffer,
	txbuf3.head = next_head;                             	// update head pointer and
	USART_GRBL->CR1 |= USART_CR1_TXEIE;                      	// enable TX interrupts

	return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serial3WriteS(const char *s)
{
	char c, *ptr = (char*) s;

	while ((c = *ptr++) != '\0')
		serial3PutC(c);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//
static void serial3Write(const char *s, uint16_t length)
{
	char *ptr = (char*) s;

	while (length--)
		serial3PutC(*ptr++);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serial3GetC(void)
{
	uint_fast16_t tail = rxbuf3.tail;    // Get buffer pointer

	if (tail == rxbuf3.head) return -1; // no data available

	char data = rxbuf3.data[tail];       // Get next character
	rxbuf3.tail = BUFNEXT(tail, rxbuf3);  // and update pointer

	return (int16_t) data;
}

static bool serial3SuspendInput(bool suspend)
{
	return stream_rx_suspend(&rxbuf3, suspend);
}

static bool serial3SetBaudRate(uint32_t baud_rate)
{
// if USART 1/6/9/10 → HAL_RCC_GetPCLK2Freq() - else: HAL_RCC_GetPCLK1Freq()

	USART_GRBL->CR1 = USART_CR1_RE | USART_CR1_TE;
	USART_GRBL->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), baud_rate);
	USART_GRBL->CR1 |= (USART_CR1_UE | USART_CR1_RXNEIE);

	return true;
}

static bool serial3Disable(bool disable)
{
	if (disable) USART_GRBL->CR1 &= ~USART_CR1_RXNEIE;
	else USART_GRBL->CR1 |= USART_CR1_RXNEIE;

	return true;
}

static bool serial3EnqueueRtCommand(char c)
{
	return enqueue_realtime_command3(c);
}

static enqueue_realtime_command_ptr serial3SetRtHandler(enqueue_realtime_command_ptr handler)
{
	enqueue_realtime_command_ptr prev = enqueue_realtime_command3;

	if (handler) enqueue_realtime_command3 = handler;

	return prev;
}

const io_stream_t* serial3Init(uint32_t baud_rate)
{

	static const io_stream_t stream =
	{
			.type = StreamType_Serial,
			.state.connected = On,
			.read = serial3GetC,
			.write = serial3WriteS,
			.write_n = serial3Write,
			.write_char = serial3PutC,
			.enqueue_rt_command = serial3EnqueueRtCommand,
			.get_rx_buffer_free = serial3RxFree,
//			.get_rx_buffer_count = serial3RxCount,
//			.get_tx_buffer_count = serial3TxCount,
//			.reset_write_buffer = serial3TxFlush,
			.reset_read_buffer = serial3RxFlush,
			.cancel_read_buffer = serial3RxCancel,
			.suspend_read = serial3SuspendInput,
			.disable_rx = serial3Disable,
			.set_baud_rate = serial3SetBaudRate,
			.set_enqueue_rt_handler = serial3SetRtHandler };

	if (serial[0].flags.claimed) return NULL;

	serial[0].flags.claimed = On;

	GPIO_InitTypeDef GPIO_InitStructure =
	{ 0 };

	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	__HAL_RCC_GPIOB_CLK_ENABLE();	// USART3 Port
	__HAL_RCC_USART3_CLK_ENABLE();

	GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	serial3SetBaudRate(baud_rate);

	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);

	static const periph_pin_t tx =
	{ .function = Output_TX, .group = PinGroup_UART3, .port = GPIOB, .pin = 10, .mode =
	{ .mask = PINMODE_OUTPUT }, .description = "UART3" };

	static const periph_pin_t rx =
	{ .function = Input_RX, .group = PinGroup_UART3, .port = GPIOB, .pin = 11, .mode =
	{ .mask = PINMODE_NONE }, .description = "UART3" };

	hal.periph_port.register_pin(&rx);
	hal.periph_port.register_pin(&tx);

	return &stream;
}

// SERIAL 4
const io_stream_t* serial4Init(uint32_t baud_rate)
{

}

void USART_GRBL_IRQHandler(void)
{
//	if (USART_GRBL->SR & USART_SR_RXNE)
//	{
//		uint32_t data = USART_GRBL->DR;
//		if (!enqueue_realtime_command3((char) data))
//		{             // Check and strip realtime commands...
//			uint16_t next_head = BUFNEXT(rxbuf3.head, rxbuf3);    // Get and increment buffer pointer
//			if (next_head == rxbuf3.tail)                         // If buffer full
//			rxbuf3.overflow = 1;                             // flag overflow
//			else
//			{
//				rxbuf3.data[rxbuf3.head] = (char) data;            // if not add data to buffer
//				rxbuf3.head = next_head;                         // and update pointer
//			}
//		}
//	}
//
//	if ((USART_GRBL->SR & USART_SR_TXE) && (USART_GRBL->CR1 & USART_CR1_TXEIE))
//	{
//		uint_fast16_t tail = txbuf3.tail;            // Get buffer pointer
//		USART_GRBL->DR = txbuf3.data[tail];               // Send next character
//		txbuf3.tail = tail = BUFNEXT(tail, txbuf3);   // and increment pointer
//		if (tail == txbuf3.head)                      // If buffer empty then
//		USART_GRBL->CR1 &= ~USART_CR1_TXEIE;         // disable UART TX interrupt
//	}
}

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

// Enable UART
	static io_stream_details_t streams =
	{ .n_streams = sizeof(serial) / sizeof(io_stream_properties_t), .streams = serial, };
	stream_register_streams(&streams);

// Enable Encoder Timer
//	MX_TIM2_Init();
//	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);

}

