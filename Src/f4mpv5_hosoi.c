/*
 * my_plugin.c
 *
 *  Created on: Dec 10, 2021
 *      Author: tai
 */

#include "f4mpv4_hosoi.h"

TIM_HandleTypeDef htim2, htim3, htim5;
UART_HandleTypeDef huart4;

static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime;

stepper_t stepper =
		{
		0
		};

static bool encoder_interrupt_busy = false;

static coord_data_t target =
		{
		0
		}, origin =
		{
		0
		};
// Initialize planner data struct for motion blocks.
static plan_line_data_t plan_data;

static void encoder_init(void);
static void encoder_handler(sys_state_t state);

// Add info about our plugin to the $I report.
static void on_report_my_options(bool newopt)
{
on_report_options(newopt);

if (!newopt)
	hal.stream.write("[PLUGIN:Blink LED v1.00]" ASCII_EOL);
}

// ENCODER
#define NUMBER_OF_ENCODER 3

volatile int step = 0;
volatile uint32_t counter = 0;
volatile int16_t count = 0;
volatile int16_t prev_count = 0;
volatile int16_t delta_count = 0;
static volatile float sign;

typedef struct
{
	volatile uint32_t counter;
	volatile int16_t count;
	volatile int16_t prev_count;
	volatile int16_t delta_count;
	volatile float sign;
	TIM_HandleTypeDef *timer;
} knob_t;

static knob_t knob_x, knob_y, knob_z;
static knob_t *knob_ptr[NUMBER_OF_ENCODER] =
		{
		&knob_x, &knob_y, &knob_z
		};

void TIM2_IRQHandler(void)
{
HAL_TIM_IRQHandler(&htim2);
}
void TIM3_IRQHandler(void)
{
HAL_TIM_IRQHandler(&htim3);
}
//void TIM5_IRQHandler(void)
//{
//HAL_TIM_IRQHandler(&htim5);
//}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim_encoder)
{
GPIO_InitTypeDef GPIO_InitStruct =
		{
		0
		};
if (htim_encoder->Instance == TIM2)
	{
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	}
else if (htim_encoder->Instance == TIM3)
	{
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	}
//else if (htim_encoder->Instance == TIM5)
//	{
//	/* USER CODE BEGIN TIM5_MspInit 0 */
//
//	/* USER CODE END TIM5_MspInit 0 */
//	/* Peripheral clock enable */
//	__HAL_RCC_TIM5_CLK_ENABLE();
//
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//	/**TIM5 GPIO Configuration
//	 PA0-WKUP     ------> TIM5_CH1
//	 PA1     ------> TIM5_CH2
//	 */
//	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//	/* TIM5 interrupt Init */
//	HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(TIM5_IRQn);
//	/* USER CODE BEGIN TIM5_MspInit 1 */
//
//	/* USER CODE END TIM5_MspInit 1 */
//	}

}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim_encoder)
{
if (htim_encoder->Instance == TIM2)
	{
	__HAL_RCC_TIM2_CLK_DISABLE();
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);
	HAL_NVIC_DisableIRQ(TIM2_IRQn);
	}
else if (htim_encoder->Instance == TIM3)
	{
	__HAL_RCC_TIM3_CLK_DISABLE();
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4 | GPIO_PIN_5);
	HAL_NVIC_DisableIRQ(TIM3_IRQn);
	}
//else if (htim_encoder->Instance == TIM5)
//	{
//	__HAL_RCC_TIM5_CLK_DISABLE();
//	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);
//	HAL_NVIC_DisableIRQ(TIM5_IRQn);
//	}

}

int temp = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
/* Prevent unused argument(s) compilation warning */
UNUSED(htim);
////	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
//	counter = __HAL_TIM_GET_COUNTER(htim);
//	count = (int16_t) (counter / 4);
//	delta_count = count - prev_count;
//	if ((delta_count < 0) || (delta_count > 0))
//	{
//		protocol_enqueue_rt_command(encoder_handler);
//	}

if (htim == knob_x.timer)
	{
	knob_x.counter = __HAL_TIM_GET_COUNTER(htim);
	knob_x.count = (int16_t) (knob_x.counter / 4);

//	if (knob_x.delta_count != 0)
//		{
//		protocol_enqueue_rt_command(encoder_handler);
//		}
	}

if (htim == knob_z.timer)
	{
	temp = __HAL_TIM_GET_COUNTER(htim);
	knob_z.counter = __HAL_TIM_GET_COUNTER(htim);
	knob_z.count = (int16_t) (knob_z.counter / 4);

//	if (knob_z.delta_count != 0)
//		{
//		protocol_enqueue_rt_command(encoder_handler);
//		}
	}
knob_x.delta_count = knob_x.count - knob_x.prev_count;
knob_z.delta_count = knob_z.count - knob_z.prev_count;

protocol_enqueue_rt_command(encoder_handler);


}

static void encoder_init(void)
{
// Encoder 1 - OKAY!!! Do not edit
TIM_Encoder_InitTypeDef sConfig =
		{
		0
		};
TIM_MasterConfigTypeDef sMasterConfig =
		{
		0
		};

//----------------------------------------
htim2.Instance = TIM2;
htim2.Init.Prescaler = 0;
htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
htim2.Init.Period = 4294967295;
htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
//----------------------------------------
//htim5.Instance = TIM5;
//htim5.Init.Prescaler = 0;
//htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
//htim5.Init.Period = 4294967295;
//htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
//sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
//sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
//sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//sConfig.IC1Filter = 0;
//sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
//sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
//sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
//sConfig.IC2Filter = 0;
//if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
//	{
//	Error_Handler();
//	}
//sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
//	{
//	Error_Handler();
//	}
//----------------------------------------
htim3.Instance = TIM3;
htim3.Init.Prescaler = 0;
htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
htim3.Init.Period = 65535;
htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
sConfig.IC1Filter = 0;
sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
sConfig.IC2Filter = 0;
if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
	Error_Handler();
	}

//----------------------------------------
knob_x.timer = &htim2;
//knob_y.timer = &htim5;
knob_z.timer = &htim3;
for (uint8_t i = 0; i < NUMBER_OF_ENCODER; i++)
	{
	knob_ptr[i]->count = 0;
	knob_ptr[i]->counter = 0;
	knob_ptr[i]->delta_count = 0;
	knob_ptr[i]->prev_count = 0;
	}
HAL_TIM_Encoder_Start_IT(knob_x.timer, TIM_CHANNEL_ALL);
//HAL_TIM_Encoder_Start_IT(knob_y.timer, TIM_CHANNEL_ALL);
HAL_TIM_Encoder_Start_IT(knob_z.timer, TIM_CHANNEL_ALL);
}

static void encoder_handler(sys_state_t state)
{
//--------------------------------------------------
//if (encoder_interrupt_busy == true) return;
//if (knob_x.delta_count != 0)
//	{
//	encoder_interrupt_busy = true;
//	sign = knob_x.delta_count >= 0 ? 1.0f : -1.0f;
//	plan_line_data_t plan_data_x =
//			{
//			0
//			};
//	plan_data.feed_rate = 30000.0f;
//	protocol_buffer_synchronize();
//	sync_position();
//	// Get current position.
//	system_convert_array_steps_to_mpos(origin.values, sys.position);
//	target.x = origin.x + sign * 25.0f;
//	knob_x.prev_count = knob_x.count;
//	mc_line(target.values, &plan_data_x);
//	}
//if (knob_z.delta_count != 0)
//	{
//	encoder_interrupt_busy = true;
//	sign = knob_z.delta_count >= 0 ? 1.0f : -1.0f;
//	plan_line_data_t plan_data_z =
//			{
//			0
//			};
//	plan_data_z.feed_rate = 30000.0f;
//	protocol_buffer_synchronize();
//	sync_position();
//	// Get current position.
//	system_convert_array_steps_to_mpos(origin.values, sys.position);
//	target.z = origin.z + sign * 25.0f;
//	knob_z.prev_count = knob_z.count;
//	mc_line(target.values, &plan_data_z);
//	}
//encoder_interrupt_busy = false;
//--------------------------------------------------
knob_x.prev_count = knob_x.count;
knob_z.prev_count = knob_z.count;
if (encoder_interrupt_busy == true) return;
if ((knob_x.delta_count != 0) || (knob_z.delta_count != 0))
	{
	encoder_interrupt_busy = true;
	knob_x.sign = knob_x.delta_count >= 0 ? 1.0f : -1.0f;
	knob_z.sign = knob_z.delta_count >= 0 ? 1.0f : -1.0f;
	plan_line_data_t plan_data =
			{
			0
			};
	plan_data.feed_rate = 30000.0f;
	protocol_buffer_synchronize();
	sync_position();
	// Get current position.
	system_convert_array_steps_to_mpos(origin.values, sys.position);
	if (knob_x.delta_count != 0) target.x = origin.x + knob_x.sign * 5.0f;
	if (knob_z.delta_count != 0) target.z = origin.z + knob_z.sign * 5.0f;

	mc_line(target.values, &plan_data);
	encoder_interrupt_busy = false;
	}
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

	// temp - to be deleted
//	    target.x = sys.position[0] + 20.0f;
//	    target.y = sys.position[1];
//	    hal.stream.write(uitoa(x));
//	    hal.stream.write(",");
//	    hal.stream.write(uitoa(y));
//	    hal.stream.write(ASCII_EOL);
//	    mc_line(target.values, &plan_data);

	}
//protocol_enqueue_rt_command(encoder_handler);
on_execute_realtime(state);
}

void led_init(void)
{
GPIO_InitTypeDef GPIO_InitStruct;

GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8,
		GPIO_PIN_RESET);
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

/************************************
 *
 * SERIAL PORT
 *
 ************************************/
const io_stream_t* serial3Init(uint32_t baud_rate);
const io_stream_t* serial4Init(uint32_t baud_rate);
static stream_rx_buffer_t rxbuf3 =
		{
		0
		};
static stream_tx_buffer_t txbuf3 =
		{
		0
		};
static enqueue_realtime_command_ptr enqueue_realtime_command3 =
		protocol_enqueue_realtime_command;

#define USART_GRBL 							USART3
#define USART_GRBL_IRQHandler 	USART3_IRQHandler

static io_stream_properties_t serial[] =
		{
		{
		.type = StreamType_Serial,
				.instance = 2,
				.flags.claimable = On,
				.flags.claimed = Off,
				.flags.connected = On,
				.flags.can_set_baud =
				On,
				.claim = serial3Init
		},

		{
		.type = StreamType_Serial,
				.instance = 3,
				.flags.claimable = On,
				.flags.claimed = Off,
				.flags.connected = On,
				.flags.can_set_baud =
				On,
				.flags.modbus_ready = On,
				.claim = serial4Init
		}

		};

//
// Returns number of free characters in serial input buffer
//
static uint16_t serial3RxFree(void)
{
uint16_t tail = rxbuf3.tail, head = rxbuf3.head;

return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//endregion

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
uint16_t next_head = BUFNEXT(txbuf3.head, txbuf3); // Get pointer to next free slot in buffer

while (txbuf3.tail == next_head)
	{                    // While TX buffer full
	if (!hal.stream_blocking_callback())         // check if blocking for space,
		return false;   // exit if not (leaves TX buffer in an inconsistent state)
	}
txbuf3.data[txbuf3.head] = c;                         	// Add data to buffer,
txbuf3.head = next_head;                            // update head pointer and
USART_GRBL->CR1 |= USART_CR1_TXEIE;                    // enable TX interrupts

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

if (tail == rxbuf3.head)
	return -1; // no data available

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
if (disable)
	USART_GRBL->CR1 &= ~USART_CR1_RXNEIE;
else
	USART_GRBL->CR1 |= USART_CR1_RXNEIE;

return true;
}

static bool serial3EnqueueRtCommand(char c)
{
return enqueue_realtime_command3(c);
}

static enqueue_realtime_command_ptr serial3SetRtHandler(
		enqueue_realtime_command_ptr handler)
{
enqueue_realtime_command_ptr prev = enqueue_realtime_command3;

if (handler)
	enqueue_realtime_command3 = handler;

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
				.write_n =
						serial3Write,
				.write_char = serial3PutC,
				.enqueue_rt_command = serial3EnqueueRtCommand,
				.get_rx_buffer_free =
						serial3RxFree,
				//			.get_rx_buffer_count = serial3RxCount,
//			.get_tx_buffer_count = serial3TxCount,
//			.reset_write_buffer = serial3TxFlush,
				.reset_read_buffer = serial3RxFlush,
				.cancel_read_buffer = serial3RxCancel,
				.suspend_read = serial3SuspendInput,
				.disable_rx = serial3Disable,
				.set_baud_rate = serial3SetBaudRate,
				.set_enqueue_rt_handler = serial3SetRtHandler
		};

if (serial[0].flags.claimed)
	return NULL;

serial[0].flags.claimed = On;

GPIO_InitTypeDef GPIO_InitStructure =
		{
		0
		};

GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
GPIO_InitStructure.Pull = GPIO_NOPULL;
GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

__HAL_RCC_GPIOB_CLK_ENABLE();	// USART3 Port
__HAL_RCC_USART3_CLK_ENABLE();

GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_11;
GPIO_InitStructure.Alternate = GPIO_AF7_USART3;
HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

serial3SetBaudRate(baud_rate);

HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(USART3_IRQn);

static const periph_pin_t tx =
		{
		.function = Output_TX,
				.group = PinGroup_UART3,
				.port = GPIOB,
				.pin = 10,
				.mode =
						{
						.mask = PINMODE_OUTPUT
						},
				.description = "UART3"
		};

static const periph_pin_t rx =
		{
		.function = Input_RX,
				.group = PinGroup_UART3,
				.port = GPIOB,
				.pin = 11,
				.mode =
						{
						.mask = PINMODE_NONE
						},
				.description = "UART3"
		};

hal.periph_port.register_pin(&rx);
hal.periph_port.register_pin(&tx);

return &stream;
}

void USART_GRBL_IRQHandler(void)
{
if (USART_GRBL->SR & USART_SR_RXNE)
	{
	uint32_t data = USART_GRBL->DR;
	if (!enqueue_realtime_command3((char) data))
		{             // Check and strip realtime commands...
		uint16_t next_head = BUFNEXT(rxbuf3.head, rxbuf3); // Get and increment buffer pointer
		if (next_head == rxbuf3.tail)                         // If buffer full
			rxbuf3.overflow = 1;                             // flag overflow
		else
			{
			rxbuf3.data[rxbuf3.head] = (char) data;     // if not add data to buffer
			rxbuf3.head = next_head;                         // and update pointer
			}
		}
	}

if ((USART_GRBL->SR & USART_SR_TXE) && (USART_GRBL->CR1 & USART_CR1_TXEIE))
	{
	uint_fast16_t tail = txbuf3.tail;            // Get buffer pointer
	USART_GRBL->DR = txbuf3.data[tail];               // Send next character
	txbuf3.tail = tail = BUFNEXT(tail, txbuf3);   // and increment pointer
	if (tail == txbuf3.head)                      // If buffer empty then
		USART_GRBL->CR1 &= ~USART_CR1_TXEIE;         // disable UART TX interrupt
	}
}

// SERIAL 4
static stream_rx_buffer_t rxbuf4 =
		{
		0
		};
static stream_tx_buffer_t txbuf4 =
		{
		0
		};
static enqueue_realtime_command_ptr enqueue_realtime_command4 =
		protocol_enqueue_realtime_command;

#define USART_MODBUS 							UART4
#define USART_MODBUS_IRQHandler 	UART4_IRQHandler

//
// Returns number of free characters in serial input buffer
//
static uint16_t serial4RxFree(void)
{
uint16_t tail = rxbuf4.tail, head = rxbuf4.head;

return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serial4RxCount(void)
{
uint32_t tail = rxbuf4.tail, head = rxbuf4.head;

return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serial4RxFlush(void)
{
rxbuf4.tail = rxbuf4.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serial4RxCancel(void)
{
rxbuf4.data[rxbuf4.head] = ASCII_CAN;
rxbuf4.tail = rxbuf4.head;
rxbuf4.head = BUFNEXT(rxbuf4.head, rxbuf4);
}

//
// Writes a character to the serial output stream
//
static bool serial4PutC(const char c)
{
uint16_t next_head = BUFNEXT(txbuf4.head, txbuf4); // Get pointer to next free slot in buffer

while (txbuf4.tail == next_head)
	{                    // While TX buffer full
	if (!hal.stream_blocking_callback())         // check if blocking for space,
		return false;   // exit if not (leaves TX buffer in an inconsistent state)
	}
txbuf4.data[txbuf4.head] = c;                         	// Add data to buffer,
txbuf4.head = next_head;                            // update head pointer and
USART_MODBUS->CR1 |= USART_CR1_TXEIE;                  // enable TX interrupts

return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serial4WriteS(const char *s)
{
char c, *ptr = (char*) s;

while ((c = *ptr++) != '\0')
	serial4PutC(c);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//
static void serial4Write(const char *s, uint16_t length)
{
char *ptr = (char*) s;

while (length--)
	serial4PutC(*ptr++);
}

//
// Flushes the serial output buffer
//
static void serial4TxFlush(void)
{
UART4->CR1 &= ~USART_CR1_TXEIE;     // Disable TX interrupts
txbuf4.tail = txbuf4.head;
}

//
// Returns number of characters pending transmission
//
static uint16_t serial4TxCount(void)
{
uint32_t tail = txbuf4.tail, head = txbuf4.head;

return BUFCOUNT(head, tail, TX_BUFFER_SIZE)
		+ (UART4->SR & USART_SR_TC ? 0 : 1);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serial4GetC(void)
{
uint_fast16_t tail = rxbuf4.tail;    // Get buffer pointer

if (tail == rxbuf4.head)
	return -1; // no data available

char data = rxbuf4.data[tail];       // Get next character
rxbuf4.tail = BUFNEXT(tail, rxbuf4);  // and update pointer

return (int16_t) data;
}

static bool serial4SuspendInput(bool suspend)
{
return stream_rx_suspend(&rxbuf4, suspend);
}

static bool serial4SetBaudRate(uint32_t baud_rate)
{
// if USART 1/6/9/10 → HAL_RCC_GetPCLK2Freq() - else: HAL_RCC_GetPCLK1Freq()

USART_MODBUS->CR1 = USART_CR1_RE | USART_CR1_TE;
USART_MODBUS->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), 38400);
USART_MODBUS->CR1 |= (USART_CR1_UE | USART_CR1_RXNEIE);

huart4.Instance = UART4;
huart4.Init.BaudRate = 38400;
huart4.Init.WordLength = UART_WORDLENGTH_9B;
huart4.Init.StopBits = UART_STOPBITS_1;
huart4.Init.Parity = UART_PARITY_EVEN;
huart4.Init.Mode = UART_MODE_TX_RX;
huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart4.Init.OverSampling = UART_OVERSAMPLING_16;
if (HAL_UART_Init(&huart4) != HAL_OK)
	{
	Error_Handler();
	}

return true;
}

static bool serial4Disable(bool disable)
{
if (disable)
	USART_MODBUS->CR1 &= ~USART_CR1_RXNEIE;
else
	USART_MODBUS->CR1 |= USART_CR1_RXNEIE;

return true;
}

static bool serial4EnqueueRtCommand(char c)
{
return enqueue_realtime_command4(c);
}

static enqueue_realtime_command_ptr serial4SetRtHandler(
		enqueue_realtime_command_ptr handler)
{
enqueue_realtime_command_ptr prev = enqueue_realtime_command4;

if (handler)
	enqueue_realtime_command4 = handler;

return prev;
}

const io_stream_t* serial4Init(uint32_t baud_rate)
{

static const io_stream_t stream =
		{
		.type = StreamType_Serial,
				.instance = 3,
				.state.connected = On,
				.read = serial4GetC,
				.write = serial4WriteS,
				.write_n = serial4Write,
				.write_char = serial4PutC,
				.enqueue_rt_command = serial4EnqueueRtCommand,
				.get_rx_buffer_free = serial4RxFree,
				.get_rx_buffer_count = serial4RxCount,
				.get_tx_buffer_count = serial4TxCount,
				.reset_write_buffer = serial4TxFlush,
				.reset_read_buffer = serial4RxFlush,
				.cancel_read_buffer = serial4RxCancel,
				.suspend_read = serial4SuspendInput,
				.disable_rx = serial4Disable,
				.set_baud_rate = serial4SetBaudRate,
				.set_enqueue_rt_handler = serial4SetRtHandler
		};

if (serial[1].flags.claimed)
	return NULL;

serial[1].flags.claimed = On;

GPIO_InitTypeDef GPIO_InitStructure =
		{
		0
		};

GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
GPIO_InitStructure.Pull = GPIO_NOPULL;
GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

__HAL_RCC_GPIOC_CLK_ENABLE();	// USART4 Port
__HAL_RCC_UART4_CLK_ENABLE();

GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_11;
GPIO_InitStructure.Alternate = GPIO_AF8_UART4;
HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

serial4SetBaudRate(baud_rate);

HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(UART4_IRQn);

static const periph_pin_t tx =
		{
		.function = Output_TX,
				.group = PinGroup_UART4,
				.port = GPIOC,
				.pin = 10,
				.mode =
						{
						.mask = PINMODE_OUTPUT
						},
				.description = "UART4"
		};

static const periph_pin_t rx =
		{
		.function = Input_RX,
				.group = PinGroup_UART4,
				.port = GPIOC,
				.pin = 11,
				.mode =
						{
						.mask = PINMODE_NONE
						},
				.description = "UART4"
		};

hal.periph_port.register_pin(&rx);
hal.periph_port.register_pin(&tx);

return &stream;
}

void USART_MODBUS_IRQHandler(void)
{
if (USART_MODBUS->SR & USART_SR_RXNE)
	{
	uint32_t data = USART_MODBUS->DR;
	if (!enqueue_realtime_command4((char) data))
		{             // Check and strip realtime commands...
		uint16_t next_head = BUFNEXT(rxbuf4.head, rxbuf4); // Get and increment buffer pointer
		if (next_head == rxbuf4.tail)                         // If buffer full
			rxbuf4.overflow = 1;                             // flag overflow
		else
			{
			rxbuf4.data[rxbuf4.head] = (char) data;     // if not add data to buffer
			rxbuf4.head = next_head;                         // and update pointer
			}
		}
	}

if ((USART_MODBUS->SR & USART_SR_TXE)
		&& (USART_MODBUS->CR1 & USART_CR1_TXEIE))
	{
	uint_fast16_t tail = txbuf4.tail;            // Get buffer pointer
	USART_MODBUS->DR = txbuf4.data[tail];               // Send next character
	txbuf4.tail = tail = BUFNEXT(tail, txbuf4);   // and increment pointer
	if (tail == txbuf4.head)                      // If buffer empty then
		USART_MODBUS->CR1 &= ~USART_CR1_TXEIE;        // disable UART TX interrupt
	}
}

/************************************
 *
 * MAIN CODE
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
encoder_init();
//HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);

// Enable UART
static io_stream_details_t streams =
		{
		.n_streams = sizeof(serial) / sizeof(io_stream_properties_t), .streams =
				serial,
		};
stream_register_streams(&streams);
// Enable VFD Spindle Control

// Enable Constant Jogging
memset(&plan_data, 0, sizeof(plan_line_data_t)); // Zero plan_data struct

}

