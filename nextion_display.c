#include "nextion_display.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;  // Already initialized in main
static uint8_t rx_data[4];
static int j = 0;

void nextion_display_init(void) {
	HAL_UART_Receive_IT(&huart1, rx_data, 4);  // Start receiving
}

void nextion_display_rx_callback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (rx_data[0] == 0x23 && rx_data[1] == 0x02 && rx_data[2] == 0x54) {
			if (rx_data[3] == 0x01) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // LED ON
				j++;

				char on_msg[30];
				sprintf(on_msg, "t0.txt=\"%d\"", j);
				uint8_t end_cmd[] = { 0xFF, 0xFF, 0xFF };

				HAL_UART_Transmit(&huart1, (uint8_t*) on_msg, strlen(on_msg),
						HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart1, end_cmd, 3, HAL_MAX_DELAY);
			} else if (rx_data[3] == 0x00) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED OFF
			}
		}
		HAL_UART_Receive_IT(&huart1, rx_data, 4);  // Re-enable reception
	}
}
