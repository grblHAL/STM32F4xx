/*

  can.c - CAN bus driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2022 Jon Escombe
  Copyright (c) 2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

#ifdef CAN_PORT

#include "grbl/task.h"
#include "grbl/canbus.h"

static CAN_HandleTypeDef hcan1 = {
    .Instance = CAN1,
    .Init.Mode = CAN_MODE_NORMAL,
    .Init.TimeTriggeredMode = DISABLE,
    .Init.AutoBusOff = DISABLE,
    .Init.AutoWakeUp = DISABLE,
    .Init.AutoRetransmission = DISABLE,
    .Init.ReceiveFifoLocked = DISABLE,
    .Init.TransmitFifoPriority = DISABLE
};
static can_rx_enqueue_fn rx_enqueue;
static can_rx_ptr rx_callbacks[28] = {0};

bool can_put (canbus_message_t message, bool ext_id)
{
    uint32_t TxMailbox;
    CAN_TxHeaderTypeDef TxHeader = {0};

    TxHeader.DLC = message.len;
    TxHeader.IDE = ext_id ? CAN_ID_EXT : CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    if(ext_id)
        TxHeader.ExtId = message.id;
    else
        TxHeader.StdId = message.id;

    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, message.data, &TxMailbox) == HAL_OK;
}

bool can_add_filter (uint32_t id, uint32_t mask, bool ext_id, can_rx_ptr callback)
{
    static uint8_t bank = 0, fmi = 0;
    static CAN_FilterTypeDef sFilterConfig = {
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .SlaveStartFilterBank = 14,
        .FilterActivation = ENABLE
    };

    if(bank == 14)
        return false;

    if(ext_id) {

        // not tested!

        id <<= CAN_RI0R_EXID_Pos;
        mask <<= CAN_RI0R_EXID_Pos;

        if(sFilterConfig.FilterScale == CAN_FILTERSCALE_16BIT &&
            sFilterConfig.FilterIdLow == sFilterConfig.FilterIdHigh &&
             sFilterConfig.FilterMaskIdLow == sFilterConfig.FilterMaskIdHigh)
            fmi++;

        sFilterConfig.FilterBank = bank;
        sFilterConfig.FilterIdLow = (uint16_t)id;
        sFilterConfig.FilterIdHigh = id >> 16;
        sFilterConfig.FilterMaskIdLow = (uint16_t)mask;
        sFilterConfig.FilterMaskIdHigh = mask >> 16;
        sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    } else {

        id <<= 5;
        mask <<= 5;

        if(fmi && sFilterConfig.FilterIdLow == sFilterConfig.FilterIdHigh && sFilterConfig.FilterMaskIdLow == sFilterConfig.FilterMaskIdHigh) {
            bank--;
            sFilterConfig.FilterIdHigh = id;
            sFilterConfig.FilterMaskIdHigh = mask;
        } else {
            sFilterConfig.FilterBank = bank;
            sFilterConfig.FilterIdLow = id;
            sFilterConfig.FilterMaskIdLow = mask;
            sFilterConfig.FilterIdHigh = id;
            sFilterConfig.FilterMaskIdHigh = mask;
            rx_callbacks[fmi + 1] = callback;
        }
        sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
    }
    rx_callbacks[fmi] = callback;
    fmi++;
    bank++;

    return HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) == HAL_OK;
}

bool can_stop (void)
{
    HAL_CAN_Stop(&hcan1);
    HAL_CAN_DeInit(&hcan1);

    return true;
}

bool can_set_baud (uint32_t baud)
{
    report_message("A hard reset of the controller is required after changing the CAN baud rate", Message_Info);

    return true;
}

bool can_start (uint32_t baud, can_rx_enqueue_fn callback)
{
    uint8_t unknown_rate = 0;

    rx_enqueue = callback;
    /*
     * Can bit time calculations taken from http://www.bittiming.can-wiki.info/ - pre-calculated
     * for supported CAN peripheral clock speeds and baud rates.
     *
     * Unable to query the CAN peripheral clock speed until it's initialised, however the
     * STM32F4 CAN peripheral appears to always be clocked from PCLK1/APB1.
     */
    uint32_t pclk1_freq = HAL_RCC_GetPCLK1Freq();

    switch (pclk1_freq) {

        case 25000000: // 25MHz APB1 clock

            switch (baud) {

                case 125000:
                    hcan1.Init.Prescaler = 20;
                    hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
                    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
                    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
                    break;

                case 250000:
                    hcan1.Init.Prescaler = 10;
                    hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
                    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
                    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
                    break;

                case 500000:
                    hcan1.Init.Prescaler = 5;
                    hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
                    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
                    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
                    break;

                default:
                    /* Unsupported baud rate */
                    unknown_rate = 1;
                    break;
            }
            break;

        case 45000000: // 45MHz APB1 clock

            switch (baud) {

                case 125000:
                    hcan1.Init.Prescaler = 20;
                    hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
                    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
                    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
                    break;

                case 250000:
                    hcan1.Init.Prescaler = 10;
                    hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
                    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
                    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
                    break;

                case 500000:
                    hcan1.Init.Prescaler = 5;
                    hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
                    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
                    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
                    break;

                case 1000000:
                    hcan1.Init.Prescaler = 3;
                    hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
                    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
                    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
                    break;

                default:
                    /* Unsupported baud rate */
                    unknown_rate = 1;
                    break;
            }
            break;

        default:
            /* unsupported CAN peripheral clock frequency */
            unknown_rate = 1;
            break;
    }

    if (unknown_rate) {
//        printf("can_start(), error - unable to calculate bit timings\n");
        return(0);
    }

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        return(0);
    }

    /* Start the CAN peripheral (calls the MspInit function) */
    if(HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return(0);
    }

    /* Add the callback for received data */

    return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;
}

/**
* @brief CAN MSP Initialization
* This function configures the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
*/
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
    if(hcan->Instance == CAN1)
    {
        __HAL_RCC_CAN1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = (1 << CAN_RX_PIN)|(1 << CAN_TX_PIN),
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF9_CAN1
        };

        HAL_GPIO_Init(CAN_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

        static const periph_pin_t rx = {
            .function = Input_RX,
            .group = PinGroup_CAN,
            .port = CAN_PORT,
            .pin = CAN_RX_PIN,
            .mode = { .mask = PINMODE_NONE },
            .description = "CAN"
        };

        static const periph_pin_t tx = {
            .function = Output_TX,
            .group = PinGroup_CAN,
            .port = CAN_PORT,
            .pin = CAN_TX_PIN,
            .mode = { .mask = PINMODE_OUTPUT },
            .description = "CAN"
        };

        hal.periph_port.register_pin(&rx);
        hal.periph_port.register_pin(&tx);
    }
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    canbus_message_t message;
    CAN_RxHeaderTypeDef RxHeader;

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {

        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, message.data) == HAL_OK) {

            /* Attempt to add incoming message to the RX queue.
             *
             * Note: not currently checking for success, if there is no space available we
             * would just end up dropping messages somewhere else (i.e. in the CAN RX mailbox)..
             */
            message.id = RxHeader.StdId;
            message.len = RxHeader.DLC;
            rx_enqueue(message, rx_callbacks[RxHeader.FilterMatchIndex]);
        }
    }
}

#endif // CAN_PORT
