#include <math.h>
#include "main.h"
#include "wloop_can.h"
#include "application_code.h"

void init_timer(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->ARR = 0xFFFF;

    TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    TIM3->CR1 |= TIM_CR1_CEN;
}

float compute_rpm(int n) {
    return (2*M_PI*R*n)/(N*SAMPLING_PERIOD_S);
}

float* velocity;
uint8_t bytes_array[4];
CAN_TxHeaderTypeDef TxHeader;
extern uint32_t TxMailbox;

int app_main() {
    if (CANBus_init(&hcan) != HAL_OK) { Error_Handler(); }
    // if (CANBus_subscribe(POD_SPEED) != HAL_OK) { Error_Handler(); }
    if (CANBus_subscribe_all() != HAL_OK) { Error_Handler(); }

    // CANFrame tx_frame = CANFrame_init(POD_SPEED.id);
    // CANFrame_set_field(&tx_frame, POD_SPEED, FLOAT_TO_UINT(69.420));

    // hcan.Instance->MCR |= (1 << 16);

    CANFrame tx_frame;
    while (1) {
        // if (CANBus_put_frame(&tx_frame) != HAL_OK) { Error_Handler(); }

        if (!Queue_empty(&RX_QUEUE)) {
            CANFrame rx_frame = CANBus_get_frame();
            
            tx_frame = CANFrame_init(rx_frame.id);
            for (uint8_t i = 0; i < 8; i++) {
                tx_frame.pld[i] = rx_frame.pld[i];
            }
            if (CANBus_put_frame(&tx_frame) != HAL_OK) { Error_Handler(); }
        }
        HAL_Delay(5);
    }

    return 0;
}

// int app_main() {
//     init_timer();
// 
//     HAL_CAN_Start(&hcan);
//     HAL_TIM_Base_Start_IT(&htim14);
//     __HAL_TIM_SET_COUNTER(&htim14, 0);
// 
//     while (1) {
//         int count = TIM3->CNT;
//         *velocity = compute_rpm(count);
//         add_data(velocity, bytes_array, 32, 1, 0);
//         HAL_CAN_AddTxMessage(&hcan, &TxHeader, bytes_array, &TxMailbox);
//         while (HAL_CAN_IsTxMessagePending(&hcan, TxMailbox));
// 
//         HAL_Delay(100);
//     }
// 
//     return 0;
// }


