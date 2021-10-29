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

int app_main() {
    if (CANBus_init(&hcan) != HAL_OK) { Error_Handler(); }

    init_timer();
    __HAL_TIM_Base_Start_IT(&htim14);

    int count;
    float rpm;
    CANFrame tx_frame = CANFrame_init(POD_SPEED.id);
    while (1) {
        count = TIM3->CNT;
        rpm = compute_rpm(count);

        CANFrame_set_field(&tx_frame, POD_SPEED, FLOAT_TO_UINT(rpm));
        if (CANBus_put_frame(&tx_frame) != HAL_OK) {
            Error_Handler();
        }

        HAL_Delay(100);
    }

    return 0;
}

