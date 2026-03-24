#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx_hal.h"

// JGA25-370: 11 PPR * 4 (quadrature) * 30 (gear ratio) = 1320 CPR
#define ENCODER_CPR         1320
#define ENCODER_DT_MS       10      // update interval: 10ms (100Hz)

typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t            last_count;
    float              rpm;
    uint8_t            is_32bit;    // 1: TIM2 (32-bit), 0: TIM4 (16-bit)
} Encoder_t;

void  Encoder_Init(Encoder_t *enc);
void  Encoder_Update(Encoder_t *enc);   // call every ENCODER_DT_MS in timer ISR
float Encoder_GetRPM(Encoder_t *enc);

#endif
