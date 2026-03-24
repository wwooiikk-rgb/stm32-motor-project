#include "encoder.h"

void Encoder_Init(Encoder_t *enc)
{
    HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);
    enc->last_count = (int32_t)__HAL_TIM_GET_COUNTER(enc->htim);
    enc->rpm = 0.0f;
}

void Encoder_Update(Encoder_t *enc)
{
    int32_t current_count = (int32_t)__HAL_TIM_GET_COUNTER(enc->htim);
    int32_t delta;

    if (enc->is_32bit) {
        // TIM2: 32-bit counter, direct subtraction
        delta = current_count - enc->last_count;
    } else {
        // TIM4: 16-bit counter, cast to int16_t for wrap-around handling
        delta = (int16_t)(current_count - enc->last_count);
    }

    enc->last_count = current_count;

    // RPM = (delta / CPR) * (60000ms / DT_MS)
    enc->rpm = (float)delta * (60000.0f / ((float)ENCODER_CPR * ENCODER_DT_MS));
}

float Encoder_GetRPM(Encoder_t *enc)
{
    return enc->rpm;
}
