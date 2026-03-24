#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"
#include "encoder.h"
#include "pid.h"

#define MOTOR_PWM_MAX   17871   // ARR value (TIM3, 84MHz, PSC=0 -> 4700.6Hz)

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           motor_channel;   // PWM for motor speed
    uint32_t           red_channel;     // PWM for red LED (speed linked)
    GPIO_TypeDef      *blue_port;       // GPIO for blue LED
    uint16_t           blue_pin;        // GPIO for blue LED
    Encoder_t         *encoder;         // encoder for this motor
    PID_t             *pid;             // PID controller for this motor
    float              target_rpm;      // desired speed (RPM)
} Motor_t;

void Motor_Init(void);
void Motor_SetTargetRPM(Motor_t *motor, float rpm);     // PID speed control
void Motor_Update(Motor_t *motor);                      // call every 10ms in TIM6 ISR
void Motor_SetSpeed(Motor_t *motor, uint8_t percent);   // direct PWM (0~100%)
void Motor_Stop(Motor_t *motor);
void Motor_StopAll(void);

#endif
