#include "motor.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

// --- Encoders ---
// TIM2: 32-bit, Motor1 / TIM4: 16-bit, Motor2
Encoder_t encoder1 = { .htim = &htim2, .is_32bit = 1 };
Encoder_t encoder2 = { .htim = &htim4, .is_32bit = 0 };

// --- PID controllers ---
// Initial tuning: Kp=0.15, Ki=0.8, Kd=0.02 (adjust as needed)
// Output range: 0~100 (PWM percent)
PID_t pid1;
PID_t pid2;

// --- Motors ---
// Motor1: TIM3 CH1 (motor) + CH3 (red LED) + BLUE_LED1 (GPIO)
// Motor2: TIM3 CH2 (motor) + CH4 (red LED) + BLUE_LED2 (GPIO)
Motor_t motor1 = {
    .htim          = &htim3,
    .motor_channel = TIM_CHANNEL_1,
    .red_channel   = TIM_CHANNEL_3,
    .blue_port     = BLUE_LED1_GPIO_Port,
    .blue_pin      = BLUE_LED1_Pin,
    .encoder       = &encoder1,
    .pid           = &pid1,
    .target_rpm    = 0.0f
};

Motor_t motor2 = {
    .htim          = &htim3,
    .motor_channel = TIM_CHANNEL_2,
    .red_channel   = TIM_CHANNEL_4,
    .blue_port     = BLUE_LED2_GPIO_Port,
    .blue_pin      = BLUE_LED2_Pin,
    .encoder       = &encoder2,
    .pid           = &pid2,
    .target_rpm    = 0.0f
};

void Motor_Init(void)
{
    // TB6612FNG: enable driver
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

    // Single direction: AIN1=HIGH, AIN2=LOW / BIN1=HIGH, BIN2=LOW
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);

    // Start motor + LED PWM channels
    HAL_TIM_PWM_Start(motor1.htim, motor1.motor_channel);
    HAL_TIM_PWM_Start(motor1.htim, motor1.red_channel);
    HAL_TIM_PWM_Start(motor2.htim, motor2.motor_channel);
    HAL_TIM_PWM_Start(motor2.htim, motor2.red_channel);

    // Init encoders
    Encoder_Init(motor1.encoder);
    Encoder_Init(motor2.encoder);

    // Init PID controllers (output: 0~100%)
    PID_Init(motor1.pid, 0.15f, 0.80f, 0.02f, 0.0f, 100.0f);
    PID_Init(motor2.pid, 0.15f, 0.80f, 0.02f, 0.0f, 100.0f);

    // Initial state: stopped -> blue LED ON
    HAL_GPIO_WritePin(motor1.blue_port, motor1.blue_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor2.blue_port, motor2.blue_pin, GPIO_PIN_SET);
}

void Motor_SetSpeed(Motor_t *motor, uint8_t percent)
{
    if (percent > 100) percent = 100;
    uint32_t pulse = (MOTOR_PWM_MAX * percent) / 100;

    __HAL_TIM_SET_COMPARE(motor->htim, motor->motor_channel, pulse);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->red_channel, pulse);

    if (percent > 0) {
        // Running: red LED ON (brightness = speed), blue LED OFF
        HAL_GPIO_WritePin(motor->blue_port, motor->blue_pin, GPIO_PIN_RESET);
    } else {
        // Stopped: red LED OFF, blue LED ON
        HAL_GPIO_WritePin(motor->blue_port, motor->blue_pin, GPIO_PIN_SET);
    }
}

void Motor_SetTargetRPM(Motor_t *motor, float rpm)
{
    if (rpm < 0.0f) rpm = 0.0f;
    motor->target_rpm = rpm;

    if (rpm == 0.0f) {
        Motor_Stop(motor);
        PID_Reset(motor->pid);
    }
}

void Motor_Update(Motor_t *motor)
{
    Encoder_Update(motor->encoder);

    if (motor->target_rpm <= 0.0f) {
        Motor_Stop(motor);
        PID_Reset(motor->pid);
        return;
    }

    float actual_rpm = Encoder_GetRPM(motor->encoder);
    float output     = PID_Compute(motor->pid, motor->target_rpm, actual_rpm);
    Motor_SetSpeed(motor, (uint8_t)output);
}

void Motor_Stop(Motor_t *motor)
{
    Motor_SetSpeed(motor, 0);
}

void Motor_StopAll(void)
{
    Motor_Stop(&motor1);
    Motor_Stop(&motor2);
}
