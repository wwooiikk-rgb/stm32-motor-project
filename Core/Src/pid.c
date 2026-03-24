#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max)
{
    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = min;
    pid->output_max = max;
}

float PID_Compute(PID_t *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    // Integral with anti-windup clamping
    pid->integral += error * PID_DT_S;
    if (pid->integral * pid->ki > pid->output_max) pid->integral = pid->output_max / pid->ki;
    if (pid->integral * pid->ki < pid->output_min) pid->integral = pid->output_min / pid->ki;

    float derivative = (error - pid->prev_error) / PID_DT_S;
    pid->prev_error = error;

    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    // Clamp output
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}
