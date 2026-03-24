#ifndef PID_H
#define PID_H

#include "encoder.h"    // for ENCODER_DT_MS

#define PID_DT_S    (ENCODER_DT_MS / 1000.0f)  // 0.01s

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max);
float PID_Compute(PID_t *pid, float setpoint, float measurement);
void  PID_Reset(PID_t *pid);

#endif
