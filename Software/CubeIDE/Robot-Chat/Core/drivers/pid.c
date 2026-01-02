/*
 * pid.c
 *
 *  Created on: Dec 11, 2025
 *      Author: mbeng
 */


#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float PID_Update(PID_t *pid, float error, float dt)
{
    // Intégrale avec anti-windup
    pid->integral += error * dt;
    if (pid->integral * pid->ki > pid->out_max) pid->integral = pid->out_max / pid->ki;
    else if (pid->integral * pid->ki < pid->out_min) pid->integral = pid->out_min / pid->ki;

    pid->prev_error = error;

    float out = pid->kp * error + pid->ki * pid->integral;

    // Saturation
    if (out > pid->out_max) out = pid->out_max;
    else if (out < pid->out_min) out = pid->out_min;

    return out;
}
