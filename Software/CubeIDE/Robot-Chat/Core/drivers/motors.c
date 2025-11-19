/*
 * motors.c
 *
 *  Created on: Nov 17, 2025
 *      Author: mbeng
 */

#include "motors.h"
#include "tim.h"
#include <stdio.h>

#define DEBUG 0
#if DEBUG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

// ---------------------------------------------------------------------------
// INIT
// ---------------------------------------------------------------------------
void Motor_Init(h_Motor_t *hMotors, TIM_HandleTypeDef *htim)
{
    hMotors->htim_pwm = htim;

    // Ramp settings
    hMotors->speed_ramp1 = 20;
    hMotors->speed_ramp2 = 20;

    hMotors->current_speed1 = 0;
    hMotors->current_speed2 = 0;
    hMotors->target_speed1  = 0;
    hMotors->target_speed2  = 0;

    hMotors->mode_mot1 = STANDBY_MODE;
    hMotors->mode_mot2 = STANDBY_MODE;

    // Start PWM on ALL channels used
    HAL_TIM_PWM_Start(htim, hMotors->m1_forward_channel);
    HAL_TIM_PWM_Start(htim, hMotors->m1_reverse_channel);
    HAL_TIM_PWM_Start(htim, hMotors->m2_forward_channel);
    HAL_TIM_PWM_Start(htim, hMotors->m2_reverse_channel);

    Motor_SetMode(hMotors);
}

// ---------------------------------------------------------------------------
// MOTOR MODE SWITCH
// ---------------------------------------------------------------------------
void Motor_SetMode(h_Motor_t *hMotors)
{
    uint32_t ARR = hMotors->htim_pwm->Init.Period;

    // ----------------- MOT1 -------------------
    switch (hMotors->mode_mot1)
    {
        case FORWARD_MODE:
            __HAL_TIM_SET_COMPARE(hMotors->htim_pwm, hMotors->m1_reverse_channel, 0);
            break;

        case REVERSE_MODE:
            __HAL_TIM_SET_COMPARE(hMotors->htim_pwm, hMotors->m1_forward_channel, 0);
            break;

        case BRAKE_MODE:
            __HAL_TIM_SET_COMPARE(hMotors->htim_pwm, hMotors->m1_forward_channel, ARR);
            __HAL_TIM_SET_COMPARE(hMotors->htim_pwm, hMotors->m1_reverse_channel, ARR);
            break;

        default: // STANDBY
            __HAL_TIM_SET_COMPARE(hMotors->htim_pwm, hMotors->m1_forward_channel, 0);
            __HAL_TIM_SET_COMPARE(hMotors->htim_pwm, hMotors->m1_reverse_channel, 0);
            break;
    }

    // ----------------- MOT2 -------------------
    switch (hMotors->mode_mot2)
    {
        case FORWARD_MODE:
            __HAL_TIM_SET_COMPARE(hMotors->htim_pwm, hMotors->m2_reverse_channel, 0);
            break;

        case REVERSE_MODE:
            __HAL_TIM_SET_COMPARE(hMotors->htim_pwm, hMotors->m2_forward_channel, 0);
            break;

        case BRAKE_MODE:
            __HAL_TIM_SET_COMPARE(hMotors->htim_pwm, hMotors->m2_forward_channel, ARR);
            __HAL_TIM_SET_COMPARE(hMotors->htim_pwm, hMotors->m2_reverse_channel, ARR);
            break;

        default: // STANDBY
            __HAL_TIM_SET_COMPARE(hMotors->htim_pwm, hMotors->m2_forward_channel, 0);
            __HAL_TIM_SET_COMPARE(hMotors->htim_pwm, hMotors->m2_reverse_channel, 0);
            break;
    }
}

// ---------------------------------------------------------------------------
void Motor_SetSpeed_percent(h_Motor_t *hMotors, float m1_percent, float m2_percent)
{
    uint16_t ARR = hMotors->htim_pwm->Init.Period;

    hMotors->target_speed1 = (int)(ARR * (m1_percent / 100.0f));
    hMotors->target_speed2 = (int)(ARR * (m2_percent / 100.0f));
}

// ---------------------------------------------------------------------------
void Motor_Stop(h_Motor_t *hMotors)
{
    hMotors->mode_mot1 = STANDBY_MODE;
    hMotors->mode_mot2 = STANDBY_MODE;

    hMotors->target_speed1 = 0;
    hMotors->target_speed2 = 0;

    Motor_SetMode(hMotors);
}

// ---------------------------------------------------------------------------
void Motor_UpdateSpeed(h_Motor_t *hMotors)
{
    // M1 ramp
    if (hMotors->current_speed1 < hMotors->target_speed1)
        hMotors->current_speed1 += hMotors->speed_ramp1;
    else if (hMotors->current_speed1 > hMotors->target_speed1)
        hMotors->current_speed1 -= hMotors->speed_ramp1;

    // M2 ramp
    if (hMotors->current_speed2 < hMotors->target_speed2)
        hMotors->current_speed2 += hMotors->speed_ramp2;
    else if (hMotors->current_speed2 > hMotors->target_speed2)
        hMotors->current_speed2 -= hMotors->speed_ramp2;

    // Apply PWM
    __HAL_TIM_SET_COMPARE(hMotors->htim_pwm,
                          hMotors->m1_forward_channel,
                          hMotors->mode_mot1 == FORWARD_MODE ? hMotors->current_speed1 : 0);

    __HAL_TIM_SET_COMPARE(hMotors->htim_pwm,
                          hMotors->m1_reverse_channel,
                          hMotors->mode_mot1 == REVERSE_MODE ? hMotors->current_speed1 : 0);

    __HAL_TIM_SET_COMPARE(hMotors->htim_pwm,
                          hMotors->m2_forward_channel,
                          hMotors->mode_mot2 == FORWARD_MODE ? hMotors->current_speed2 : 0);

    __HAL_TIM_SET_COMPARE(hMotors->htim_pwm,
                          hMotors->m2_reverse_channel,
                          hMotors->mode_mot2 == REVERSE_MODE ? hMotors->current_speed2 : 0);
}
