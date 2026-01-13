/*
 * motors.c
 *
 *  Created on: Nov 17, 2025
 *      Author: mbeng
 */

#include "motors.h"
#include "tim.h"
#include <stdio.h>

// ---------------------------------------------------------------------------
// INIT
// ---------------------------------------------------------------------------
void Motor_Init(h_Motor_t *hMotors, TIM_HandleTypeDef *htim)
{
    hMotors->htim_pwm = htim;

    // Ramp settings
    hMotors->speed_ramp1 = 20;
    hMotors->speed_ramp2 = 20;

    hMotors->current_speed1 = 0.0f;
    hMotors->current_speed2 = 0.0f;
    hMotors->target_speed1  = 0.0f;
    hMotors->target_speed2  = 0.0f;

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

    hMotors->target_speed1 = (ARR * (m1_percent / 100.0f));
    hMotors->target_speed2 = (ARR * (m2_percent / 100.0f));
}

// ---------------------------------------------------------------------------
void Motor_SetSpeed(h_Motor_t *hMotors, float speed)
{
    if (speed >  ROBOT_V_MAX) speed =  ROBOT_V_MAX;
    if (speed < -ROBOT_V_MAX) speed = -ROBOT_V_MAX;

    // Conversion m/s -> duty%
    float speed_abs = fabsf(speed);
    float duty_percent = (speed_abs / ROBOT_V_MAX) * 100.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    // On ne change PAS le mode ici (sinon inversion instantanée)
    // On stocke juste la consigne physique
    hMotors->target_vel_left  = speed;
    hMotors->target_vel_right = speed;

    // On met juste la PWM cible
    Motor_SetSpeed_percent(hMotors, duty_percent, duty_percent);
}

// ---------------------------------------------------------------------------
// Helpers for LR command
// ---------------------------------------------------------------------------
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float vel_to_pwm_target(float v)
{
    float v_abs = fabsf(v);
    float duty = (v_abs / ROBOT_V_MAX) * 100.0f;
    if (duty > 100.0f) duty = 100.0f;
    return duty; // percent
}

static inline int pwm_near_zero(float pwm)
{
    return (pwm < 100.0f); // seuil à ajuster (counts PWM)
}

static inline float pwm_to_percent(h_Motor_t *m, float pwm)
{
    float ARR = (float)m->htim_pwm->Init.Period;
    if (ARR <= 0.0f) return 0.0f;
    return (pwm / ARR) * 100.0f;
}

void Motor_CommandVelLR(h_Motor_t *m, float v_left, float v_right)
{
    // 1. Saturation
    v_left  = clampf(v_left,  -ROBOT_V_MAX, ROBOT_V_MAX);
    v_right = clampf(v_right, -ROBOT_V_MAX, ROBOT_V_MAX);

    // 2. Déterminer les modes cibles selon le signe de la vitesse demandée
    MotorMode target_mode1 = (v_left > 0.01f)  ? FORWARD_MODE : (v_left < -0.01f) ? REVERSE_MODE : STANDBY_MODE;
    MotorMode target_mode2 = (v_right > 0.01f) ? FORWARD_MODE : (v_right < -0.01f) ? REVERSE_MODE : STANDBY_MODE;

    uint32_t ARR = m->htim_pwm->Init.Period;

    // --- GESTION MOTEUR 1 ---
    if (m->mode_mot1 != target_mode1) {
        // On veut changer de sens ou s'arrêter : on met la cible PWM à 0
        m->target_speed1 = 0.0f;
        // On ne change le mode QUE si le moteur est pratiquement arrêté
        if (m->current_speed1 < 100.0f) {
            m->mode_mot1 = target_mode1;
            Motor_SetMode(m); // Applique physiquement le nouveau sens
        }
    } else {
        // On est déjà dans le bon sens, on applique la consigne de vitesse
        m->target_speed1 = fabsf(v_left) / ROBOT_V_MAX * ARR;
    }

    // --- GESTION MOTEUR 2 ---
    if (m->mode_mot2 != target_mode2) {
        m->target_speed2 = 0.0f;
        if (m->current_speed2 < 100.0f) {
            m->mode_mot2 = target_mode2;
            Motor_SetMode(m);
        }
    } else {
        m->target_speed2 = fabsf(v_right) / ROBOT_V_MAX * ARR;
    }

    // Sauvegarde des consignes pour l'asservissement
    m->target_vel_left  = v_left;
    m->target_vel_right = v_right;
}

// ---------------------------------------------------------------------------
void Motor_Stop(h_Motor_t *hMotors)
{
    hMotors->mode_mot1 = STANDBY_MODE;
    hMotors->mode_mot2 = STANDBY_MODE;

    hMotors->target_speed1 =  0.0f;
    hMotors->target_speed2 =  0.0f;

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
                          hMotors->mode_mot1 == FORWARD_MODE ? (uint32_t)hMotors->current_speed1 : 0);

    __HAL_TIM_SET_COMPARE(hMotors->htim_pwm,
                          hMotors->m1_reverse_channel,
                          hMotors->mode_mot1 == REVERSE_MODE ? (uint32_t)hMotors->current_speed1 : 0);

    __HAL_TIM_SET_COMPARE(hMotors->htim_pwm,
                          hMotors->m2_forward_channel,
                          hMotors->mode_mot2 == FORWARD_MODE ? (uint32_t)hMotors->current_speed2 : 0);

    __HAL_TIM_SET_COMPARE(hMotors->htim_pwm,
                          hMotors->m2_reverse_channel,
                          hMotors->mode_mot2 == REVERSE_MODE ? (uint32_t)hMotors->current_speed2 : 0);
}
