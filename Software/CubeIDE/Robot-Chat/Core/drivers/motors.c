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
    return (pwm < 2.0f); // seuil à ajuster (counts PWM)
}

static inline float pwm_to_percent(h_Motor_t *m, float pwm)
{
    float ARR = (float)m->htim_pwm->Init.Period;
    if (ARR <= 0.0f) return 0.0f;
    return (pwm / ARR) * 100.0f;
}

// ---------------------------------------------------------------------------
// Commande vitesses gauche/droite (m/s) avec rampe + inversion douce
//  - si inversion demandée : rampe à 0, puis switch de sens, puis rampe vers cible
//  - la rampe réelle est appliquée dans Motor_UpdateSpeed()
// ---------------------------------------------------------------------------
void Motor_CommandVelLR(h_Motor_t *m, float v_left, float v_right)
{
    // Saturation physique
    v_left  = clampf(v_left,  -ROBOT_V_MAX, ROBOT_V_MAX);
    v_right = clampf(v_right, -ROBOT_V_MAX, ROBOT_V_MAX);

    // Sens désiré par roue
    MotorMode desired1 = (v_left  > 0.0f) ? FORWARD_MODE :
                         (v_left  < 0.0f) ? REVERSE_MODE : STANDBY_MODE;

    MotorMode desired2 = (v_right > 0.0f) ? FORWARD_MODE :
                         (v_right < 0.0f) ? REVERSE_MODE : STANDBY_MODE;

    // PWM cible (en %)
    float duty1 = vel_to_pwm_target(v_left);
    float duty2 = vel_to_pwm_target(v_right);

    // On veut éviter d'écraser la target de l'autre moteur car Motor_SetSpeed_percent()
    // modifie les 2 en même temps => on "préserve" l'autre par conversion pwm->%
    float keep2 = pwm_to_percent(m, m->target_speed2);
    float keep1 = pwm_to_percent(m, m->target_speed1);

    // --- Gestion inversion moteur 1 ---
    if (desired1 != STANDBY_MODE && m->mode_mot1 != desired1)
    {
        // rampe vers 0 avant inversion (on garde la consigne de M2)
        Motor_SetSpeed_percent(m, 0.0f, keep2);

        if (pwm_near_zero(m->current_speed1))
        {
            m->mode_mot1 = desired1;
            Motor_SetMode(m);

            // puis appliquer la nouvelle consigne de M1
            Motor_SetSpeed_percent(m, duty1, keep2);
        }
    }
    else
    {
        // pas d'inversion : appliquer target PWM (si STANDBY -> duty=0)
        if (desired1 == STANDBY_MODE && pwm_near_zero(m->current_speed1))
        {
            m->mode_mot1 = STANDBY_MODE;
            Motor_SetMode(m);
        }
        else if (desired1 != STANDBY_MODE)
        {
            m->mode_mot1 = desired1;
            Motor_SetMode(m);
        }

        Motor_SetSpeed_percent(m, duty1, keep2);
    }

    // Recalculer keep1 après éventuelle modif
    keep1 = pwm_to_percent(m, m->target_speed1);

    // --- Gestion inversion moteur 2 ---
    if (desired2 != STANDBY_MODE && m->mode_mot2 != desired2)
    {
        // rampe vers 0 avant inversion (on garde la consigne de M1)
        Motor_SetSpeed_percent(m, keep1, 0.0f);

        if (pwm_near_zero(m->current_speed2))
        {
            m->mode_mot2 = desired2;
            Motor_SetMode(m);

            // puis appliquer la nouvelle consigne de M2
            Motor_SetSpeed_percent(m, keep1, duty2);
        }
    }
    else
    {
        if (desired2 == STANDBY_MODE && pwm_near_zero(m->current_speed2))
        {
            m->mode_mot2 = STANDBY_MODE;
            Motor_SetMode(m);
        }
        else if (desired2 != STANDBY_MODE)
        {
            m->mode_mot2 = desired2;
            Motor_SetMode(m);
        }

        Motor_SetSpeed_percent(m, keep1, duty2);
    }

    // garder la consigne physique (optionnel)
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
