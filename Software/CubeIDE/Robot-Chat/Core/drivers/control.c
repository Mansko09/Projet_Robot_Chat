/*
 * control.c
 *
 *  Created on: Dec 11, 2025
 *      Author: mbeng
 */

#include "control.h"



void Control_UpdateSpeed(h_control_t *c, const Encodeur_t *enc, const Odom_Params_t *p, float dt)
{
    // distance roue gauche / droite (m)
    float dl = 2.0f * M_PI * p->wheel_radius * ((float)enc->delta_1 / p->ticks_per_rev);
    float dr = 2.0f * M_PI * p->wheel_radius * ((float)enc->delta_2 / p->ticks_per_rev);

    // vitesse = distance / temps
    c->v_left_meas = dl / dt;
    c->v_right_meas = dr / dt;
}

// ------------------------------------------------------------
// PID → COMMANDE MOTEURS
// ------------------------------------------------------------
void Control_RunPID(h_control_t *c, float dt, float wheel_base)
{
    // Conversion (v, w) → vitesses roue
    float v_l_ref = c->v_ref - 0.5f * wheel_base * c->w_ref;
    float v_r_ref = c->v_ref + 0.5f * wheel_base * c->w_ref;

    // Erreurs
    float err_l = v_l_ref - c->v_left_meas;
    float err_r = v_r_ref - c->v_right_meas;

    // PID → commande PWM [-100, 100]
    float pwm_l = PID_Update(&c->pid_left, err_l, dt);
    float pwm_r = PID_Update(&c->pid_right, err_r, dt);

    // Mise à jour des PWM cibles et modes
    if(pwm_l >= 0.0f) {
        if(pwm_l > 100.0f) pwm_l = 100.0f;
        c->hMotors.mode_mot1 = FORWARD_MODE;
        c->hMotors.target_speed1 = pwm_l;
    } else {
        if(pwm_l < -100.0f) pwm_l = -100.0f;
        c->hMotors.mode_mot1 = REVERSE_MODE;
        c->hMotors.target_speed1 = -pwm_l;
    }

    if(pwm_r >= 0.0f) {
        if(pwm_r > 100.0f) pwm_r = 100.0f;
        c->hMotors.mode_mot2 = FORWARD_MODE;
        c->hMotors.target_speed2 = pwm_r;
    } else {
        if(pwm_r < -100.0f) pwm_r = -100.0f;
        c->hMotors.mode_mot2 = REVERSE_MODE;
        c->hMotors.target_speed2 = -pwm_r;
    }

    // Appliquer la commande
    Motor_SetMode(&c->hMotors);
    Motor_SetSpeed_percent(&c->hMotors, c->hMotors.target_speed1, c->hMotors.target_speed2);
}



void Control_SetSpeed(h_control_t *c, float v, float w)
{
    c->v_ref = v;   // m/s
    c->w_ref = w;   // rad/s
}


