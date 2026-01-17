/*
 * motors.h
 *
 *  Created on: Nov 17, 2025
 *      Author: mbeng
 */

#ifndef DRIVER_MOTOR_H
#define DRIVER_MOTOR_H
#include "main.h"
#include "math.h"

#define ROBOT_V_MAX   0.30f

// Mode de fonctionnement du moteur
typedef enum {
    STANDBY_MODE = 0,
    FORWARD_MODE,
    REVERSE_MODE,
    BRAKE_MODE
} MotorMode;

// Structure principale
typedef struct {

    // Modes moteurs
    MotorMode mode_mot1;
    MotorMode mode_mot2;

    // Vitesse cible
    float target_speed1;  // PWM cible
    float target_speed2;

    // Vitesse courante
    float current_speed1; // PWM courant (rampe)
    float current_speed2;

    // Rampes
    int speed_ramp1;
    int speed_ramp2;

    // Handles timer
    TIM_HandleTypeDef *htim_pwm;

    // --- CONFIG SPECIFIQUE PAR MOTEUR ---
    uint32_t m1_forward_channel;   // TIM_CHANNEL_1
    uint32_t m1_reverse_channel;   // TIM_CHANNEL_2

    uint32_t m2_forward_channel;   // TIM_CHANNEL_4
    uint32_t m2_reverse_channel;   // TIM_CHANNEL_3

    //asservissement
    float target_vel_left;     // m/s  // consigne physique
    float target_vel_right;    // m/s

    float measured_vel_left;   // m/s   // vitesse réelle (encodeurs)
    float measured_vel_right;  // m/s


} h_Motor_t;


// Prototypes
void Motor_Init(h_Motor_t *hMotors, TIM_HandleTypeDef *htim);
void Motor_SetMode(h_Motor_t *hMotors); //
void Motor_SetSpeed_percent(h_Motor_t *hMotors, float m1_percent, float m2_percent);
void Motor_UpdateSpeed(h_Motor_t *hMotors);
void Motor_SetSpeed(h_Motor_t *hMotors, float speed);
void Motor_CommandVelLR(h_Motor_t *m, float v_left, float v_right);
#endif /* DRIVERS_MOTORS_H_ */
