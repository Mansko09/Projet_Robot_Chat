/*
 * control.h
 *
 *  Created on: Dec 9, 2025
 *      Author: mbeng
 */

#ifndef DRIVERS_CONTROL_H_
#define DRIVERS_CONTROL_H_
#include "TOFs.h"
#include "ADXL.h"
#include "I2C_mux.h"
#include "motors.h"
#include "odometry.h"
#include "pid.h"
#include "ADXL.h" //accelero
#define CTRL_PERIOD_MS  10
#define CTRL_DT         0.01f   // 10 ms

//Controle central FREERTOS
typedef struct {
	h_Motor_t hMotors;
	Odom_t odom;

    PID_t pid_left;
    PID_t pid_right;

    float v_left_meas;    // m/s
    float v_right_meas;   // m/s

    float v_ref;          // commande vitesse linéaire (m/s)
    float w_ref;          // commande vitesse angulaire (rad/s)

	int AccData;//pour l'accéléromètre, égal à 0 ou 1 si choc
	int vide; //variable qui contient 0 (si pas vide), 1, 2, 3 ou 4 qui correspondent au numéro du tof impliqué

}h_control_t;
void Control_RunPID(h_control_t *c, float dt,float wheel_base);
void Control_UpdateSpeed(h_control_t *c, const Encodeur_t *enc, const Odom_Params_t *p, float dt);
void Control_SetSpeed(h_control_t *c, float v, float w);
#endif /* DRIVERS_CONTROL_H_ */
