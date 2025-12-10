/*
 * control.h
 *
 *  Created on: Dec 9, 2025
 *      Author: mbeng
 */

#ifndef DRIVERS_CONTROL_H_
#define DRIVERS_CONTROL_H_
#include "TOFs.h"
#include "motors.h"
#include "odometry.h"

typedef struct {
	h_Motor_t hMotors;
	h_tof_t hTof;
	Odom_t odom;
	int AccData; //pour l'accéléromètre, égal à 0 ou 1 si choc
	int vide; //variable qui contient 0 (si pas vide), 1, 2, 3 ou 4 qui correspondent au numéro du tof impliqué

}h_control_t;

#endif /* DRIVERS_CONTROL_H_ */
