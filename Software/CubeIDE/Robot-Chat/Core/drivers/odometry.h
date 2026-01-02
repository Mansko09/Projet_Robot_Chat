/*
 * odometry.h
 *
 *  Created on: Dec 2, 2025
 *      Author: mbeng
 */

#ifndef DRIVERS_ODOMETRY_H_
#define DRIVERS_ODOMETRY_H_

#include <stdint.h>
#include <math.h>
#include "encodeur.h"

typedef struct {
    float wheel_radius;   // Rayon roue (m)
    float wheel_base;     // Distance entre roues (m)
    float ticks_per_rev;  // Nombre de ticks par tour
} Odom_Params_t;

typedef struct {
    float x;        // Position en mètres
    float y;
    float theta;    // Orientation en radians
    Odom_Params_t odom_params;
} Odom_t;

/**
 * @brief Initialise la structure odométrie
 */
void Odom_Init(Odom_t *odom);

/**
 * @brief Met à jour la position x,y,theta
 * @param odom structure position
 * @param enc nouvelles mesures encodeurs
 * @param p paramètres physiques
 */
void Odom_Update(Odom_t *odom, const Encodeur_t *enc, const Odom_Params_t *p);


#endif /* DRIVERS_ODOMETRY_H_ */
