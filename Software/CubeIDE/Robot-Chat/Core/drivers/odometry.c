/*
 * odometry.c
 *
 *  Created on: Dec 2, 2025
 *      Author: mbeng
 */


#include "odometry.h"

void Odom_Init(Odom_t *odom)
{
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
}

/*
 * calcul de la position du robot grâce aux encodeurs
 */
void Odom_Update(Odom_t *odom, const Encodeur_t *enc, const Odom_Params_t *p)
{
    // Convertir ticks -> distance par roue
    float dl = 2.0f * M_PI * p->wheel_radius * ((float)enc->delta_1  / p->ticks_per_rev);
    float dr = 2.0f* M_PI * p->wheel_radius * ((float)enc->delta_2 / p->ticks_per_rev);

    // Mouvement du centre
    float d_center = (dr + dl) / 2.0f;

    // Variation angulaire
    float d_theta = (dr - dl) / p->wheel_base;

    // Mise à jour orientation
    odom->theta += d_theta;

    // Normalisation
    if (odom->theta > M_PI) odom->theta -= 2*M_PI;
    else if (odom->theta < -M_PI) odom->theta += 2*M_PI;

    // Mise à jour position x,y
    odom->x += d_center * cosf(odom->theta); //en mètres
    odom->y += d_center * sinf(odom->theta); //en mètres

}
