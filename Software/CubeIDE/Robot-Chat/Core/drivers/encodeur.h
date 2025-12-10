/*
 * encodeur.h
 *
 *  Created on: Dec 2, 2025
 *      Author: mbeng
 */

#ifndef DRIVERS_ENCODEUR_H_
#define DRIVERS_ENCODEUR_H_


#include "tim.h"
#include "lptim.h"
#include <stdint.h>

typedef struct {
    int16_t delta_1;      // ticks depuis dernière lecture, encodeur 1
    int16_t delta_2;     // ticks depuis dernière lecture, encodeur 2
    int32_t total_1;      // ticks cumulés, encodeur 1
    int32_t total_2;     // ticks cumulés, encodeur 2
} Encodeur_t;

/**
 * @brief  Initialise les encodeurs : reset counters et valeurs internes
 */
void Encodeur_Init(void);

/**
 * @brief  Lit les 2 compteurs, calcule les deltas et les cumuls.
 * @param  enc structure où stocker les valeurs
 */
void Encodeur_Read(Encodeur_t *enc);

#endif /* DRIVERS_ENCODEUR_H_ */
