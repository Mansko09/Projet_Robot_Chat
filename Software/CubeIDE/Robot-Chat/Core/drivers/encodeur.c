/*
 * encodeur.c
 *
 *  Created on: Dec 2, 2025
 *      Author: mbeng
 */


#include "encodeur.h"

static int32_t last_tim2 = 0;
static int32_t last_lptim1 = 0;

/*
 * Initialisation des timers des encodeurs
 */
void Encodeur_Init(void)
{
    // Reset software
    last_tim2 = 0;
    last_lptim1 = 0;

    // Reset hardware counters
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_LPTIM_Counter_Start(&hlptim1,0xFFFF);

    // Start encoder interfaces
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // c le Global Capture/compare channel identifier

    HAL_LPTIM_Encoder_Start(&hlptim1, 0xFFFF);
}

/*
 * mesure la variation des ticks depuis la dernière lecture pour chaque roue,
 * gère les débordements des compteurs
 * et accumule la distance totale parcourue en corrigeant l'inversion de sens d'un des deux encodeurs.
 */
void Encodeur_Read(Encodeur_t *enc)
{
    uint16_t now_1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t now_2 = (uint16_t)HAL_LPTIM_ReadCounter(&hlptim1);

    int16_t d1 = (int16_t)(now_1 - last_tim2);   // SUB 16 bits → automatique modulo 65536
    int16_t d2 = (int16_t)(now_2 - last_lptim1);

    last_tim2   = now_1;
    last_lptim1 = now_2;

    enc->delta_1  = d1;
    enc->delta_2 = -d2;      // inversion du signe car encodeur droit incrémente négativement

    enc->total_1 += enc->delta_1;
    enc->total_2 += enc->delta_2;
}

