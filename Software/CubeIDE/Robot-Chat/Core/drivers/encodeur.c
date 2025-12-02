/*
 * encodeur.c
 *
 *  Created on: Dec 2, 2025
 *      Author: mbeng
 */


#include "encodeur.h"

static int32_t last_tim2 = 0;
static int32_t last_lptim1 = 0;

void Encodeur_Init(void)
{
    // Reset software
    last_tim2 = 0;
    last_lptim1 = 0;

    // Reset hardware counters
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_LPTIM_Counter_Start(&hlptim1,0xFFFF); // si pas déjà lancé par CubeMX

    // Start encoder interfaces
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    // Mode X4 : lecture sur les 2 signaux
    HAL_LPTIM_Encoder_Start(&hlptim1, 0xFFFF);
}

void Encodeur_Read(Encodeur_t *enc)
{
    // Lire les 2 compteurs
    int32_t now_left  = __HAL_TIM_GET_COUNTER(&htim2);
    int32_t now_right = HAL_LPTIM_ReadCounter(&hlptim1);

    // Calculer les deltas (signed)
    int16_t d_left  = (int16_t)(now_left  - last_tim2);
    int16_t d_right = (int16_t)(now_right - last_lptim1);

    last_tim2   = now_left;
    last_lptim1 = now_right;

    enc->delta_left  = d_left;
    enc->delta_right = d_right;

    // Cumuls
    enc->total_left  += d_left;
    enc->total_right += d_right;
}
