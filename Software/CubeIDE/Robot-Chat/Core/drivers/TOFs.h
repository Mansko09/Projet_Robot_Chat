/*
 * TOFs.h
 *
 *  Created on: Nov 19, 2025
 *      Author: mbeng
 */

#ifndef DRIVERS_TOFS_H_
#define DRIVERS_TOFS_H_


#include "../VL53L0X_API/core/vl53l0x_api.h"
#include "I2C_mux.h"
#include <stdio.h>
#include <stdlib.h>

#define VL53L0X_DEFAULT_ADDRESS 0x29
extern I2C_HandleTypeDef hi2c3;



void configure_TOF(uint8_t addr);
int TOF_Init();
int data_read_TOF(uint8_t addr,int ch);
#endif /* DRIVERS_TOFS_H_ */
