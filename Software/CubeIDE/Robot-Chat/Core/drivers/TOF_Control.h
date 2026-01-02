/*
 * TOF_Control.h
 *
 *  Created on: Dec 3, 2025
 *      Author: david
 */

#ifndef INC_TOF_CONTROL_H_
#define INC_TOF_CONTROL_H_

#include "vl53l0x_api.h"
#include "i2c-mux.h"
#include <stdio.h>
#include <stdlib.h>

#define VL53L0X_DEFAULT_ADDRESS 0x29
extern I2C_HandleTypeDef hi2c3;



void configure_TOF(uint8_t addr);
int TOF_Init();
int data_read_TOF(uint8_t addr,int ch);


#endif /* INC_TOF_CONTROL_H_ */
