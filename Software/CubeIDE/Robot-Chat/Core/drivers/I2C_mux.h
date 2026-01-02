/*
 * I2C_mux.h
 *
 *  Created on: Jan 2, 2026
 *      Author: mbeng
 */

#ifndef DRIVERS_I2C_MUX_H_
#define DRIVERS_I2C_MUX_H_

#include "stm32wbxx_hal.h" // Can be replaced by stm32xxxx_hal.h

#define I2C_MUX_BASE_ADDR 112

// All times are defined using milliseconds
#define I2C_MUX_TIMEOUT 10		// Depends on bit rate. At 400kHz, 1ms should be fine
#define I2C_MUX_RESET_TIME_LOW 1	// Minimum 6ns reset pulse according to datasheet
#define I2C_MUX_RESET_TIME_HIGH 0	// Start condition can begin immediately after reset

#define CHANNEL_0 1
#define CHANNEL_1 2
#define CHANNEL_2 4
#define CHANNEL_3 8


typedef struct i2c_mux {
	I2C_HandleTypeDef* hi2c;	// I2C bus controller
	GPIO_TypeDef* rst_port;		// Reset pin GPIO port (set NULL if unused)
	uint16_t rst_pin;		// Reset pin bitmask
	uint8_t addr_offset;		// Offset from base address (set using address pins)  (ici 0)
} i2c_mux_t;

// Returns 0 on success, 1 on error.
int i2c_mux_reset(i2c_mux_t* mux);

// Assigning "ch" as 0-7 will enable the corresponding multiplexer channels.
// Any other value of "ch" will disable all channels.
// Returns 0 on success, 1 on error.
int i2c_mux_select(i2c_mux_t* mux, int ch);


// Multiple multiplexer channels can be enabled or disabled simultaneously.
// The 8 bits of "mask" correspond to the 8 multiplexer channels.
// If a bit is 1 then the corresponding channel is enabled, otherwise it is disabled.
// Returns 0 on success, 1 on error.
int i2c_mux_select_multi(i2c_mux_t* mux, uint8_t mask);

#endif /* DRIVERS_I2C_MUX_H_ */
