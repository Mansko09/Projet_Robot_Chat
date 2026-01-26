/*
 * lidar.h
 *
 *  Created on: Jan 16, 2026
 *      Author: danil
 */

#ifndef DRIVERS_LIDAR_H_
#define DRIVERS_LIDAR_H_

#define BUFFER_SIZE 100
#define HALF_BUFFER_SIZE BUFFER_SIZE/2


#define STACK_DEPTH_READ_LIDAR_TASK 512
#define PRIORITY_READ_LIDAR_TASK 2
#define STACK_DEPTH_PROCESS_LIDAR_TASK 256
#define PRIORITY_PROCESS_LIDAR_TASK 1



//State machine
typedef enum
{
	INIT,
	SEARCH_HEADER,
	GET_SAMPLE_QUANTITY,
	GET_START_ANGLE,
	GET_END_ANGLE,
	GET_CHECK_CODE,
	GET_DISTANCE_DATA,
	CHECK_CODE_PARSING,
	DISTANCE_ANALYSIS,
	ANGLE_ANALYSIS,
	DATA_PROCESSING,
	FIND_TARGET,
	DEBUG_D
}state_e;



extern uint8_t buffer_lidar[BUFFER_SIZE]; //Buffer to receive the data from the LIDAR sensor

extern uint8_t buffer_half_DMA[HALF_BUFFER_SIZE]; //Buffer to process the data. Data of first half
extern uint8_t buffer_complet_DMA[HALF_BUFFER_SIZE]; //Buffer to process the data. Data of second half
extern uint8_t buffer_lidar[BUFFER_SIZE]; //Buffer to receive the data from the LIDAR sensor

extern state_e state;

#endif /* DRIVERS_LIDAR_H_ */
