/*
 * lidar.c
 *
 *  Created on: Jan 16, 2026
 *      Author: danil
 */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h" //To use the uint8_t typedef
#include "cmsis_os.h"
#include <math.h> //To use atan (Tan-1) in the angle analysis
#include <string.h>


/* Private typedef -----------------------------------------------------------*/

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
	DATA_PROCESSING,
	FIND_TARGET,
	DEBUG_D
}state_e;


typedef struct data_sm
{
	uint8_t current_index;
	uint8_t current_buffer;
	uint8_t first_index;
	uint8_t first_buffer;
} data_state_machine;


typedef struct data_lidar
{
	uint8_t packet_header;
	uint8_t package_type;
	uint8_t sample_quantity;
	uint16_t start_angle;
	uint16_t end_angle;
	uint16_t check_code;
	uint16_t code_calculated;
}s_data_lidar;


//Structure for the final output data: distance and angles, from ~0° to ~360°
typedef struct output_data_lidar
{
	uint16_t distance;
	float angle;
}output_data_lidar_t;


/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE 100

#define HALF_BUFFER_SIZE BUFFER_SIZE/2
#define LIMIT_BUFFER HALF_BUFFER_SIZE-1

#define INDEX_LIMIT_HEADER HALF_BUFFER_SIZE - 3 //To test if a buffer change is necessary to store sample_quantity
#define LIMIT_INDEX_SAVE_DISTANCE LIMIT_BUFFER-1

#define MAX_SAMPLE_QUANTITY 40

#define STACK_DEPTH_READ_LIDAR_TASK 512
#define PRIORITY_READ_LIDAR_TASK 2
#define STACK_DEPTH_PROCESS_LIDAR_TASK 256
#define PRIORITY_PROCESS_LIDAR_TASK 1






/* Private variables ---------------------------------------------------------*/


uint8_t buffer_lidar[BUFFER_SIZE]; //Buffer to receive the data from the LIDAR sensor
uint8_t buffer_half_DMA[HALF_BUFFER_SIZE]; //Buffer to process the data. Data of first half
uint8_t buffer_complet_DMA[HALF_BUFFER_SIZE]; //Buffer to process the data. Data of second half


uint8_t lsb_byte; //Variable to store the low part of read data
uint8_t msb_byte; //Variable to store the high part of read data

uint8_t bytes_required; //Variable to determine if there it's possible to save bytes in the current buffer
uint8_t bytes_availables; //Variable to determine if there it's possible to save bytes in the current buffer
uint8_t bytes_presents; //Variable to determine how many bytes can I store in the current buffer.

uint16_t * pDistanceData; //Pointer for the distance data.
float * pAngles; //Pointer for the angles data.

uint8_t flag_byte_alone = 0;
uint8_t quantity_saved_data = 0;

uint16_t distance_data[40] = {0}; //Buffer/table to save all the distance data from the buffer_half_DMA and buffer_complet_DMA
float angles_data[40] = {0}; //Buffer/table to save all the angles


uint16_t distance_buffer[40]; //Buffer with fixed size of 40 to visualize the distance data. Here will be a copy of the data
float angle_buffer[40]; //Buffer with fixed size of 40 to visualize the distance data. Here will be a copy of the data

uint16_t code_calculated = 0;

data_state_machine data_sm;
s_data_lidar data_lidar;
state_e state = INIT;

output_data_lidar_t output_data_lidar[360];

uint16_t target_distance;
float target_angle;

uint8_t index_target = 0;
static int index_angle;



//variables for debug. DELETE
uint8_t debug_buffer[HALF_BUFFER_SIZE];
uint8_t debug_buffer2[HALF_BUFFER_SIZE];
uint8_t debug_pheader;
uint8_t debug_squantity;
int error_calloc = 0;
int loops = 0;


SemaphoreHandle_t sem_lidar;













/* Functions ---------------------------------------------------------*/


data_state_machine find_beginning(uint8_t first_index, uint8_t first_buffer)
{
	int j;
	while(1)
	{
		if( first_buffer == 1 )
		{
			for(j = first_index; j < HALF_BUFFER_SIZE; j++)
			{
				if(buffer_half_DMA[j] == 0xaa)
				{
					//TODO:It's not possible to identify if the header is in the last position of the buffer
					//The header will be between the 0 and BUFFER_SIZE-2 positions in the buffer
					if(buffer_half_DMA[j+1] == 0x55)
					{
						data_lidar.packet_header = j;
						data_sm.current_index = j;
						data_sm.current_buffer = 1;
						data_lidar.package_type = buffer_half_DMA[j+2]; //Save the Package Type byte: 2 bytes after header.
						return data_sm;
					}
				}
			}

			xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received
			first_buffer = 2;
			first_index = 0;

		}

		if( first_buffer == 2 )
		{
			for(j = first_index; j < HALF_BUFFER_SIZE; j++)
			{
				if(buffer_complet_DMA[j] == 0xaa)
				{
					if(buffer_complet_DMA[j+1] == 0x55)
					{
						data_lidar.packet_header = j;
						data_sm.current_index = j;
						data_sm.current_buffer = 2;
						data_lidar.package_type = buffer_complet_DMA[j+2]; //Save the Package Type byte: 2 bytes after header.
						return data_sm;
					}
				}
			}

			xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received
			first_buffer = 1;
			first_index = 0;

		}
	}

}



uint16_t get_2_bytes(uint16_t data_byte)
{
	bytes_required = 2;
	bytes_availables = LIMIT_BUFFER - data_sm.current_index;

	if(bytes_availables >= bytes_required)
	{
		switch(data_sm.current_buffer)
		{
		case 1:
			lsb_byte = buffer_half_DMA[data_sm.current_index+1];
			msb_byte = buffer_half_DMA[data_sm.current_index+2];
			data_byte = (uint16_t)msb_byte << 8 | lsb_byte;
			data_sm.current_index += 2;
			return data_byte;
			break;

		case 2:
			lsb_byte = buffer_complet_DMA[data_sm.current_index+1];
			msb_byte = buffer_complet_DMA[data_sm.current_index+2];
			data_byte = (uint16_t)msb_byte << 8 | lsb_byte;
			data_sm.current_index += 2;
			return data_byte;
			break;

		default:
			break;
		}


	}
	else //There are not 2 bytes in the current buffer
	{
		if(bytes_availables == 1) //There is 1 byte available
		{
			switch(data_sm.current_buffer)
			{
			case 1:
				lsb_byte = buffer_half_DMA[data_sm.current_index+1];

				xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received

				data_sm.current_index = 0;
				data_sm.current_buffer = 2;

				msb_byte = buffer_complet_DMA[data_sm.current_index];
				data_byte = (uint16_t)msb_byte << 8 | lsb_byte;
				return data_byte;

				break;

			case 2:
				lsb_byte = buffer_complet_DMA[data_sm.current_index+1];

				xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received

				data_sm.current_index = 0;
				data_sm.current_buffer = 1;

				msb_byte = buffer_half_DMA[data_sm.current_index];
				data_byte = (uint16_t)msb_byte << 8 | lsb_byte;
				return data_byte;

				break;

			default:
				break;
			}


		}
		else
		{
			//There is 0 bytes available
			switch(data_sm.current_buffer)
			{
			case 1:

				xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received

				data_sm.current_index = 0;
				data_sm.current_buffer = 2;

				lsb_byte = buffer_complet_DMA[data_sm.current_index];
				msb_byte = buffer_complet_DMA[data_sm.current_index+1];
				data_byte = (uint16_t)msb_byte << 8 | lsb_byte;
				data_sm.current_index += 1;
				return data_byte;

				break;

			case 2:

				xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received

				data_sm.current_index = 0;
				data_sm.current_buffer = 1;

				lsb_byte = buffer_half_DMA[data_sm.current_index];
				msb_byte = buffer_half_DMA[data_sm.current_index+1];
				data_byte = (uint16_t)msb_byte << 8 | lsb_byte;
				data_sm.current_index += 1;
				return data_byte;

				break;

			default:
				break;
			}

		}
	}
	return data_byte;
}



void get_distance_data()
{

	int i; //Variable for the for cycles
	//pDistanceData = calloc(data_lidar.sample_quantity, sizeof(uint16_t));
	pDistanceData = distance_data;
	if( pDistanceData == NULL)
	{
		error_calloc = 1;
	}

	bytes_required = 2 * data_lidar.sample_quantity;
	bytes_availables = LIMIT_BUFFER - data_sm.current_index;

	if(bytes_required <= bytes_availables) //NO NEED TO CHANGE BUFFER
	{
		//SAVE DISTANCE DATA
		switch(data_sm.current_buffer)
		{
		case 1:

			for(i = 0; i <= data_lidar.sample_quantity-1 ; i++)
			{
				lsb_byte = buffer_half_DMA[data_sm.current_index+1];
				msb_byte = buffer_half_DMA[data_sm.current_index+2];
				pDistanceData[i] = (uint16_t)msb_byte << 8 | lsb_byte;
				//pDistanceData[i] = pDistanceData[i] / 4;
				data_sm.current_index+=2;
				//pDistanceData++;
			}

			if(data_sm.current_index == LIMIT_BUFFER) //Index is the last byte in the current buffer?
			{
				data_sm.current_buffer = 2;
				data_sm.current_index = 0;
			}


			break;

		case 2:

			for(i = 0; i <= data_lidar.sample_quantity-1 ; i++)
			{
				lsb_byte = buffer_complet_DMA[data_sm.current_index+1];
				msb_byte = buffer_complet_DMA[data_sm.current_index+2];
				pDistanceData[i] = (uint16_t)msb_byte << 8 | lsb_byte;
				//pDistanceData[i] = pDistanceData[i] / 4;
				data_sm.current_index+=2;
				//pDistanceData++;
			}

			if(data_sm.current_index == LIMIT_BUFFER) //Index is the last byte in the current buffer?
			{
				data_sm.current_buffer = 1;
				data_sm.current_index = 0;
			}

			break;

		default:
			break;
		}

	}
	else //The complete amount of data is not in the current buffer
	{
		bytes_presents = bytes_availables / 2; //Amount of available distance data in the buffer to save

		if(bytes_availables % 2 == 1) //There is a byte at the end of the buffer and it's the lsb of a data, and the msb it's in the other buffer
		{
			flag_byte_alone = 1;
		}

		switch(data_sm.current_buffer)
		{
		case 1:

			for(i = 0; i <= bytes_presents-1 ; i++)
			{
				lsb_byte = buffer_half_DMA[data_sm.current_index+1];
				msb_byte = buffer_half_DMA[data_sm.current_index+2];
				pDistanceData[i] = (uint16_t)msb_byte << 8 | lsb_byte;
				//pDistanceData[i] = pDistanceData[i] / 4;
				data_sm.current_index+=2;
				//pDistanceData++;
				quantity_saved_data++;
			}


			break;

		case 2:

			for(i = 0; i <= bytes_presents-1 ; i++)
			{
				lsb_byte = buffer_complet_DMA[data_sm.current_index+1];
				msb_byte = buffer_complet_DMA[data_sm.current_index+2];
				pDistanceData[i] = (uint16_t)msb_byte << 8 | lsb_byte;
				//pDistanceData[i] = pDistanceData[i] / 4;
				data_sm.current_index+=2;
				//pDistanceData++;
				quantity_saved_data++;
			}

			break;

		default:
			break;
		}


		if(flag_byte_alone == 1) //There is a byte at the end of the buffer and it's the lsb of a data, and the msb it's in the other buffer
		{
			switch(data_sm.current_buffer)
			{
			case 1:

				lsb_byte = buffer_half_DMA[data_sm.current_index+1]; //Save the last byte of the buffer

				xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received

				data_sm.current_buffer = 2; //Change of buffer
				data_sm.current_index = 0;

				msb_byte = buffer_complet_DMA[data_sm.current_index];

				pDistanceData[quantity_saved_data] = (uint16_t)msb_byte << 8 | lsb_byte;
				//pDistanceData[i] = pDistanceData[i] / 4;

				//pDistanceData++;
				quantity_saved_data++;

				break;

			case 2:

				lsb_byte = buffer_complet_DMA[data_sm.current_index+1]; //Save the last byte of the buffer

				xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received

				data_sm.current_buffer = 1; //Change of buffer
				data_sm.current_index = 0;

				msb_byte = buffer_half_DMA[data_sm.current_index];

				pDistanceData[quantity_saved_data] = (uint16_t)msb_byte << 8 | lsb_byte;
				//pDistanceData[i] = pDistanceData[i] / 4;

				//pDistanceData++;
				quantity_saved_data++;

				break;

			default:
				break;
			}

			flag_byte_alone = 0;
		}


		while( quantity_saved_data != data_lidar.sample_quantity ) //Save the data after the first buffer and until the total amount of data is saved
		{
			if(data_sm.current_buffer == 1)
			{
				while( (data_sm.current_index <= LIMIT_BUFFER) && (quantity_saved_data != data_lidar.sample_quantity) )
				{
					if( data_sm.current_index == LIMIT_BUFFER ) //current_index is in the last byte of the buffer
					{
						xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received
						data_sm.current_buffer = 2;
						data_sm.current_index = 0;

						lsb_byte = buffer_complet_DMA[data_sm.current_index];
						msb_byte = buffer_complet_DMA[data_sm.current_index+1];
						pDistanceData[quantity_saved_data] = (uint16_t)msb_byte << 8 | lsb_byte;
						//pDistanceData[i] = pDistanceData[i] / 4;
						data_sm.current_index=1;
						//pDistanceData++;
						quantity_saved_data++;

						break;
					}

					if( data_sm.current_index == LIMIT_INDEX_SAVE_DISTANCE ) //current_index is in the third to last byte
					{
						lsb_byte = buffer_half_DMA[data_sm.current_index+1]; //Save the last byte of the buffer

						xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received

						data_sm.current_buffer = 2; //Change of buffer
						data_sm.current_index = 0;

						msb_byte = buffer_complet_DMA[data_sm.current_index];

						pDistanceData[quantity_saved_data] = (uint16_t)msb_byte << 8 | lsb_byte;
						//pDistanceData[i] = pDistanceData[i] / 4;

						//pDistanceData++;
						quantity_saved_data++;

						break;
					}

					lsb_byte = buffer_half_DMA[data_sm.current_index+1];
					msb_byte = buffer_half_DMA[data_sm.current_index+2];
					pDistanceData[quantity_saved_data] = (uint16_t)msb_byte << 8 | lsb_byte;
					//pDistanceData[i] = pDistanceData[i] / 4;
					data_sm.current_index+=2;
					//pDistanceData++;
					quantity_saved_data++;
				}

			}


			if(data_sm.current_buffer == 2)
			{
				while( (data_sm.current_index <= LIMIT_BUFFER) && (quantity_saved_data != data_lidar.sample_quantity) )
				{
					if( data_sm.current_index == LIMIT_BUFFER ) //current_index is in the last byte of the buffer
					{
						data_sm.current_buffer = 1;
						data_sm.current_index = 0;

						lsb_byte = buffer_complet_DMA[data_sm.current_index];
						msb_byte = buffer_complet_DMA[data_sm.current_index+1];
						pDistanceData[quantity_saved_data] = (uint16_t)msb_byte << 8 | lsb_byte;
						//pDistanceData[i] = pDistanceData[i] / 4;
						data_sm.current_index=1;
						//pDistanceData++;
						quantity_saved_data++;

						break;
					}

					if( data_sm.current_index == LIMIT_INDEX_SAVE_DISTANCE ) //current_index is in the third to last byte
					{
						lsb_byte = buffer_complet_DMA[data_sm.current_index+1]; //Save the last byte of the buffer

						xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received

						data_sm.current_buffer = 1; //Change of buffer
						data_sm.current_index = 0;

						msb_byte = buffer_half_DMA[data_sm.current_index];

						pDistanceData[quantity_saved_data] = (uint16_t)msb_byte << 8 | lsb_byte;
						//pDistanceData[i] = pDistanceData[i] / 4;

						//pDistanceData++;
						quantity_saved_data++;

						break;
					}

					lsb_byte = buffer_complet_DMA[data_sm.current_index+1];
					msb_byte = buffer_complet_DMA[data_sm.current_index+2];
					pDistanceData[quantity_saved_data] = (uint16_t)msb_byte << 8 | lsb_byte;
					//pDistanceData[i] = pDistanceData[i] / 4;
					data_sm.current_index+=2;
					//pDistanceData++;
					quantity_saved_data++;
				}
			}
		}


	}

	//memcpy(distance_buffer, pDistanceData, 80);
	quantity_saved_data = 0;

}



void check_code_parsing()
{
	error_calloc = 0;

	data_lidar.code_calculated = 0x55AA;

	data_lidar.code_calculated ^= (uint16_t)data_lidar.sample_quantity << 8 | data_lidar.package_type;

	data_lidar.code_calculated ^= data_lidar.start_angle;

	data_lidar.code_calculated ^= data_lidar.end_angle;


	for(int i = 0; i <= data_lidar.sample_quantity-1; i++)
	{
		data_lidar.code_calculated ^= pDistanceData[i];
	}

	if(data_lidar.check_code == data_lidar.code_calculated)
	{
		error_calloc = 20;
		state = DATA_PROCESSING;
	}
	else
	{
		error_calloc = 99;

		state = SEARCH_HEADER;
	}

}





void data_processing() //Process distance and angle data at the same time
{
	int i;
	//static int index_angle;
	int data_state = 0;
	//Angle process : First-level analysis:
	float angle_fsa = (float)( data_lidar.start_angle >> 1) / (64.0f);
	float angle_lsa = (float)( data_lidar.end_angle >> 1) / (64.0f);
	float diff_angle = angle_lsa - angle_fsa;
	float aux = (diff_angle)/(data_lidar.sample_quantity - 1);
	float angle_correct;

	//Check the first angle is 0 or greater. Don't save out of range in the structure (0°-359°)
	if( (int)angle_fsa < 0)
	{
		index_angle = 0;
		output_data_lidar[index_angle].angle = angle_fsa;
	}

	index_angle = (int)angle_fsa;
	output_data_lidar[index_angle].angle = angle_fsa;



	//Check the last angle is 359 or lower. Don't save out of range in the structure (0°-359°)
	if( (int)angle_lsa > 359)
	{
		output_data_lidar[359].angle = angle_lsa;
	}

	output_data_lidar[index_angle + data_lidar.sample_quantity - 1].angle = angle_lsa;



	angle_buffer[0] = angle_fsa;
	angle_buffer[data_lidar.sample_quantity-1] = angle_lsa;




	for(i = index_angle+1; i <= (index_angle + data_lidar.sample_quantity - 2); i ++)
	{
		output_data_lidar[i].angle = ( aux * (i-index_angle - 1) ) + angle_fsa;
	}




	for(i = 1; i <= (data_lidar.sample_quantity - 2); i ++)
	{
		angle_buffer[i] = ( aux * (i - 1) ) + angle_fsa;
	}




	//Angle process : Second-level analysis and distance calculation
	for(i = index_angle; i <= (index_angle + MAX_SAMPLE_QUANTITY - 1); i ++)
	{

		output_data_lidar[i].distance = pDistanceData[i-index_angle] / 4; //Distance calculation

		if( output_data_lidar[i].distance == 0)
		{
			angle_correct = 0;
		}
		else
		{
			angle_correct = atan( 21.8 * ( (155.3 - output_data_lidar[i].distance)/(155.3 * output_data_lidar[i].distance) ) );
		}

		output_data_lidar[i].angle = output_data_lidar[i].angle + angle_correct;

		/*if( (i != index_angle) && (i != index_angle + data_lidar.sample_quantity - 1) )
		{
			output_data_lidar[i].angle = output_data_lidar[i].angle + angle_correct;
		}*/

	}




	//Angle process : Second-level analysis and distance calculation
	for(i = 0; i <= (MAX_SAMPLE_QUANTITY - 1); i ++) //BEFORE MODIFICATION THE FOR CONDITION WAS: i <= (MAX_SAMPLE_QUANTITY - 1), I CHANGED BY
	{												 //i <= (data_lidar.sample_quantity - 1) TO MATCH WITH Angle correction formula OF DATASHEET

		distance_buffer[i] = pDistanceData[i] / 4; //Distance calculation

		if( distance_buffer[i] == 0)
		{
			angle_correct = 0;
		}
		else
		{
			if( distance_buffer[i] != 0)
			{
				angle_correct = atan( 21.8 * ( (155.3 - distance_buffer[i])/(155.3 * distance_buffer[i]) ) );
			}

		}

		angle_buffer[i] = angle_buffer[i] + angle_correct;

		/*if( (i != 0) && (i != data_lidar.sample_quantity-1) )
		{
			angle_buffer[i] = angle_buffer[i] + angle_correct;
		}*/

	}

	//Check if the table/structure (0°-359°) is full. Check if the
	//TODO: It must verify if we are about to write out of the structure angle=360 or the next angle is lower than the previous
	//With that we could know the exactly moment when the table/structure is full, and ignore the next angles and start FIND_TARGET
	/*if( angle_lsa >= 320 ) //Verify if the last angle saved is greater or equal to 320° -> I assume the table is full,
	{
		data_state = 1;
	}*/


	//Define the next step in the state machine
	/*if(data_state == 1)
	{
		state = SEARCH_HEADER;//FIND_TARGET;
		data_state = 0;
	}*/


	//DEBUG DELETE THIS!
	loops++;

	/*if(loops == 2)
	{
		state = DEBUG_D;
	}*/

	state = FIND_TARGET;
}


void find_target()
{
	//target_distance = 0;
	//target_angle = 0;
	index_target = index_angle; //Assume the target is the first data in the sample received

	//Find the first data different of zero as target
	for(int i = index_angle; i < (index_angle + MAX_SAMPLE_QUANTITY); i++)
	{
		if( output_data_lidar[i].distance != 0)
		{
			index_target = i;
			break;
		}
	}

	for(int i = index_angle; i < (index_angle + MAX_SAMPLE_QUANTITY); i++)
	{
		if( (output_data_lidar[index_target].distance > output_data_lidar[i+1].distance) && (output_data_lidar[i+1].distance > 0) )
		{
			if( output_data_lidar[i+1].angle > 0.0)
			{
				index_target = i + 1;
			}
		}
	}

	target_distance = output_data_lidar[index_target].distance;
	target_angle = output_data_lidar[index_target].angle;

	state = DEBUG_D;
}



void lidar_state_machine()
{
	switch(state)
	{
		case INIT:
			//Initial values of variables
			data_sm.current_index = 0;
			data_sm.current_buffer = 1;
			state = SEARCH_HEADER;
			break;

		case SEARCH_HEADER:

			find_beginning(data_sm.current_index, data_sm.current_buffer);
			//DEBUG.DELETE THIS
			switch(data_sm.current_buffer)
			{
			case 1:
				memcpy(debug_buffer, buffer_half_DMA, HALF_BUFFER_SIZE);
				memcpy(debug_buffer2, buffer_complet_DMA, HALF_BUFFER_SIZE);

				break;
			case 2:
				memcpy(debug_buffer, buffer_complet_DMA, HALF_BUFFER_SIZE);
				memcpy(debug_buffer2, buffer_half_DMA, HALF_BUFFER_SIZE);

				break;
			default:
				break;
			}
			debug_pheader = data_lidar.packet_header;
			//END OF DEBUG

			state = GET_SAMPLE_QUANTITY;
			break;


		case GET_SAMPLE_QUANTITY:

			if(data_sm.current_index == INDEX_LIMIT_HEADER) //Index in the third to last byte of the buffer. IT IS NECESSARY TO Change of buffer
			{

				switch(data_sm.current_buffer)
				{
				case 1:

					xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received
					data_sm.current_index = 0;
					data_sm.current_buffer = 2;

					break;

				case 2:

					xSemaphoreTake(sem_lidar, portMAX_DELAY); //Blocking the task until the new buffer is received
					data_sm.current_index = 0;
					data_sm.current_buffer = 1;

					break;

				default:
					break;
				}
			}
			else
			{
				data_sm.current_index += 3; //Move the current index to the position of the sample quantity
			}

			//Save the sample quantity after the movement of the index
			if(data_sm.current_buffer == 1)
			{
				data_lidar.sample_quantity = buffer_half_DMA[data_sm.current_index];
			}
			if(data_sm.current_buffer == 2)
			{
				data_lidar.sample_quantity = buffer_complet_DMA[data_sm.current_index];
			}
			debug_squantity = data_lidar.sample_quantity;

			state = GET_START_ANGLE;
			break;

		case GET_START_ANGLE:

			data_lidar.start_angle = get_2_bytes(data_lidar.start_angle);
			state = GET_END_ANGLE;
			break;

		case GET_END_ANGLE:

			data_lidar.end_angle = get_2_bytes(data_lidar.end_angle);
			state = GET_CHECK_CODE;
			break;

		case GET_CHECK_CODE:

			data_lidar.check_code = get_2_bytes(data_lidar.check_code);
			state = GET_DISTANCE_DATA;
			break;

		case GET_DISTANCE_DATA:

			get_distance_data();

			state = CHECK_CODE_PARSING;
			break;


		case CHECK_CODE_PARSING:

			check_code_parsing();
			break;


		case DATA_PROCESSING:

			data_processing();
			break;


		case FIND_TARGET:

			find_target();
			break;


		case DEBUG_D:
			while(1)
			{

			}
			break;

		default:
			break;
	}
}



/*void distance_analysis()
{
	for(int i = 0; i <= data_lidar.sample_quantity-1; i++)
	{
		pDistanceData[i] = pDistanceData[i] / 4;
	}
	memcpy(distance_buffer, pDistanceData, 80);

	state = ANGLE_ANALYSIS;
}

void angle_analysis()
{
	int i;
	//First-level analysis:
	float angle_fsa = (float)( data_lidar.start_angle >> 1) / (64.0f);
	float angle_lsa = (float)( data_lidar.end_angle >> 1) / (64.0f);
	float diff_angle = angle_lsa - angle_fsa;
	float aux = (diff_angle)/(data_lidar.sample_quantity - 1);
	float angle_correct;

	pAngles = angles_data; //Pointer to the table

	pAngles[0] = angle_fsa;
	pAngles[data_lidar.sample_quantity-1] = angle_lsa;

	for(i = 1; i <= (data_lidar.sample_quantity - 2); i ++)
	{
		pAngles[i] = ( aux * (i - 1) ) + angle_fsa;
	}

	//Second-level analysis:
	for(i = 0; i <= (data_lidar.sample_quantity - 1); i ++)
	{
		if( distance_buffer[i] == 0)
		{
			angle_correct = 0;
		}
		else
		{
			angle_correct = atan( 21.8 * ( (155.3 - distance_buffer[i])/(155.3 * distance_buffer[i]) ) );
		}

		pAngles[i] = pAngles[i] + angle_correct;
	}

	memcpy(angle_buffer, pAngles, 160);

	state = SEARCH_HEADER;

}*/
