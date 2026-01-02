/*
 * pid.h
 *
 *  Created on: Dec 11, 2025
 *      Author: mbeng
 */

#ifndef DRIVERS_PID_H_
#define DRIVERS_PID_H_

typedef struct {
    float kp;
    float ki;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float out_min, float out_max);
float PID_Update(PID_t *pid, float error, float dt);

#endif /* DRIVERS_PID_H_ */
