#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include "pid.h"

typedef struct
{
	uint16_t counter;
	uint16_t rounds;
	float velocity;
	uint16_t o_counter;
	float position;
	uint16_t o_position;
} Motor_t;

extern void MotorSetDir(int8_t nDir);
extern void MotorSetDuty(uint16_t nDuty);
extern void ReadEncoder();
extern void MotorTuningVelocity(PID_CONTROL_t *PIDControl,Motor_t * tmotor,float vel);
extern void MotorTuningPosition(PID_CONTROL_t * PIDControl,Motor_t * tmotor, float pos);

#endif /* INC_MOTOR_H_ */
