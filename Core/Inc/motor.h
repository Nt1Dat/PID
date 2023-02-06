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
} Motor_t;

extern void MotorSetDir(int8_t nDir);
extern void MotorSetDuty(uint16_t nDuty);
extern void ReadEncoder();
extern void MotorTuning(PID_CONTROL_t *PIDControl,Motor_t * tmotor,float vel);

#endif /* INC_MOTOR_H_ */
