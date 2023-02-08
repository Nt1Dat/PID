#include "motor.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "tim.h"
#include "pid.h"



//0 : CLW
void MotorSetDir(int8_t nDir)
{
    switch(nDir)
    {
        case 0:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}

//duty cycle of motor
void MotorSetDuty(uint16_t nDuty)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, nDuty);
}

// sample 10ms
void ReadEncoder(Motor_t * tmotor){
			tmotor->counter=__HAL_TIM_GET_COUNTER(&htim4);
//fix when overflow
//if((TIM4->CR1 & ) == )


			if(tmotor->counter < 1000)
			{
				tmotor->counter = tmotor->counter;
			}
			tmotor->rounds=tmotor->counter/330/4;
			uint16_t temp_data = (tmotor->counter - tmotor->o_counter);
			tmotor->velocity = temp_data/330.0/4.0*100.0*60.0; // rpm

			tmotor->position+=tmotor->velocity/100/60*360; //deg
			if(tmotor->position>=360){//
						tmotor->position=0;
			}

			tmotor->o_counter=tmotor->counter;
}

//Turning
void MotorTuningVelocity(PID_CONTROL_t * PIDControl,Motor_t * tmotor,float vel)
{

    float SetPoint = vel;
    float Input=tmotor->velocity;
    float g_nDutyCycle = PIDCompute(PIDControl, SetPoint,Input, 0.01f);



    if(g_nDutyCycle >= 0.)
    {
        MotorSetDir(0);
        MotorSetDuty(abs((int)g_nDutyCycle));
    }
    else if(g_nDutyCycle < 0.)
    {
        MotorSetDir(1);
        MotorSetDuty(99-abs((int)g_nDutyCycle));
    }

}
void MotorTuningPosition(PID_CONTROL_t * PIDControl,Motor_t * tmotor, float pos)
{

    float SetPoint = pos;
    float Input=tmotor->position;
    float g_nDutyCycle = PIDCompute(PIDControl, SetPoint,Input, 0.01f);



    if(g_nDutyCycle >= 0.)
    {
        MotorSetDir(0);
        MotorSetDuty(abs((int)g_nDutyCycle));
    }
    else if(g_nDutyCycle < 0.)
    {
        MotorSetDir(1);
        MotorSetDuty(99-abs((int)g_nDutyCycle));
    }

}

