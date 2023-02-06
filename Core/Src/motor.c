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
			tmotor->rounds=tmotor->counter/330/4;
			tmotor->velocity=((float)tmotor->counter-(float)tmotor->o_counter)/330/4*100*60;
			tmotor->o_counter=tmotor->counter;
}

//Turning
void MotorTuning(PID_CONTROL_t * PIDControl,Motor_t * tmotor,float vel)
{

    float SetPoint = vel;
    float Input=tmotor->velocity;
    float g_nDutyCycle = PIDCompute(PIDControl, SetPoint,Input, 0.01f);

//    if(g_nDutyCycle<30){g_nDutyCycle=30;}
//    if(g_nDutyCycle>99){g_nDutyCycle=99.;}


       if(PIDControl->dIntergral>3000){PIDControl->dIntergral=3000;}




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

