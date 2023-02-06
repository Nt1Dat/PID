#include "pid.h"
#include "tim.h"

float Error = 0;

//reset PID params
void PIDReset(PID_CONTROL_t *PID_Ctrl)
{
    PID_Ctrl->dIntergral = 0.0f;
    PID_Ctrl->dErrorTerm = 0.0f;
    Error = 0;
}

//init PID
void PIDInit(PID_CONTROL_t *PID_Ctrl, float dKp, float dKi, float dKd)
{
    PIDReset(PID_Ctrl);
    PID_Ctrl->dKp = dKp;
    PID_Ctrl->dKi = dKi;
    PID_Ctrl->dKd = dKd;

}

//Compute PID Controllers
float PIDCompute(PID_CONTROL_t *PID_Ctrl, float SetPoint, float Input, float dTs)
{
    float dPIDResult;
    Error = SetPoint - Input;
    float dP = 0, dI = 0, dD = 0;

    dP = PID_Ctrl -> dKp *Error;
    PID_Ctrl -> dIntergral += Error;
    dI = PID_Ctrl->dKi * dTs * PID_Ctrl->dIntergral;
    dD = PID_Ctrl->dKd * (Error - PID_Ctrl->dErrorTerm) /dTs;
    dPIDResult = dP + dI + dD;
    PID_Ctrl->dErrorTerm = Error;
   // PID_Ctrl->result=dPIDResult;
    return dPIDResult;
}
