#include "Pid.h"


void SetOutput_1_ms(TIM_HandleTypeDef htim)
{
	htim.Instance->CCR1 = 499;//pa0 red
	htim.Instance->CCR2 = 499;//pa1 red
	htim.Instance->CCR3 = 499;//pa2 white
	htim.Instance->CCR4 = 499;//pa3 white
}
/****************************************************************************************************************************************************************/
int Calculate_PID(float Setpoint, float Reading, float Kp, float Ki, float IntegratedError, float dt, int WindupGuard)
{

	int Output = 0;
	float Error = 0;

			Error = (Setpoint - Reading);
			IntegratedError = IntegratedError +  Error * dt;

				if(IntegratedError > WindupGuard)
				{IntegratedError   = WindupGuard;}
				else
				if(IntegratedError < (-WindupGuard))
				{IntegratedError = 	 (-WindupGuard);}

			Output = rint((Error * Kp + IntegratedError * Ki));


	return Output;
}
