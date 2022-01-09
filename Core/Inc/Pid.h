
#ifndef INC_PID_H_
#define INC_PID_H_

#include <math.h>
#include <stm32f4xx_hal.h>

void SetOutput_1_ms(TIM_HandleTypeDef htim);
int Calculate_PID(float Setpoint, float Reading, float Kp, float Ki, float IntegratedError, float dt, int WindupGuard);


#endif /* INC_PID_H_ */
