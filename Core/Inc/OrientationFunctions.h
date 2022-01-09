
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stm32f4xx_hal.h>
#include "DataStructures.h"

#define MAGNETIC_DECLINATION  ((6.0f + (20.0f / 60.0f)) * (M_PI / 180.0f))
#define DEG_TO_RAD_CONST  0.0175f
#define RAD_TO_DEG_CONST 57.2957f


Vector3AxisF Calc_Accel_Angle(struct Data accelReading);
void Calc_Angle(Vector3AxisF *angle, struct Data currImuReading, Vector3AxisF currCalculatedAngle, float heading, uint8_t interval);
float Calculate_Heading(Vector3AxisF magReading, Vector3AxisF angle, float declination);
float Calculate_Altitude(float seaLevelhPa, uint32_t pressure);
