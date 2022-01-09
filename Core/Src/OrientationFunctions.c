#include "OrientationFunctions.h"


/*
  * @brief  Calculates angle from based on accelerometer data.
  *
  * @param  accelReading: 	Structure containing accelerometer data.
  * @retval structure with angle values calculated using accelerometer data.
  */
Vector3AxisF Calc_Accel_Angle(struct Data accelReading)
{
	 Vector3AxisF Angle;

		 Angle.x = -atan2f(accelReading.accelY, sqrtf((accelReading.accelX * accelReading.accelX) + (accelReading.accelZ * accelReading.accelZ)));	//phi around x (roll)
		 Angle.y = -atan2f(accelReading.accelX, sqrtf((accelReading.accelY * accelReading.accelY) + (accelReading.accelZ * accelReading.accelZ)));	//theta around y (pitch)

		 return Angle;
}
/***************************************************************************************************************************************************************/
/*
  * @brief  Calculates final angle from accel & gyro with complementary filter.
  *
  * @param  *angle: 	 		 Pointer holding current angle values [deg/s].
  * @param  currGytoReading: 	 Current IMU reading containing rotation data [deg/s].
  * @param  currCalculatedAngle: Angle calculated using only accelerometer data [rad].
  * @param  interval: 			 Time between data aquisitions [s].
  * @example values: 0.00125 800Hz //0.0025 400Hz //0.01 100Hz
  * @retval structure with angle values [deg]
  */
void Calc_Angle(Vector3AxisF *angle, struct Data currGyroReading, Vector3AxisF currCalculatedAngle, float heading, uint8_t interval)
{
	    angle->x = (0.97 * (angle->x + currGyroReading.rotX * interval ) + 0.03 * currCalculatedAngle.x * RAD_TO_DEG_CONST);
	    angle->y = (0.97 * (angle->y + currGyroReading.rotY * interval ) + 0.03 * currCalculatedAngle.y * RAD_TO_DEG_CONST);
	    angle->z = (0.97 * (angle->z + currGyroReading.rotZ * interval ) + 0.03 * heading * RAD_TO_DEG_CONST);
}
/***************************************************************************************************************************************************************/
/*
  * @brief  Calculates heading based on magnetometer data.
  *
  * @param  magReading: 	Magnetometer readings [gauss].
  * @param  angle:  	 	Angle needed to calculate projected values in [deg].
  * @param  declination:  	Magnetic declination specific to application region in [rad].
  * @retval heading: 	 	Calculated heading in [deg].
  */
float Calculate_Heading(Vector3AxisF magReading, Vector3AxisF angle, float declination)
{
	float heading = 0,
		  xH, yH, sinPitch, cosPitch, sinRoll, cosRoll;

	sinPitch = sinf((angle.y*DEG_TO_RAD_CONST));
	cosPitch = cosf((angle.y*DEG_TO_RAD_CONST));
	sinRoll =  sinf((angle.x*DEG_TO_RAD_CONST));
	cosRoll =  cosf((angle.x*DEG_TO_RAD_CONST));

	xH = (magReading.x * cosPitch + magReading.y * sinPitch * sinRoll + magReading.z * sinPitch * cosRoll);
	yH = (magReading.y * cosRoll - magReading.z * sinRoll);

	heading = atan2f(yH, xH);	// Calculating heading in [rad]
	heading += declination;

		if (heading < 0 )
		{
			heading += 2*M_PI;
		}
		else
		if(heading > 2 * M_PI)
		{
			heading -= 2*M_PI;
		}

		heading *= RAD_TO_DEG_CONST;	// Converting value to [deg]

  return heading;
}
/***************************************************************************************************************************************************************/
/*
  * @brief  Calculates altitude based on pressure.
  *
  * @param  seaLevelhPa: Pressure on see level in [hPa].
  * @param  pressure:  	 Pressure on current altitude in [hPa].
  * @retval altitude: 	 Calculated altitude in [m].
  */
float Calculate_Altitude(float seaLevelhPa, uint32_t pressure)
{
  float altitude;

  	  pressure /= 100;// in Si units for Pascal
  	  altitude = 44330 * (1.0 - powf(pressure / seaLevelhPa, 0.1903));

  return altitude;
}
