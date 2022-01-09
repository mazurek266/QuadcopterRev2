#include "MPU6050.h"


void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_Mem_Write(hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, (uint8_t *)MPU6050_CLOCK_PLL_XGYRO, 1, 100);

	HAL_I2C_Mem_Write(hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, (uint8_t *)MPU6050_GYRO_FS_500, 1, 100);

	HAL_I2C_Mem_Write(hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, (uint8_t *)MPU6050_ACCEL_FS_16, 1, 100);
}
/****************************************************************************************************************************************************************/
struct Data MPU6050_Read_Data(I2C_HandleTypeDef *hi2c )
{

		Vector3AxisI rawGyro;
		Vector3AxisI rawAccel;
		struct Data dataVector;
		struct Data dataVectorRaw;
		uint8_t data[14];
		int16_t tempRaw;
		float normAccel;

			   // Reading data from MPU_6050
				HAL_I2C_Mem_Read(hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, data, 14, 100);

/*****AccelerometerData**************************************************/
			   // Converting acceleration data to int16_t
			   rawAccel.x = ((data[0] << 8) | data[1]) - 53;
			   rawAccel.y = ((data[2] << 8) | data[3]) + 80;
			   rawAccel.z = ((data[4] << 8) | data[5]) + 626;

			   // Calculating raw acceleration values
			   dataVectorRaw.accelX = ((float) rawAccel.x * MPU6050_ACC_RESOLUTION_16G) / (float) INT16_MAX;
			   dataVectorRaw.accelY = ((float) rawAccel.y * MPU6050_ACC_RESOLUTION_16G) / (float) INT16_MAX;
			   dataVectorRaw.accelZ = ((float) rawAccel.z * MPU6050_ACC_RESOLUTION_16G) / (float) INT16_MAX;

			   normAccel = sqrt( ((dataVectorRaw.accelX * dataVectorRaw.accelX) +
					   	   	   	  (dataVectorRaw.accelY * dataVectorRaw.accelY) +
								  (dataVectorRaw.accelZ * dataVectorRaw.accelZ)) );

			   // Calculating normalized acceleration values
			   dataVector.accelX = dataVectorRaw.accelX / normAccel;
			   dataVector.accelY = dataVectorRaw.accelY / normAccel;
			   dataVector.accelZ = dataVectorRaw.accelZ / normAccel;

/*****TempData**********************************************************/

			   tempRaw = ((data[6] << 8) | data[7]);

			   // Scaling temperature data to 'C
			   dataVector.temp = ((float) tempRaw / 340 ) + 36.53  ;

/*****GyroData**********************************************************/
			  // Konwersja odebranych bajtow danych na typ int16_t
			   rawGyro.x = ((data[8] << 8) | data[9]) - 1562;
			   rawGyro.y = ((data[10] << 8) | data[11]) + 21;
			   rawGyro.z = ((data[12] << 8) | data[13]) + 413;


			  dataVector.rotX = (((float) rawGyro.x  * MPU6050_GYRO_RESOLUTION_500) / (float) INT16_MAX );// * (M_PI / 180);
			  dataVector.rotY = (((float) rawGyro.y * MPU6050_GYRO_RESOLUTION_500) / (float) INT16_MAX );//* (M_PI / 180);
			  dataVector.rotZ = (((float) rawGyro.z * MPU6050_GYRO_RESOLUTION_500) / (float) INT16_MAX );//* (M_PI / 180);

		  return dataVector;
}
