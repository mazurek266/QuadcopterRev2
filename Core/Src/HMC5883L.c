#include "HMC5883L.h"


void HMC5883L_Init(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDRESS , HMC5883L_REG_CONFIG_A, 1, (uint8_t *)((HMC5883L_SAMPLES_1<<5) | (HMC5883L_DATARATE_75HZ<<2) | HMC5883L_NORMAL_MODE), 1, 100);

	HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDRESS , HMC5883L_REG_CONFIG_B, 1, (uint8_t *)(HMC5883L_RANGE_1_3GA<<5), 1, 100);

	HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDRESS , HMC5883L_REG_MODE, 1, (uint8_t *)HMC5883L_SINGLE, 1, 100);

}


Vector3AxisF HMC5883L_Read_Data(I2C_HandleTypeDef *hi2c)
{

	Vector3AxisI rawMagnetometerData;
	Vector3AxisF magnetometerData;

	uint8_t data[14];

		HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDRESS, HMC5883L_REG_OUT_X_M, 1, data, 6, 100);

		// Converting magnetometer data to int16_t
		rawMagnetometerData.x = (((data[0] << 8) | data[1]) - MAGNETOMETER_OFFSET_X);
		rawMagnetometerData.z = (((data[2] << 8) | data[3]) - MAGNETOMETER_OFFSET_Z);
		rawMagnetometerData.y = (((data[4] << 8) | data[5]) - MAGNETOMETER_OFFSET_Y);

		magnetometerData.x = rawMagnetometerData.x  * HMC5883L_SCALE_FACTOR;
		magnetometerData.y = rawMagnetometerData.y  * HMC5883L_SCALE_FACTOR;
		magnetometerData.z = rawMagnetometerData.z  * HMC5883L_SCALE_FACTOR;

	return magnetometerData;
}
