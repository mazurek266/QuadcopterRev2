#include "BMP280.h"

uint16_t BMP280_read16_LE( uint8_t *pData, uint8_t LSB, uint8_t MSB)
{
	uint16_t temp = 0;

		temp = ((pData[MSB] << 8) | pData[LSB]);

	return temp;

}
/********************************************************************************************************************************************************************************************/
void BMP280_Init(I2C_HandleTypeDef *hi2c, bmp280_calib_data *_bmp280_calib)
{

	uint8_t BMPData[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


	 HAL_I2C_Mem_Read(hi2c, BMP280_ADDRESS, BMP280_REGISTER_DIG_T1, 1, BMPData, 24, 100) ;

	 _bmp280_calib->dig_T1 = BMP280_read16_LE(BMPData, 0, 1);
	 _bmp280_calib->dig_T2 = (int16_t)BMP280_read16_LE(BMPData, 2, 3);
	 _bmp280_calib->dig_T3 = (int16_t)BMP280_read16_LE(BMPData, 4, 5);


	 _bmp280_calib->dig_P1 = BMP280_read16_LE(BMPData, 6, 7);
	 _bmp280_calib->dig_P2 = (int16_t)BMP280_read16_LE(BMPData, 8, 9);
	 _bmp280_calib->dig_P3 = (int16_t)BMP280_read16_LE(BMPData, 10, 11);
	 _bmp280_calib->dig_P4 = (int16_t)BMP280_read16_LE(BMPData, 12, 13);
	 _bmp280_calib->dig_P5 = (int16_t)BMP280_read16_LE(BMPData, 14, 15);
	 _bmp280_calib->dig_P6 = (int16_t)BMP280_read16_LE(BMPData, 16, 17);
	 _bmp280_calib->dig_P7 = (int16_t)BMP280_read16_LE(BMPData, 18, 19);
	 _bmp280_calib->dig_P8 = (int16_t)BMP280_read16_LE(BMPData, 20, 21);
	 _bmp280_calib->dig_P9 = (int16_t)BMP280_read16_LE(BMPData, 22, 23);

	HAL_I2C_Mem_Write(hi2c, BMP280_ADDRESS, BMP280_REGISTER_CONFIG, 1, (uint8_t *)((T_STANDBY_0_5MS	<< 5) | (FILTER_COEFFICIENT_X16 << 2)) , 1, 100);
	HAL_Delay(10);
	HAL_I2C_Mem_Write(hi2c, BMP280_ADDRESS, BMP280_REGISTER_CONTROL, 1, (uint8_t *)((OSRS_T_OVERSAMPLING_X1 << 5) | (OSRS_P_OVERSAMPLING_X4 << 2) | NORMAL_MODE), 1, 100);

}
/********************************************************************************************************************************************************************************************/
float BMP280_readTemperature(I2C_HandleTypeDef *hi2c, bmp280_calib_data *_bmp280_calib)
{

  int32_t var1, var2;
  float T;
  uint32_t value;
  uint8_t BMPData[3] = {0,0,0};

  HAL_I2C_Mem_Read(hi2c, BMP280_ADDRESS, BMP280_REGISTER_TEMPDATA, 1, BMPData, 3, 100);

   value = ((uint32_t)BMPData[0] << 12 ) | ((uint32_t)BMPData[1] << 4 ) | ((uint32_t)BMPData[2] ) ;

   int32_t adc_T = value;

  var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib->dig_T1 <<1))) *
	   ((int32_t)_bmp280_calib->dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib->dig_T1)) *
	     ((adc_T>>4) - ((int32_t)_bmp280_calib->dig_T1))) >> 12) *
	   ((int32_t)_bmp280_calib->dig_T3)) >> 14;

  _bmp280_calib->t_fine = var1 + var2;

   T  = (_bmp280_calib->t_fine * 5 + 128) >> 8;

  return (T/100);

}
/********************************************************************************************************************************************************************************************/
float BMP280_readPressure(I2C_HandleTypeDef *hi2c, bmp280_calib_data *_bmp280_calib)
{
  int64_t var1, var2, p;
  uint32_t value;
  float t;
  uint8_t BMPData[3] = {0,0,0};

  // Must be done first to get the t_fine variable set up
  t = BMP280_readTemperature( hi2c, _bmp280_calib);


  HAL_I2C_Mem_Read(hi2c, BMP280_ADDRESS, BMP280_REGISTER_PRESSUREDATA, 1, BMPData, 3, 100);

  value = ((uint32_t)BMPData[0] << 12 ) | ((uint32_t)BMPData[1] << 4 ) | ((uint32_t)BMPData[2] ) ;

     int32_t adc_P = value;

  var1 = (int64_t)_bmp280_calib->t_fine - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib->dig_P6;
  var2 = var2 + ((var1*(int64_t)_bmp280_calib->dig_P5)<<17);
  var2 = var2 + (((int64_t)_bmp280_calib->dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib->dig_P3)>>8) +
    ((var1 * (int64_t)_bmp280_calib->dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib->dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bmp280_calib->dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)_bmp280_calib->dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib->dig_P7)<<4);

  return (float)(p/25600);

}
