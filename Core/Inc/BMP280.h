#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "stdint.h"
#include <stm32f4xx_hal.h>
#include <math.h>

/*=========================================================================
    I2C ADDRESS/BITS/SETTINGS
    -----------------------------------------------------------------------*/
    #define BMP280_ADDRESS                (0x77<<1)
    #define BMP280_CHIPID                 (0x58)

	#define OSRS_T_OVERSAMPLING_X1		0b001
	#define OSRS_T_OVERSAMPLING_X2		0b010
	#define OSRS_T_OVERSAMPLING_X4		0b011
	#define OSRS_T_OVERSAMPLING_X8		0b100
	#define OSRS_T_OVERSAMPLING_X16		0b101

	#define OSRS_P_OVERSAMPLING_X1		0b001
	#define OSRS_P_OVERSAMPLING_X2		0b010
	#define OSRS_P_OVERSAMPLING_X4		0b011
	#define OSRS_P_OVERSAMPLING_X8		0b100
	#define OSRS_P_OVERSAMPLING_X16		0b101

	#define FILTER_COEFFICIENT_X1		0b001
	#define FILTER_COEFFICIENT_X2		0b010
	#define FILTER_COEFFICIENT_X4		0b011
	#define FILTER_COEFFICIENT_X8		0b100
	#define FILTER_COEFFICIENT_X16		0b101

	#define T_STANDBY_0_5MS				0b000
	#define T_STANDBY_62_5MS			0b001
	#define T_STANDBY_125MS				0b010
	#define T_STANDBY_250MS				0b011
	#define T_STANDBY_500MS				0b100
	#define T_STANDBY_1000MS			0b101
	#define T_STANDBY_2000MS			0b110
	#define T_STANDBY_4000MS			0b111

	#define SLEEP_MODE					0b00
	#define FORCED_MODE					0b01
	#define NORMAL_MODE					0b11


/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
      BMP280_REGISTER_DIG_T1              = 0x88,
      BMP280_REGISTER_DIG_T2              = 0x8A,
      BMP280_REGISTER_DIG_T3              = 0x8C,

      BMP280_REGISTER_DIG_P1              = 0x8E,
      BMP280_REGISTER_DIG_P2              = 0x90,
      BMP280_REGISTER_DIG_P3              = 0x92,
      BMP280_REGISTER_DIG_P4              = 0x94,
      BMP280_REGISTER_DIG_P5              = 0x96,
      BMP280_REGISTER_DIG_P6              = 0x98,
      BMP280_REGISTER_DIG_P7              = 0x9A,
      BMP280_REGISTER_DIG_P8              = 0x9C,
      BMP280_REGISTER_DIG_P9              = 0x9E,

      BMP280_REGISTER_CHIPID             = 0xD0,
      BMP280_REGISTER_VERSION            = 0xD1,
      BMP280_REGISTER_SOFTRESET          = 0xE0,

      BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

      BMP280_REGISTER_CONTROL            = 0xF4,
      BMP280_REGISTER_CONFIG             = 0xF5,
      BMP280_REGISTER_PRESSUREDATA       = 0xF7,
      BMP280_REGISTER_TEMPDATA           = 0xFA,
    };

/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct
    {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;

      int32_t t_fine;

    } bmp280_calib_data;
/*=========================================================================*/


void BMP280_Init(I2C_HandleTypeDef *hi2c, bmp280_calib_data *_bmp280_calib);
uint16_t BMP280_read16_LE( uint8_t *pData, uint8_t LSB, uint8_t MSB);
float BMP280_readTemperature(I2C_HandleTypeDef *hi2c, bmp280_calib_data *_bmp280_calib);
float BMP280_readPressure(I2C_HandleTypeDef *hi2c, bmp280_calib_data *_bmp280_calib);

#endif /* INC_BMP280_H_ */
