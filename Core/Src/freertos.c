/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "BMP280.h"
#include "HMC5883L.h"
#include "ESP8266.h"
#include "OrientationFunctions.h"
#include "Pid.h"
#include "externalVariables.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osMutexId_t i2cHandle;

const osMutexAttr_t i2cThread_Mutex_attr = {
	 "i2cThreadMutex",
	  osMutexPrioInherit,
	  NULL,
	  0U
	};

volatile struct Data mpu6050Data;
volatile Vector3AxisF hmc5883lData;
volatile float bmp280Pressure = 0;

volatile struct Data mpu6050Data_filtered;
volatile Vector3AxisF hmc5883lData_filtered;
volatile float bmp280Pressure_filtered;

volatile float altitude = 0;
float heading = 0;
Vector3AxisF quadAngles = {0.0f, 0.0f, 0.0f};



/* USER CODE END Variables */
/* Definitions for mainController */
osThreadId_t mainControllerHandle;
const osThreadAttr_t mainController_attributes = {
  .name = "mainController",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for telemetrySender */
osThreadId_t telemetrySenderHandle;
const osThreadAttr_t telemetrySender_attributes = {
  .name = "telemetrySender",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for bmp280Reader */
osThreadId_t bmp280ReaderHandle;
const osThreadAttr_t bmp280Reader_attributes = {
  .name = "bmp280Reader",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mpu6050Reader */
osThreadId_t mpu6050ReaderHandle;
const osThreadAttr_t mpu6050Reader_attributes = {
  .name = "mpu6050Reader",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal	,
};
/* Definitions for hmc5883lReader */
osThreadId_t hmc5883lReaderHandle;
const osThreadAttr_t hmc5883lReader_attributes = {
  .name = "hmc5883lReader",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void mainControllerLoop(void *argument);
void telemetrySenderTask(void *argument);
void bmp280ReaderTask(void *argument);
void mpu6050ReaderTask(void *argument);
void hmc5883lReaderTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */


	i2cHandle = osMutexNew(&i2cThread_Mutex_attr);


  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainController */
  mainControllerHandle = osThreadNew(mainControllerLoop, NULL, &mainController_attributes);

  /* creation of telemetrySender */
  telemetrySenderHandle = osThreadNew(telemetrySenderTask, NULL, &telemetrySender_attributes);

  /* creation of bmp280Reader */
  bmp280ReaderHandle = osThreadNew(bmp280ReaderTask, NULL, &bmp280Reader_attributes);

  /* creation of mpu6050Reader */
  mpu6050ReaderHandle = osThreadNew(mpu6050ReaderTask, NULL, &mpu6050Reader_attributes);

  /* creation of hmc5883lReader */
  hmc5883lReaderHandle = osThreadNew(hmc5883lReaderTask, NULL, &hmc5883lReader_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_mainControllerLoop */
/**
  * @brief  Function implementing the mainControllerL thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_mainControllerLoop */
void mainControllerLoop(void *argument)
{
  /* USER CODE BEGIN mainControllerLoop */
  /* Infinite loop */
  for(;;)
  {



    osDelay(1000);
  }
  /* USER CODE END mainControllerLoop */
}

/* USER CODE BEGIN Header_telemetrySenderTask */
/**
* @brief Function implementing the telemetrySender thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_telemetrySenderTask */
void telemetrySenderTask(void *argument)
{
  /* USER CODE BEGIN telemetrySenderTask */
  /* Infinite loop */
  for(;;)
  {

	  //Esp8266_SendData(huart, message)

    osDelay(1000);
  }
  /* USER CODE END telemetrySenderTask */
}

/* USER CODE BEGIN Header_bmp280ReaderTask */
/**
* @brief Function implementing the bmp280Reader thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_bmp280ReaderTask */
void bmp280ReaderTask(void *argument)
{
  /* USER CODE BEGIN bmp280ReaderTask */
  /* Infinite loop */
  for(;;)
  {
	  if (osMutexAcquire(i2cHandle,0) == osOK)
	  {

		 bmp280Pressure = BMP280_readPressure(&hi2c1, &bmp280CalibData);

		 	 bmp280Pressure_filtered = RCFilter_Update(&barLpf, bmp280Pressure);
		 	 altitude = Calculate_Altitude(1013.25, bmp280Pressure_filtered);


		 HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);	//RED

		 osMutexRelease(i2cHandle);
	  }


    osDelay(20);
  }
  /* USER CODE END bmp280ReaderTask */
}

/* USER CODE BEGIN Header_mpu6050ReaderTask */
/**
* @brief Function implementing the mpu6050Reader thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mpu6050ReaderTask */
void mpu6050ReaderTask(void *argument)
{
  /* USER CODE BEGIN mpu6050ReaderTask */
  /* Infinite loop */
  for(;;)
  {
	  if (osMutexAcquire(i2cHandle,0) == osOK)
	  {

		  mpu6050Data = MPU6050_Read_Data(&hi2c1);

		  	  mpu6050Data_filtered.accelX = RCFilter_Update(&(accLpf[0]), mpu6050Data.accelX);
		  	  mpu6050Data_filtered.accelY = RCFilter_Update(&(accLpf[1]), mpu6050Data.accelY);
		  	  mpu6050Data_filtered.accelZ = RCFilter_Update(&(accLpf[2]), mpu6050Data.accelZ);
		  	  mpu6050Data_filtered.rotX = RCFilter_Update(&(gyrLpf[0]), mpu6050Data.rotX);
		  	  mpu6050Data_filtered.rotY = RCFilter_Update(&(gyrLpf[1]), mpu6050Data.rotY);
		  	  mpu6050Data_filtered.rotZ = RCFilter_Update(&(gyrLpf[2]), mpu6050Data.rotZ);

		  	  Calc_Angle(&quadAngles, mpu6050Data_filtered, Calc_Accel_Angle(mpu6050Data_filtered),heading, 0.01);

		  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13); //ORANGE

		  osMutexRelease(i2cHandle);
	  }

    osDelay(10);
  }
  /* USER CODE END mpu6050ReaderTask */
}

/* USER CODE BEGIN Header_hmc5883lReaderTask */
/**
* @brief Function implementing the hmc5883lReader thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_hmc5883lReaderTask */
void hmc5883lReaderTask(void *argument)
{
  /* USER CODE BEGIN hmc5883lReaderTask */
  /* Infinite loop */
  for(;;)
  {
	  if (osMutexAcquire(i2cHandle,0) == osOK)
	  {

		  hmc5883lData = HMC5883L_Read_Data(&hi2c1);

		  	  hmc5883lData_filtered.x = RCFilter_Update(&(magLpf[0]), hmc5883lData.x);
		  	  hmc5883lData_filtered.y = RCFilter_Update(&(magLpf[1]), hmc5883lData.y);
		  	  hmc5883lData_filtered.z = RCFilter_Update(&(magLpf[2]), hmc5883lData.z);

		  	  heading = Calculate_Heading(hmc5883lData_filtered, quadAngles, MAGNETIC_DECLINATION);

		  	  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);	// BLUE

		  osMutexRelease(i2cHandle);
	  }

	  	  osDelay(10);
  }
  /* USER CODE END hmc5883lReaderTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
