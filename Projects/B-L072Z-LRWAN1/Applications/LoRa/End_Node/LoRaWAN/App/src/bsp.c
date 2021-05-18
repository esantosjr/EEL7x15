/**
  ******************************************************************************
  * @file    bsp.c
  * @author  MCD Application Team
  * @brief   manages the sensors on the application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "hw.h"
#include "timeServer.h"
#include "bsp.h"
#if defined(LRWAN_NS1)
#include "lrwan_ns1_humidity.h"
#include "lrwan_ns1_pressure.h"
#include "lrwan_ns1_temperature.h"
#else  /* not LRWAN_NS1 */
#if defined(SENSOR_ENABLED)
#if defined (X_NUCLEO_IKS01A1)
#include "x_nucleo_iks01a1_humidity.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_temperature.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#else  /* X_NUCLEO_IKS01A2 */
#include "x_nucleo_iks01a2_humidity.h"
#include "x_nucleo_iks01a2_pressure.h"
#include "x_nucleo_iks01a2_temperature.h"
#include "x_nucleo_iks01a2_magneto.h"
#include "x_nucleo_iks01a2_accelero.h"
#include "x_nucleo_iks01a2_gyro.h"
#endif  /* X_NUCLEO_IKS01A1 */
#endif  /* SENSOR_ENABLED */
#endif  /* LRWAN_NS1 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
#if defined(SENSOR_ENABLED) || defined (LRWAN_NS1)
void *HUMIDITY_handle = NULL;
void *TEMPERATURE_handle = NULL;
void *PRESSURE_handle = NULL;
void *MAGNETO_handle = NULL;
void *ACCELERO_handle = NULL;
void *GYRO_handle = NULL;
#endif

void BSP_sensor_Read(sensor_t *sensor_data)
{
  /* USER CODE BEGIN 5 */
  float HUMIDITY_Value = 0;
  float TEMPERATURE_Value = 0;
  float PRESSURE_Value = 0;
  SensorAxes_t MAGNETO_Value = {0};
  SensorAxes_t GYRO_Value = {0};
  SensorAxes_t ACCELERO_Value = {0};
  SensorAxesRaw_t ACCELERO_Value_Raw = {0};

#if defined(SENSOR_ENABLED) || defined (LRWAN_NS1)
  BSP_HUMIDITY_Get_Hum(HUMIDITY_handle, &HUMIDITY_Value);
  BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle, &TEMPERATURE_Value);
  BSP_PRESSURE_Get_Press(PRESSURE_handle, &PRESSURE_Value);
  BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAGNETO_Value);  
  BSP_GYRO_Get_Axes(GYRO_handle, &GYRO_Value);
  BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACCELERO_Value);
  BSP_ACCELERO_Get_AxesRaw(ACCELERO_handle, &ACCELERO_Value_Raw);
#endif
  sensor_data->humidity     = HUMIDITY_Value;
  sensor_data->temperature  = TEMPERATURE_Value;
  sensor_data->pressure     = PRESSURE_Value;
  sensor_data->magneto      = MAGNETO_Value;
  sensor_data->gyro         = GYRO_Value;
  sensor_data->accelero     = ACCELERO_Value;
  sensor_data->accelero_raw = ACCELERO_Value_Raw;
  /* USER CODE END 5 */
}

void  BSP_sensor_Init(void)
{
  /* USER CODE BEGIN 6 */

#if defined(SENSOR_ENABLED) || defined (LRWAN_NS1)
  /* Initialize sensors */
  BSP_HUMIDITY_Init(HTS221_H_0, &HUMIDITY_handle);
  BSP_TEMPERATURE_Init(HTS221_T_0, &TEMPERATURE_handle);

#ifdef X_NUCLEO_IKS01A1
  BSP_PRESSURE_Init(LPS25HB_P_0, &PRESSURE_handle);
  BSP_MAGNETO_Init(LIS3MDL_0, &MAGNETO_handle);
  BSP_ACCELERO_Init(LSM6DS0_X_0, &ACCELERO_handle);
  BSP_GYRO_Init(LSM6DS0_G_0, &GYRO_handle);
#else /* X_NUCLEO_IKS01A2 */
  BSP_PRESSURE_Init(LPS22HB_P_0, &PRESSURE_handle);
  BSP_MAGNETO_Init(LSM303AGR_M_0, &MAGNETO_handle);
  BSP_ACCELERO_Init(LSM303AGR_X_0, &ACCELERO_handle);
  BSP_GYRO_Init(LSM6DSL_G_0, &GYRO_handle);
#endif

  /* Enable sensors */
  BSP_HUMIDITY_Sensor_Enable(HUMIDITY_handle);
  BSP_TEMPERATURE_Sensor_Enable(TEMPERATURE_handle);
  BSP_PRESSURE_Sensor_Enable(PRESSURE_handle);
  BSP_MAGNETO_Sensor_Enable(MAGNETO_handle);
  BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
  BSP_GYRO_Sensor_Enable(GYRO_handle);
#endif
  /* USER CODE END 6 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
