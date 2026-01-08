/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
  typedef struct {
    uint32_t raw_temp;
    uint32_t raw_light;
    uint32_t raw_soilm;
    uint32_t temp_voltage;
    uint32_t light_voltage;
    uint32_t soilm_voltage;
    uint32_t Temperature;
    uint32_t SoilMoisture;
    uint32_t Illumination;
  } Sensor;
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */

#define NTC_R0      10000.0f    // 25°C 时的阻值（如 10kΩ）
#define NTC_T0      298.15f     // 25°C 对应开尔文温度
#define NTC_BETA    3950.0f     // B 值（常见 3950、3435 等，查规格书）

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */
  float Calculate_NTC_Temperature(float v_adc);
  float Calculate_Light_Intensity(uint32_t raw_adc);
  float Calculate_Soil_Moisture(uint32_t adc_val);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

