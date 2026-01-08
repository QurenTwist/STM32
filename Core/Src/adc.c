/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#include <math.h>
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA1     ------> ADC1_IN1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA5     ------> ADC2_IN5
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* ADC3 clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**ADC3 GPIO Configuration
    PF8     ------> ADC3_IN6
    */
    GPIO_InitStruct.Pin = LightSensor_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(LightSensor_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA1     ------> ADC1_IN1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();

    /**ADC2 GPIO Configuration
    PA5     ------> ADC2_IN5
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();

    /**ADC3 GPIO Configuration
    PF8     ------> ADC3_IN6
    */
    HAL_GPIO_DeInit(LightSensor_GPIO_Port, LightSensor_Pin);

  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */



/**
 * @brief 根据 ADC 电压值计算 NTC 温度（单位：摄氏度）
 * @param v_adc: ADC 采集到的电压值（单位：V），范围 [0, 3.3]
 * @return 温度（单位：°C）
 */
float Calculate_NTC_Temperature(uint32_t raw_adc)
{
  const float VCC = 3.3f;
  const float R_REF = 100000.0f; // 上拉电阻 100kΩ

  // 转换为电压
  float v_adc = (raw_adc * VCC) / 4095.0f;

  // 防止除零或无效值
  if (v_adc >= VCC) {
    return -40.0f; // 极低温（开路）
  }
  if (v_adc <= 0.0f) {
    return 125.0f; // 极高温（短路）
  }

  // 计算 NTC 阻值
  float r_ntc = R_REF * v_adc / (VCC - v_adc);

  // Steinhart-Hart 方程（简化 B 参数模型）
  float inv_t = 1.0f / NTC_T0 + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0);
  float t_kelvin = 1.0f / inv_t;
  return t_kelvin - 273.15f;
}

/**
 * @brief 根据 ADC 原始值计算光照强度（单位：Lux）
 * @param raw_adc: ADC 原始值 (0～4095)，如 HAL_ADC_GetValue() 返回值
 * @return 光照强度（单位：Lux），若超出范围则返回 -1
 */
float Calculate_Light_Intensity(uint32_t raw_adc)
{
  const float VCC = 3.3f;            // 电源电压
  const float R27 = 47000.0f;        // 上拉电阻 47kΩ
  const float R26 = 1000.0f;         // 下拉电阻 1kΩ
  const float R10 = 10000.0f;        // LDR 在 10Lux 时的典型阻值（查规格书）
  const float GAMMA = 0.7f;          // LDR 特性参数

  if (raw_adc > 4095) return -1.0f;

  float voltage = (raw_adc * VCC) / 4095.0f;
  if (voltage >= VCC) return 100000.0f;
  if (voltage <= 0.0f) return 0.0f;

  // 计算 LDR 的等效阻值（与 R26 并联）
  float R_LDR_eq = (R27 * voltage) / (VCC - voltage);  // 这是 LDR || R26 的等效阻值

  // 由于 R26 很小（1kΩ），LDR 的实际阻值远大于它
  // 所以我们可以近似认为 R_LDR ≈ R_LDR_eq * (R26 / (R26 - R_LDR_eq))，但这复杂
  // 更简单的方法是：假设 R26 不影响，直接用 R_LDR_eq 作为 LDR 阻值
  // 但注意：R26 会限制最小阻值

  // 使用 LDR 模型计算光照
  float lux = R10 * powf(R_LDR_eq / R10, -1.0f / GAMMA);

  if (lux < 0.0f) lux = 0.0f;
  if (lux > 100000.0f) lux = 100000.0f;

  return lux;
}

/**
 * @brief 根据 ADC 原始值计算土壤湿度百分比（0% ～ 100%）
 * @param raw_adc: ADC 原始值 (0 ～ 4095)，注意：值越大表示越湿（根据实测数据）
 * @return 湿度百分比（0.0 ～ 100.0），float 类型
 */
float Calculate_Soil_Moisture(uint32_t raw_adc)
{
  // === 根据最新实测数据校准 ===
  const uint32_t DRY_RAW = 0;     // 传感器在干燥空气中测得的 ADC 值 (0% 湿度)
  const uint32_t WET_RAW = 2271;  // 传感器完全浸入清水中测得的 ADC 值 (100% 湿度)

  // 如果配置错误或传感器超出范围，进行保护
  if (WET_RAW <= DRY_RAW) {
    return 0.0f;
  }

  // 线性映射：raw_adc 越大 → 湿度越高
  // 公式: moisture = (raw - DRY) / (WET - DRY) * 100%
  float moisture = (float)(raw_adc - DRY_RAW) * 100.0f / (float)(WET_RAW - DRY_RAW);

  // 限幅到 [0, 100]
  if (moisture < 0.0f) moisture = 0.0f;
  if (moisture > 100.0f) moisture = 100.0f;

  return moisture;
}



/* USER CODE END 1 */
