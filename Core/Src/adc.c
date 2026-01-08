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
float Calculate_NTC_Temperature(float v_adc)
{
  float vcc = 3.3f;             // 电源电压
  float r_ref = 100000.0f;      // 上拉电阻 R72 = 100kΩ
  float r_ntc;                  // 当前 NTC 阻值
  float inv_t;                  // 1/T 的中间变量
  float t_kelvin;               // 绝对温度（K）
  float temperature_celsius;    // 摄氏度

  // 1. 计算当前 NTC 阻值
  if (v_adc >= vcc) {
    r_ntc = 0.0f;
  } else if (v_adc <= 0.0f) {
    r_ntc = 1e9f; // 近似无穷大
  } else {
    r_ntc = r_ref * v_adc / (vcc - v_adc);
  }

  // 2. 使用 Steinhart-Hart 方程计算温度
  inv_t = 1.0f / NTC_T0 + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0);
  t_kelvin = 1.0f / inv_t;
  temperature_celsius = t_kelvin - 273.15f;

  return temperature_celsius;
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
 * @brief 将土壤传感器 ADC 值转换为 0～100% 湿度百分比
 * @param adc_val: 从 ADC 读取的原始值（例如 0～4095）
 * @return int16_t: 湿度百分比 (0～100)
 *
 * 注意：
 * - DRY_ADC   = 传感器在干燥空气中测得的 ADC 值（典型值 3800～4095）
 * - WET_ADC   = 传感器完全浸入清水中测得的 ADC 值（典型值 400～800）
 * - 这两个值必须通过实际校准获得！
 */
float Calculate_Soil_Moisture(uint32_t adc_val)
{
  // === 校准参数（请根据你的传感器实测修改！）===
  const uint32_t DRY_ADC = 4000;   // 干燥时 ADC 值（空气中）
  const uint32_t WET_ADC = 600;    // 完全湿润时 ADC 值（清水中）

  // 防止除零
  if (DRY_ADC <= WET_ADC) {
    return 0; // 配置错误，返回安全值
  }

  // 线性映射：adc_val 越小 → 湿度越高（反向特性）
  int32_t moisture = (int32_t)(DRY_ADC - adc_val) * 100 / (int32_t)(DRY_ADC - WET_ADC);

  // 限制范围 0～100
  if (moisture < 0)   moisture = 0;
  if (moisture > 100) moisture = 100;

  return (uint32_t)moisture;
}
/* USER CODE END 1 */
