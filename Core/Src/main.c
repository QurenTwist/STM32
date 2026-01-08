/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "aht20.h"
#include "tftlcd.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef ESP8266_SendCmd(char *cmd, char *resp, uint32_t timeout_ms);
void ESP8266_Reconnect_VOFA(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  TFTLCD_Init();
  FRONT_COLOR = RED;   // 前景色为红色
  BACK_COLOR = WHITE;  // 背景色为白色


  //HAL_GPIO_WritePin(CH_PD_GPIO_Port, CH_PD_Pin, GPIO_PIN_SET); // 拉高使能
  // HAL_Delay(300); // 等待 ESP8266 启动
  // ESP8266_Reconnect_VOFA();// 连接到 VOFA+ 服务器


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */





HAL_StatusTypeDef ESP8266_SendCmd(char *cmd, char *resp, uint32_t timeout_ms) {
  // 清空接收缓冲区
  uint8_t dummy;
  while (HAL_UART_Receive(&huart2, &dummy, 1, 1) == HAL_OK);

  // 发送命令
  HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 1000);
  if (HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY)==HAL_OK)
    {
    LCD_ShowString(10, 400, 200, 30, 16, (u8*)"Sendsuccess");
    }

  HAL_Delay(200); // 增加延时

  // 接收响应
  uint8_t rx_buf[256] = {0};//sizeof(rx_buf)-1
  if (HAL_UART_Receive(&huart2, rx_buf,2 , HAL_MAX_DELAY) == HAL_OK) {
    // 显示实际收到的内容（用于调试）
    LCD_Fill(10, 270, 300, 290, WHITE);  // 清除区域
    LCD_ShowString(10, 270, 200, 20, 12, rx_buf);

    if (resp && strstr((char*)rx_buf, resp)) {
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}


void ESP8266_Reconnect_VOFA(void) {
  LCD_ShowString(10, 30, 300, 20, 16, (u8*)"ESP INIT...");

  // 1. 检查串口连接
  LCD_ShowString(10, 50, 300, 20, 16, (u8*)"TEST UART...");
  if (ESP8266_SendCmd("AT\r\n", "OK", 1000) == HAL_OK) {
    LCD_ShowString(10, 70, 300, 20, 16, (u8*)"UART OK!");
    goto continue_config;  // 如果串口正常，直接配置
  }

  LCD_ShowString(10, 70, 300, 20, 16, (u8*)"UART FAIL");

  // 2. 硬件重启 ESP8266
  LCD_ShowString(10, 90, 300, 20, 16, (u8*)"RESETTING...");

  // 拉低 CH_PD 重启
  HAL_GPIO_WritePin(CH_PD_GPIO_Port, CH_PD_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);  // 保持低电平
  HAL_GPIO_WritePin(CH_PD_GPIO_Port, CH_PD_Pin, GPIO_PIN_SET);
  HAL_Delay(3000);  // 等待启动（增加等待时间）

  // 3. 再次测试 AT 命令
  if (ESP8266_SendCmd("AT\r\n", "OK", 2000) == HAL_OK) {
    LCD_ShowString(10, 110, 300, 20, 16, (u8*)"RESET OK!");
  } else {
    LCD_ShowString(10, 110, 300, 20, 16, (u8*)"HARDWARE ERROR!");
    LCD_ShowString(10, 130, 300, 20, 16, (u8*)"CHECK WIRING!");
    return;
  }

continue_config:
  // 4. 设置为 STA 模式
  LCD_ShowString(10, 130, 300, 20, 16, (u8*)"SET STA MODE...");
  if (ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK", 1000) != HAL_OK) {
    LCD_ShowString(10, 150, 300, 20, 16, (u8*)"CWMODE FAIL");
    return;
  }

  // 5. 连接 WiFi
  LCD_ShowString(10, 150, 300, 20, 16, (u8*)"CONNECT WIFI...");
  if (ESP8266_SendCmd("AT+CWJAP=\"uva\",\"12345678\"\r\n", "WIFI GOT IP", 10000) != HAL_OK) {
    LCD_ShowString(10, 170, 300, 20, 16, (u8*)"WIFI FAIL");
    return;
  }

  // 6. TCP 连接
  LCD_ShowString(10, 170, 300, 20, 16, (u8*)"TCP CONNECT...");
  if (ESP8266_SendCmd("AT+CIPSTART=\"TCP\",\"192.168.94.154\",8888\r\n", "CONNECT", 5000) != HAL_OK) {
    LCD_ShowString(10, 190, 300, 20, 16, (u8*)"TCP FAIL");
    return;
  }

  // 7. 进入透传模式
  LCD_ShowString(10, 190, 300, 20, 16, (u8*)"ENTER TRANSMIT...");
  if (ESP8266_SendCmd("AT+CIPMODE=1\r\n", "OK", 1000) != HAL_OK) {
    LCD_ShowString(10, 210, 300, 20, 16, (u8*)"CIPMODE FAIL");
    return;
  }

  if (ESP8266_SendCmd("AT+CIPSEND\r\n", ">", 1000) != HAL_OK) {
    LCD_ShowString(10, 230, 300, 20, 16, (u8*)"CIPSEND FAIL");
    return;
  }

  LCD_ShowString(10, 250, 300, 20, 16, (u8*)"ESP OK!");
}





/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
