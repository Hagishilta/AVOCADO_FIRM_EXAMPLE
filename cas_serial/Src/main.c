/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum _CAS_HEADER1{
 CAS_OVERLOAD,
 CAS_UNDERLOAD,
 CAS_STABLE,
 CAS_UNSTABLE
}CAS_HEADER1;

typedef enum _CAS_HEADER2{
 CAS_NET_WEIGHT,
 CAS_GROSS_WEIGHT
}CAS_HEADER2;

typedef enum _CAS_UNIT{
  CAS_UNIT_Kg,
  CAS_UINT_t,
  CAS_UINT_g
}CAS_UNIT;

typedef struct _CAS_DATA{
  CAS_HEADER1 header1;
  CAS_HEADER2 header2;
  float data;
  CAS_UNIT unit;
}CAS_DATA;


// My Global Variables
#define CAS_RX_BUFFSIZE         100
uint8_t m_casRxBuffer[CAS_RX_BUFFSIZE];

CAS_DATA m_casData;

#define ci(i) (((i) + CAS_RX_BUFFSIZE) % CAS_RX_BUFFSIZE)
bool parse_casData(){
  
  static uint8_t temp[7];
  static CAS_DATA casData;
  
  uint8_t i, j;
  bool state = false;
  
  int32_t weight_hi, weight_lo;
  bool is_neg, is_decimal;
  
  char* x = (char*)&temp;
  
  // 1. Find frame between CR(0x0D) LF(0x0A)
  for(i = 0; i < CAS_RX_BUFFSIZE; i++){
    if(m_casRxBuffer[i] == 0x0D){
      if(i == CAS_RX_BUFFSIZE - 1){
        if(m_casRxBuffer[0] == 0x0A){
          state = true;
          break;
        }
      }else{
        if(m_casRxBuffer[i + 1] == 0x0A){
          state = true;
          break;
        }
      }
    }
  }
  
  if(!state) return false;
  
  // 2. Check data frame briefly
  i = ci(i - 16);
  if(m_casRxBuffer[ci(i + 2)] != ',') return false;
  if(m_casRxBuffer[ci(i + 5)] != ',') return false;
  
  // 3. Check Data Header1
  temp[0] = m_casRxBuffer[i];
  temp[1] = m_casRxBuffer[ci(i + 1)];
  
  
  if(!strncmp(x, "OL", 2)) casData.header1 = CAS_OVERLOAD;
  else if(!strncmp(x, "UL", 2)) casData.header1 = CAS_UNDERLOAD;
  else if(!strncmp(x, "ST", 2)) casData.header1 = CAS_STABLE;
  else if(!strncmp(x, "US", 2)) casData.header1 = CAS_UNSTABLE;
  else return false;
  
  // 4. Check Data Header2
  temp[0] = m_casRxBuffer[ci(i + 3)];
  temp[1] = m_casRxBuffer[ci(i + 4)];
  
  if(!strncmp(x, "NT", 2)) casData.header2 = CAS_NET_WEIGHT;
  else if(!strncmp(x, "GS", 2)) casData.header2 = CAS_GROSS_WEIGHT;
  else return false;
  
  // 5. Check Weight Data
  for(j = 0; j < 8; j++) temp[j] = m_casRxBuffer[ci(i + 6 + j)];
  
  is_neg = false;
  is_decimal = false;
  switch(temp[0]){
  case '+' : break;
  case '-' : is_neg = true; break;
  case ' ' : break;
  case '.' : is_decimal = true; break;
  default : return false;
  }
  
  weight_hi = 0;
  weight_lo = 0;
  for(j = 1; j < 8; j++){
    if(temp[j] >= '0' && temp[j] <= '9'){
      if(is_decimal) weight_lo = weight_lo * 10 + (temp[j] - '0');
      else weight_hi = weight_hi * 10 + (temp[j] - '0');
    }else if(temp[j] == '.'){
      is_decimal = true;
    }else{
      return false;
    }
  }
  
  casData.data = (float)weight_lo;
  while(casData.data > 1.0f) casData.data /= 10.0f;
  casData.data += (float)weight_hi;
  if(is_neg) casData.data *= -1.0f;
  
  // 6. Check units
  temp[0] = m_casRxBuffer[ci(i + 14)];
  temp[1] = m_casRxBuffer[ci(i + 15)];
  if(!strncmp(x, "kg", 2)) casData.unit = CAS_UNIT_Kg;
  else if(!strncmp(x, "-t", 2)) casData.unit = CAS_UINT_t;
  else if(!strncmp(x, "-g", 2)) casData.unit = CAS_UINT_g;
  else return false;
  
  // 7. Copy data & erase frame from buffer
  for(j = 0; j < 18; j++) m_casRxBuffer[ci(i + j)] = 0x00;
  memcpy(&m_casData, &casData, sizeof(CAS_DATA));
  
  return true;
  
}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart2, m_casRxBuffer, CAS_RX_BUFFSIZE);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(parse_casData()){
      if(m_casData.header1 == CAS_STABLE) HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
      else HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
    }
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
