/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"
#include "math.h"

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t Read_ADC_Channel(uint32_t channel, uint32_t _rank) {
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = _rank;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

    uint32_t adcValue = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);
    return adcValue;
}






uint32_t adcValueIN2 = 0;
uint32_t adcValueIN3 = 0;

bool flag = 1;
bool flagStop = 1;

uint32_t previousTime = HAL_GetTick();
uint32_t previousAdc2 = 0;
uint32_t previousAdc3 = 0;


uint8_t TxData[4];

uint32_t zeugma_abs(uint32_t _a, uint32_t _b) {
	if(_a < _b) {
		return _b - _a;
	}
	else if(_a > _b) {
		return _a - _b;
	}
	else {
		return 0;
	}
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  LoRa Wireless(&huart1, 100, 18);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		adcValueIN2 = Read_ADC_Channel(ADC_CHANNEL_2, ADC_REGULAR_RANK_1); // Motor adc
//		adcValueIN3 = Read_ADC_Channel(ADC_CHANNEL_3, ADC_REGULAR_RANK_2); // Servo adc


		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		adcValueIN2 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		HAL_Delay(1);

		HAL_ADC_Start(&hadc2); // X yani servo için
		HAL_ADC_PollForConversion(&hadc2, 100);
		adcValueIN3 = HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Stop(&hadc2);


		if(zeugma_abs(adcValueIN2, previousAdc2) > 80 || zeugma_abs(adcValueIN3, previousAdc3) > 80) flag = 1;

	  /*
	   * TxData[0] = HighByte Motor
	   * TxData[1] = LowByte Motor
	   *
	   * TxData[2] = HighByte Servo
	   * TxData[3] = LowByte Servo
	   */



	  TxData[0] = (adcValueIN3 >> 8) & 0xFF;
	  TxData[1] = adcValueIN3 & 0xFF;

	  TxData[2] = (adcValueIN2 >> 8) & 0xFF;
	  TxData[3] = adcValueIN2 & 0xFF;



	  if( (HAL_GetTick() - previousTime > 150) && flag) { // && (flag || ((adcValueIN2 < 2100 && adcValueIN2 > 1950) &&  ((adcValueIN3 < 2100 && adcValueIN3 > 1950) ) ) ) ) {
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  Wireless.Transmit(TxData, sizeof(TxData));
		  flag = 0;
		  previousAdc2 = adcValueIN2;
		  previousAdc3 = adcValueIN3;
		  previousTime = HAL_GetTick();
	  }

	  if(( adcValueIN2 < 2100 && adcValueIN2 > 1950 && adcValueIN3 < 2100 && adcValueIN3 > 1950 )) {
		  flagStop = 0;
	  }
	  else if(!( adcValueIN2 < 2100 && adcValueIN2 > 1950 && adcValueIN3 < 2100 && adcValueIN3 > 1950 )) {
		  flagStop = 1;
	  }

	  if( flagStop ) flag = 1;
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

/* USER CODE END 4 */

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
