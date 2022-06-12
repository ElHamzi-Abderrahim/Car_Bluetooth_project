
/*
 * Created on : 30/05/2022
 *
 * Author : EL HAMZI Abderrahim
 *
 */

#include "main.h"

TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart5;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM2_Init(void);

// FONCTIONS DES MOUVEMENTS

void mvt_forward()
{
		HAL_GPIO_WritePin(GPIOA, INE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, INE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, INE3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, INE4_Pin, GPIO_PIN_RESET);
}
void mvt_backward()
{
		HAL_GPIO_WritePin(GPIOA, INE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, INE2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, INE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, INE4_Pin, GPIO_PIN_SET);
}
void mvt_right()
{
		HAL_GPIO_WritePin(GPIOA, INE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, INE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, INE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, INE4_Pin, GPIO_PIN_SET);
}
void mvt_left()
{
		HAL_GPIO_WritePin(GPIOA, INE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, INE2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, INE3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, INE4_Pin, GPIO_PIN_RESET);
}
void stop_mvt()
{
		HAL_GPIO_WritePin(GPIOA, INE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, INE2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, INE3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, INE4_Pin, GPIO_PIN_RESET);
	
}
// VARIABLES
uint8_t Data_rx[2] ;
uint8_t mvt;
uint8_t mode_vitess;


int main(void)
{
  
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
 
	HAL_UART_Receive_IT(&huart5, Data_rx, 2); // commancement de la reception des donnees
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// demmarage des pwms modulation a largueur d'impultion
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  while (1)
  {
    mvt 					= Data_rx[0];
		mode_vitess 	= Data_rx[1];
//		if ( mode_vitess == 'H' )	// rotation a grande vitesse
//			{ 
//			htim2.Instance->CCR1 = 89;
//			htim2.Instance->CCR2 = 89; 
//			}
//		else						// rotation a moyenne vitesse
//			{ 
//			htim2.Instance->CCR1 = 50;
//			htim2.Instanc
		e->CCR2 = 50; 
//			}

		switch (mvt) 
		{
		case 'R' :
					mvt_right();
					break;
		case 'L' :
					mvt_left();
					break;
		case 'B' :
					mvt_backward();
					break;
		case 'F' :
					mvt_forward();
					break;
		default :
					stop_mvt();
		}
		
  
	}
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 450-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, INE2_Pin|INE1_Pin|INE3_Pin|INE4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INE2_Pin INE1_Pin INE3_Pin INE4_Pin */
  GPIO_InitStruct.Pin = INE2_Pin|INE1_Pin|INE3_Pin|INE4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart5.Instance)
    {
			HAL_UART_Receive_IT(&huart5, Data_rx, 2);
    }
}
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

