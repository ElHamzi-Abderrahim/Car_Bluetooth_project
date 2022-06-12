#include "main.h"
#include "stdio.h"

ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);

// fonction pour selectionner le channel de conversion ADC :

ADC_ChannelConfTypeDef sConfig = {0};

void ADC_Select_CH0(void)
{
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
void ADC_Select_CH1(void)
{
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

// declaration des variables :

uint16_t Data_rx ; 						// peut prendre 'F' ou 'B'
uint16_t Data_ry ; 						// peut prendre 'L' ou 'R'
uint8_t Data_transmit[2] ;
uint8_t Data_commande ;
uint8_t commande_mode = 'N'; 	// le mode de commande par defaut  est avec les bottons
uint8_t commande_vitess='Q';	// le mode de vitess rapide= 'H' ou lente = 'Q'

		// fonction de verification du mode de la commande //
void Mode_Commande()
{
	if (HAL_GPIO_ReadPin(GPIOB, botton_commande_Pin) == GPIO_PIN_SET) 
	{
		if (commande_mode == 'N') 
			{ commande_mode = 'J' ; 
				HAL_GPIO_WritePin(GPIOB, led_mode_Pin, GPIO_PIN_SET);
				HAL_Delay(500); // creation d'une retard temporelle pour qu'on soit sur au cas du botton.
			} 
		else 
		{
			commande_mode = 'N' ;
			HAL_GPIO_WritePin(GPIOB, led_mode_Pin, GPIO_PIN_RESET);
			HAL_Delay(500);  // creation d'une retard temporelle pour qu'on soit sur au cas du botton.
		}
	}
	else commande_mode = commande_mode;
}


// fonctions de traitement du variables du joystick //
uint8_t traitement_X_JS(uint16_t Data_rx) 
{
	if ( Data_rx > 2500  ) Data_commande = 'B' ;
	else if ( Data_rx < 1500) Data_commande = 'F';
	return Data_commande;
}

uint8_t traitement_Y_JS(uint16_t Data_ry)
{
	if ( Data_ry > 2500 ) Data_commande = 'L' ;
	else if ( Data_ry < 1500) Data_commande = 'R';
	return Data_commande;
}

uint8_t traitement_etat_stable_JS(uint16_t Data_rx, uint16_t Data_ry) 
{
	if ( Data_rx > 1500 && Data_rx < 2500 && Data_ry > 1500 && Data_ry < 2500) Data_commande = 'S' ;
	else Data_commande = Data_commande ;
	return Data_commande;
}


// fonctions de traitement du variables des bottons //
uint8_t traitement_bottons()
{
	if (HAL_GPIO_ReadPin(GPIOA, RIGHT_Pin) == GPIO_PIN_SET) 				Data_commande 	= 'R';
	else if (HAL_GPIO_ReadPin(GPIOA, LEFT_Pin) == GPIO_PIN_SET)			Data_commande 	= 'L';
	else if (HAL_GPIO_ReadPin(GPIOA, BACKWARD_Pin) == GPIO_PIN_SET) Data_commande 	= 'B';
	else if (HAL_GPIO_ReadPin(GPIOA, FORWARD_Pin) == GPIO_PIN_SET) 	Data_commande 	= 'F';
	else 																														Data_commande 	= 'S';
	return Data_commande;
}	


void Mode_Vitess()
{
	if (HAL_GPIO_ReadPin(GPIOB, botton_vitess_Pin) == GPIO_PIN_SET)
		{
		if (commande_vitess == 'Q') 
			{ commande_vitess='H';
				HAL_GPIO_WritePin(GPIOB, led_vitess_Pin, GPIO_PIN_SET);
				HAL_Delay(250); // creation d'une retard temporelle pour qu'on soit sur au cas du botton.
			} 
		else 
		{
			commande_vitess='Q' ;
			HAL_GPIO_WritePin(GPIOB, led_vitess_Pin, GPIO_PIN_RESET);
			HAL_Delay(250);  // creation d'une retard temporelle pour qu'on soit sur au cas du botton.
		}
	}
	else commande_vitess = commande_vitess ;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
 
	
  while (1)
  {
		ADC_Select_CH0();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		Data_rx = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		
		ADC_Select_CH1();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		Data_ry = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		
		/// tester s'il y a un changement des modes ///
		Mode_Commande();
		Mode_Vitess();
		
		
		/// 	FONCTIONS DE TRAITEMENT DES DONNEES COMMANDEE PAR LE JOYSTICK(JS)		///
		if (commande_mode == 'J')
		{
			Data_commande = traitement_X_JS(Data_rx); // test suivant les axes des X
			Data_commande = traitement_Y_JS(Data_ry); // test suivant les axes des Y
			Data_commande = traitement_etat_stable_JS(Data_rx, Data_ry); // test si le joystick est dans l'etat stable
		}
		/// 	FONCTIONS DE TRAITEMENT DES DONNEES COMMANDEE PAR LES BOTTONS	///
		else 
		{
			Data_commande = traitement_bottons();
		}
		
		
		Data_transmit[0] = Data_commande;
		Data_transmit[1] = commande_vitess;

		/// Transmission des donnees ///
		HAL_UART_Transmit(&huart1, Data_transmit ,2 , 500);	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led_mode_Pin|led_vitess_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RIGHT_Pin LEFT_Pin FORWARD_Pin BACKWARD_Pin */
  GPIO_InitStruct.Pin = RIGHT_Pin|LEFT_Pin|FORWARD_Pin|BACKWARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led_mode_Pin led_commande_Pin */
  GPIO_InitStruct.Pin = led_mode_Pin|led_vitess_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : botton_vitess_Pin botton_commande_Pin */
  GPIO_InitStruct.Pin = botton_vitess_Pin|botton_commande_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

