/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DOOR_TIME 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
bool MasterMode, Master_add;
uint8_t ID1[26], ID2[26], StoredID[26], IsMaster, data[65],
				MasterCard[4], numb_of_cards, numb_of_pages, mul_pages,
				id1bit, id2bit, hexid[4], readID[4],
				counter1, counter2, btn_time;
uint32_t MasterTime, Relay_Time, ct;
uint16_t btn1, btn2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void FullID(uint8_t ID[26]);
void Check_ID(uint8_t ID[26]);
void Door_open(void);
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_Delay(100);
	memset(data,0xFF,64);
	
	EEPROM_Read(0, 1, &IsMaster, 1);
	if(IsMaster!=1){
		Master_add=true;
	}
	else(EEPROM_Read(0, 2, MasterCard, 3));
	EEPROM_Read(0, 6, &numb_of_cards, 1);
	EEPROM_Read(0, 7, &numb_of_pages, 1);
	EEPROM_Read(0, 8, &mul_pages, 1);
	if(numb_of_cards==0xFF){
		numb_of_cards=0;
		EEPROM_Write(0, 6, &numb_of_cards, 1);	
	} 
	if(numb_of_pages==0xFF){
		numb_of_pages=0;
		EEPROM_Write(0, 7, &numb_of_pages, 1);	
	} 
	if(mul_pages==0xFF){
		mul_pages=1;
		EEPROM_Write(0, 8, &mul_pages, 1);	
	} 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	while(HAL_GPIO_ReadPin(Manager_Btn_GPIO_Port, Manager_Btn_Pin)==GPIO_PIN_RESET){
		btn1++; 
		HAL_Delay(100);
		if(btn1==30){ Master_add=true;
		}
	} btn1=0;
	while(HAL_GPIO_ReadPin(Wipe_Btn_GPIO_Port, Wipe_Btn_Pin)==GPIO_PIN_RESET){
		btn2++; 
		HAL_Delay(100);
		if(btn2==30){
			for (uint16_t i=1; i<=numb_of_pages*mul_pages+1; i++)
			{
				EEPROM_PageErase(i);
			}
			numb_of_cards=0;
			numb_of_pages=0;
			mul_pages=1;
			EEPROM_Write(0, 6, data, 1);
			EEPROM_Write(0, 7, data, 1);
			EEPROM_Write(0, 8, &mul_pages, 1);
		}
	} btn2=0;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA3 PA4 PA5
                           PA6 PA7 PA9 PA12
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_12
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB3 PB4 PB5
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief Interrupt callback function, opens door on button press and receives wiegand data
  * 			 If door button is pressed function jumps into Door_open function,
	*        If full wiegand data is received function adds 1 or 0 to IDX array
	*	@retval none
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin){
		case Door_Btn_Pin:
			Door_open();
			break;
		case W1_0_Pin: 
			ID1[id1bit] = 0;
			id1bit++;
			counter1=0;
			break;
		case W1_1_Pin: 
			ID1[id1bit] = 1;
			id1bit++;
			counter1=0;
			break;
		case W2_0_Pin: 
			ID2[id2bit] = 0;
			id2bit++;
			counter2=0;
			break;
		case W2_1_Pin: 
			ID2[id2bit] = 1;
			id2bit++;
			counter2=0;
			break; 
	}
}

/**
  * @brief Calls back every 10ms,
  * 			 If full wiegand data is received sends it into FULLID function,
	*				 Handles delay for open door
	*			 	 Turns off mastermode after 3 seconds
	* @param DOOR_TIME defines time door stays open
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	counter1++;
	counter2++;
	MasterTime++;
	Relay_Time++;
	if(Relay_Time >= DOOR_TIME){
		HAL_GPIO_WritePin(RelayM_GPIO_Port, RelayM_Pin, GPIO_PIN_SET);
   	HAL_GPIO_WritePin(RelayP_GPIO_Port, RelayP_Pin, GPIO_PIN_RESET); 
	}
	if(counter1 == 10){
			if(id1bit>25){
			FullID(ID1); 
			}
			id1bit=0;
			counter1=0;
		}
	if(counter2 == 10){
			if(id2bit>25){
		  FullID(ID2); 
			}
			counter2=0;
			id2bit=0;
		}
	if(MasterMode==true && MasterTime>=3000)
		MasterMode=false;
}

/**
  * @brief Called after wiegand reception ends
  * @param ID[26] 26 byte's received from wiegand 
	* @note goto is used to quickly exit nested for loops
  * @retval None
  */
void FullID(uint8_t ID[26]){
	uint8_t j, i, one=1;
		for(i=0; i<3; i++){
        for(j = 0; j <8; j++){
            hexid[i] |= (ID[j+1+i*8]<<j);
			}
		}	
	if(Master_add==true){
		EEPROM_Write(0, 2, hexid, 4);
		EEPROM_Write(0, 1, &one, 1);
 		Master_add=false;
		memcpy(MasterCard, hexid, 4);
		EEPROM_Read(0, 2, readID, 4);
	} 
	else if(MasterMode==true){
		for (uint16_t i=1; i<=numb_of_pages*mul_pages+1; i++){
			for (uint8_t j=0; j<16; j++){
				EEPROM_Read(i, j*4, readID, 4);
					if(!memcmp(hexid, readID, 4)){
						EEPROM_Write(i, j*4, data, 4);
						MasterTime=0;
						goto end;
					}
				}
			}
		EEPROM_Write(1+numb_of_pages*mul_pages, numb_of_cards*4, hexid, 4);
			numb_of_cards++;
			if(numb_of_cards==16){
			numb_of_cards=0;
			numb_of_pages++;
				if(numb_of_pages==0xFF){
					mul_pages=2;
					numb_of_pages=0;
					EEPROM_Write(0, 8, &mul_pages, 1);
				}
			EEPROM_Write(0, 6, &numb_of_cards, 1);
			EEPROM_Write(0, 7, &numb_of_pages, 1);
			} else 			
			EEPROM_Write(0, 6, &numb_of_cards, 1);
		MasterTime=0;
		} 
		
	if(!memcmp(MasterCard,hexid,4)){
			MasterMode=true;
			MasterTime=0;
		}
	else {
		for (uint16_t i=1; i<=numb_of_pages*mul_pages+1; i++){
		for (uint8_t j=0; j<16; j++){
			EEPROM_Read(i, j*4, readID, 4);
				if(!memcmp(hexid, readID, 4)){
				  Door_open();
					goto end;
					}
				}
			} 
		}
	end:	memset(hexid, 0, 4);
} 

/**
  * @brief switches polarity of RelayX pins
  * @param None
  * @retval None
	* @note Pins auto-reset after from timer callback after defined time
  */
void Door_open(){
		Relay_Time=0;
		HAL_GPIO_WritePin(RelayM_GPIO_Port, RelayM_Pin, GPIO_PIN_RESET);
   	HAL_GPIO_WritePin(RelayP_GPIO_Port, RelayP_Pin, GPIO_PIN_SET); 		
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
