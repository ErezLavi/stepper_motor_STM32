/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32g4xx_hal.h"
//#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>
//#include "usbd_cdc_if.h"
//#include "string.h"

#include "Utilities.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_STRING_SIZE 128
#define USB_SENDER 0
#define enable GPIO_PIN_5

// pinout

#define LED GPIO_PIN_7 		//PORT B
#define TRIG GPIO_PIN_7 	//PORT A
#define ECHO_1 GPIO_PIN_6	//PORT A
#define STEP_PIN GPIO_PIN_12	//PORT B
#define DIR_PIN GPIO_PIN_14		//PORT B
#define MOTOR_GPIO_PORT GPIOB

ADC_HandleTypeDef hadc3;
UART_HandleTypeDef huart3;
char str[MAX_STRING_SIZE];
char buffer[MAX_STRING_SIZE];
float Vbat = 0.0;
volatile int current_step_delay = 1000; // Default step delay
volatile int delay_updated = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

void respond(char* response, uint8_t destination);
void commandProcess(char *cmd, uint8_t sender);
int measureBattery();
void stepMotor(int direction, int step_delay);
void updateStepDelay(int new_delay);



void stepMotor(int direction, int step_delay) {
	int sensor;
	int dir = 0;
    // Set direction
    if (direction == 1)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET);

    // Step the motor infinitely
    while(*buffer != NULL)
    {
    	sensor = measureBattery();  //0-4095
    	sensor /= 100;


		//sprintf(str, "sensor: %f\n", sensor);
		if (sensor > 20)
		{
			dir = 0;
			sensor = 40-sensor;
		}
		else
			dir = 1;


		sprintf(str, "sensor: %d\n", sensor);
		USB_write(str);

		HAL_GPIO_WritePin(MOTOR_GPIO_PORT, DIR_PIN, dir);
		HAL_Delay(sensor);
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, STEP_PIN, GPIO_PIN_SET);
        HAL_Delay(sensor); // Pulse width for the step signal
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, STEP_PIN, GPIO_PIN_RESET);

    }
}

void updateStepDelay(int new_delay) {
	current_step_delay = new_delay;
	delay_updated = 1;
}


int measureBattery()
{
	float Vin = 0.0f;
	// Start ADC conversion
	if (HAL_ADC_Start(&hadc3) != HAL_OK)
	{
	  // ADC start error handling
	}

	// Wait for conversion to complete
	if (HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY) != HAL_OK)
	{
	  // ADC conversion error handling
	}

	// Read ADC value
	uint32_t analogValue = HAL_ADC_GetValue(&hadc3);
	Vin = ( 3.4f * analogValue ) / 4095;
	return analogValue;
}

/*   		Command Handler
 *  		List of Commands:
 *  1. "LED=#" 		- Turn PCB LED on(1) or off(0).
 *  2. "battery" 	- Return battery voltage.
 *  3. "reset"		- Reset self.
 *  4. "stepper=# #" - Move stepper motor.
 *
 */
void commandProcess(char *cmd, uint8_t sender)
{
	int result = 0;

	if(StrContains(cmd, "LED="))
	{
		sscanf(cmd, "LED=%d", &result);
		HAL_GPIO_WritePin(GPIOB, LED, result);
		respond("OK\n", sender);
	}

	else if (StrContains(cmd, "battery"))
	{
		sprintf(str, "Voltage = %.3f V\n", Vbat);
		respond(str, sender);
	}

	else if(StrContains(cmd, "reset"))
	{
		respond("Bye.\n", sender);
		NVIC_SystemReset();
	}


	else if(StrContains(cmd, "stepper=")) {
	    int direction, step_delay;
	    sscanf(cmd, "stepper=%d %d", &direction, &step_delay);
	    updateStepDelay(step_delay); // Update the step delay
	    stepMotor(direction, step_delay); // Call stepMotor with initial delay value
	    respond("Stepper", sender);
	}

	else respond("Unknown command\n", sender);

	memset(cmd, '\0', strlen(cmd)); 	//clear buffer
	memset(str, '\0', sizeof(str));		//clear string
}

void respond(char* response, uint8_t destination)
{
	switch (destination)
	{
		case USB_SENDER:
			USB_write(response);
			break;
		default: break;
	}
}

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
  MX_USART3_UART_Init();
  MX_USB_Device_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //Servo 1

  HAL_GPIO_WritePin(GPIOB, FLT_1, 1);
  HAL_GPIO_WritePin(GPIOB, FLT_2, 1);
  HAL_GPIO_WritePin(GPIOB, FLT_3, 1);
  HAL_GPIO_WritePin(GPIOB, FLT_4, 1);
  HAL_GPIO_WritePin(GPIOA, enable, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_Delay(10);

		if(*buffer != NULL) 				// Check if received commands via usb
		{
			commandProcess(buffer, USB_SENDER);
		}

		else								// if not, read sensors
		{
			Vbat = measureBattery();
		}


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}//end while
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FLT_1_Pin|FLT_2_Pin|FLT_3_Pin|FLT_4_Pin
                          |LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ECHO_2_Pin ECHO_1_Pin */
  GPIO_InitStruct.Pin = ECHO_2_Pin|ECHO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO_3_Pin ECHO_4_Pin */
  GPIO_InitStruct.Pin = ECHO_3_Pin|ECHO_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FLT_1_Pin FLT_2_Pin FLT_3_Pin FLT_4_Pin
                           LED_Pin */
  GPIO_InitStruct.Pin = FLT_1_Pin|FLT_2_Pin|FLT_3_Pin|FLT_4_Pin
                          |LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
