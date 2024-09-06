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

//Arm object


#define DEGREES 2.7f // for 270 degrees use 2.7f
#define DOF 6

typedef struct
{
	BOOL active;
	uint16_t servo[DOF];
	uint16_t default_position[DOF];	// = { 90, 150, 90, 120, 120, 0 };
	uint16_t limit[DOF];			// = { 180, 180, 180, 200, 270, 80 };
}Arm;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_STRING_SIZE 128
#define TIMEOUT_DURATION 50 //ms
#define USB_SENDER 0
#define UART3_SENDER 1

// pinout

#define LED GPIO_PIN_7 		//PORT B
#define TRIG GPIO_PIN_7 	//PORT A

#define ECHO_1 GPIO_PIN_6	//PORT A
#define ECHO_2 GPIO_PIN_4	//PORT A
#define ECHO_3 GPIO_PIN_0	//PORT B
#define ECHO_4 GPIO_PIN_1	//PORT B

#define FLT_1 GPIO_PIN_2	//PORT B
#define FLT_2 GPIO_PIN_3	//PORT B
#define FLT_3 GPIO_PIN_4	//PORT B
#define FLT_4 GPIO_PIN_5	//PORT B


#define SERVO_1 GPIO_PIN_0
#define SERVO_2 GPIO_PIN_1
#define SERVO_3 GPIO_PIN_2
#define SERVO_4 GPIO_PIN_3
#define SERVO_5 GPIO_PIN_8
#define SERVO_6 GPIO_PIN_9
#define SERVO_7 GPIO_PIN_10

//#define speedOfSound 0.0343 // u cm/s
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
Arm roboticArm;
char str[MAX_STRING_SIZE];
char buffer[MAX_STRING_SIZE];
const float speedOfSound = 343.0f; 			// m/s
const float timeOffsetCorrection = 0.88f; 	// us
float distanceSensor[4] = {-1.0};
float Vbat = 0.0;

// uart variables
volatile char rx_buffer[MAX_STRING_SIZE];  	// Buffer for received data
volatile uint8_t rx_index = 0;        		// Index for buffer
volatile uint8_t received_uart3_command = 0;    		// Flag to indicate a full message is received
uint8_t temp_rx_byte;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

void respond(char* response, uint8_t destination);
void commandProcess(char *cmd, uint8_t sender);
float measureBattery();

// Arm functions
void initServoPWM();
Arm createArm();
void setServoPWMtimer(uint8_t serNum, int serVal);
BOOL inLimit(Arm *arm, uint16_t ser_val, int ser_num);
BOOL setPositions(Arm* arm, uint16_t *servos);
void setDefaultPositions(Arm* arm);

void initServoPWM()
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //Servo 1
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //Servo 2
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //Servo 3
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); //Servo 4
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //Servo 5
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //Servo 6
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //Servo 7
}

Arm createArm()
{
    Arm arm = {
    	.active = TRUE,
        .servo = {90, 150, 90, 120, 120, 0},
        .default_position = {90, 120, 90, 120, 120, 0},
        .limit = {180, 180, 180, 200, 270, 80}
    };
    initServoPWM();
    return arm;
}

void setServoPWMtimer(uint8_t serNum, int serVal)
{
	switch (serNum)
	{
		case 1: htim2.Instance->CCR1 = serVal; break;
		case 2: htim2.Instance->CCR2 = serVal; break;
		case 3: htim2.Instance->CCR3 = serVal; break;
		case 4: htim2.Instance->CCR4 = serVal; break;
		case 5: htim1.Instance->CCR1 = serVal; break;
		case 6: htim1.Instance->CCR2 = serVal; break;
		case 7: htim1.Instance->CCR3 = serVal; break;
		default: break;
	}
}

BOOL inLimit(Arm *arm, uint16_t ser_val, int ser_num)
{
	if (ser_val <= arm->limit[ser_num]) return TRUE;
	else return FALSE;
}

BOOL setPositions(Arm *arm, uint16_t *servos)
{
	float timerValue;
	for (uint8_t i = 0; i < DOF; i++)
	{
		if (inLimit(arm, servos[i], i))
		{
			if (abs(servos[i] - arm->servo[i]) < 20)
			{
				arm->servo[i] = servos[i];
				timerValue = (float)(servos[i]/DEGREES + 25); //Translate Degrees to PWM time value
				setServoPWMtimer(i + 1, (int)round(timerValue));
			}
			else
			{
				sprintf(str, "Irrational degree change servo %d set to %d from %d. \n", i + 1, servos[i], arm->servo[i]);
				USB_write(str);
				return FALSE;
			}
		}
		else
		{
			sprintf(str, "Out of limit servo %d set to %d limit is %d. \n", i + 1, servos[i], arm->limit[i]);
			USB_write(str);
			return FALSE; //NOT IN LIMIT!
		}
	}
	return TRUE;
}

void setDefaultPositions(Arm *arm)
{
	setPositions(arm, arm->default_position);
}


//Delay in microseconds using timer 4
void Delay_us(unsigned int us)
{
	if(us < 2) us = 2;
	TIM4->ARR = us - 1; //set value to aouto relode register
	TIM4->EGR = 1;		//re initiate the timer
	TIM4->SR &= ~1;		//reset the flag
	TIM4->CR1 |= 1;		//enable counter
	while((TIM4->SR & 0x0001) != 1);
	TIM4->SR &= ~1;
}

// measuring time
void start_timer()
{
    TIM4->ARR = 0xFFFF;     	// Set maximum reload value
    TIM4->EGR = TIM_EGR_UG; 	// Reinitialize the timer
    TIM4->SR &= ~TIM_SR_UIF; 	// Clear update interrupt flag
    TIM4->CR1 |= TIM_CR1_CEN; 	// Start the timer
}

uint32_t stop_timer()
{
    TIM4->CR1 &= ~TIM_CR1_CEN; 	// Stop the timer
    return TIM4->CNT; 			// Return the current timer value (elapsed time)
}

float measureBattery()
{
	float Vin;
	// Vin = Vref * ADC / 2^n -1   ,  n = 12bit
	// Start ADC conversion
	if (HAL_ADC_Start(&hadc2) != HAL_OK)
	{
	  // ADC start error handling
	}

	// Wait for conversion to complete
	if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) != HAL_OK)
	{
	  // ADC conversion error handling
	}

	// Read ADC value
	uint32_t analogValue = HAL_ADC_GetValue(&hadc2);
	Vin = ( 3.4f * analogValue ) / 4095;
	Vin = Vin * 1.07 * 6; // Measured correction
	return Vin;
}

float distance_sensor(int sen)
{

	float echo_time;
	uint32_t ECHOPin;
	GPIO_TypeDef * ECHOPort;

	// generate 10us trigger pulse
	HAL_GPIO_WritePin(GPIOA, TRIG, 1);
	Delay_us(10);
	HAL_GPIO_WritePin(GPIOA, TRIG, 0);

	switch(sen)
	{
		case 1:
			ECHOPin = ECHO_1;
			ECHOPort = GPIOA;
			break;
		case 2:
			ECHOPin = ECHO_2;
			ECHOPort = GPIOA;
			break;
		case 3:
			ECHOPin = ECHO_3;
			ECHOPort = GPIOB;
			break;
		case 4:
			ECHOPin = ECHO_4;
			ECHOPort = GPIOB;
			break;
		default: return -1.0f;
	}

	//wait for echo to rise
	uint32_t startTick = HAL_GetTick();
	while(HAL_GPIO_ReadPin(ECHOPort, ECHOPin) == 0)
		if (HAL_GetTick() - startTick > TIMEOUT_DURATION)
		{
			//USB_write("\nTimeout error. Sensor is absent.\n");
			return -1.0f;
		}

	start_timer();
	startTick = HAL_GetTick();
	//wait for echo to fall - sound wave returns
	while(HAL_GPIO_ReadPin(ECHOPort, ECHOPin) == 1)
		if (HAL_GetTick() - startTick > TIMEOUT_DURATION)
		{
			//USB_write("\nTimeout error. Distance too long.\n");
			return -1.0f;
		}

	echo_time = (float)stop_timer() + 2*timeOffsetCorrection;

	return (float)(echo_time*speedOfSound/2000000);
}

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
    	__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE); // Disable UART interrupts
        if (rxData != '\n')
        {
            rxBuffer[rxIndex++] = rxData;
            if (rxIndex >= sizeof(rxBuffer) - 1)
            {
                rxIndex = 0;
            }
        }
        else
        {
            rxBuffer[rxIndex] = '\0'; // Null-terminate the received string
            commandProcess((char *)rxBuffer, UART3_SENDER);
            rxIndex = 0;
            memset(rxBuffer, '\0', strlen(rxBuffer)); 	//clear buffer
        }
        HAL_UART_Receive_IT(huart, (uint8_t *)&rxData, 1); // Restart UART reception
        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE); // Re-enable UART interrupts
    }
}*/


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3 && received_uart3_command == 0)
    {
        if (temp_rx_byte == '\n')
        {
            received_uart3_command = 1;  	// Set the flag indicating message completion
            rx_buffer[rx_index] = '\0';  	// Null-terminate the string
            //rx_index = 0;  					// Reset buffer index for the next message
        }
        else
        {
            rx_buffer[rx_index++] = temp_rx_byte;  // Add received character to buffer
            if (rx_index >= sizeof(rx_buffer))
            {
                				// Prevent buffer overflow, handle error if necessary
                rx_index = 0;  	// Reset index
            }
        }
    }
    // Re-enable the UART receive interrupt for the next byte
    HAL_UART_Receive_IT(huart, &temp_rx_byte, 1);
}

void read_UART3()
{
	received_uart3_command = 0;  							// Reset the flag
	commandProcess((char*)rx_buffer, UART3_SENDER); // Process the message
	rx_index = 0; 	 								// Reset the buffer index for the next message
}


/*   		Command Handler
 *  		List of Commands:
 *  1. "arm" 		- Initiate Robotic Arm.
 *  2. "arm=#e"  	- Robotic Arm movement.
 *  3. "arm?"		- Return current arm positions
 *  4. "LED=#" 		- Turn PCB LED on(1) or off(0).
 *  5. "dist-#"		- Return distance from sensor #.
 *  6. "dist-all"	- Return all distance sensors.
 *  7. "battery" 	- Return battery voltage.
 *  8. "whoami"  	- Return sender ID.
 *  9. "reset"		- Reset self.
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

	else if(StrContains(cmd, "dist-all"))
	{
		sprintf(str,
					"Distance SEN = %.3f %.3f %.3f %.3f\n",
					distanceSensor[0],
					distanceSensor[1],
					distanceSensor[2],
					distanceSensor[3]);
		respond(str, sender);
	}

	else if(StrContains(cmd, "dist-"))
	{
		sscanf(cmd, "dist-%d", &result);
		sprintf(str, "Distance SEN %d = %.3f m\n", result, distance_sensor(result));
		respond(str, sender);
	}

	else if (StrContains(cmd, "battery"))
	{

		sprintf(str, "Voltage = %.3f V\n", Vbat);
		respond(str, sender);
	}


	else if(StrContains(cmd, "arm="))
	{
		uint16_t s[DOF];
		sscanf(cmd, "arm=%d %d %d %d %d %de", &s[0], &s[1], &s[2], &s[3], &s[4], &s[5]);
		//sprintf(str, "Received: %d %d %d %d %d %d\n",  s[0], s[1], s[2], s[3], s[4], s[5]);
		if (roboticArm.active == TRUE && setPositions(&roboticArm, s) == TRUE) respond("OK\n", sender);
		else if(roboticArm.active == FALSE) respond("NO\n", sender);
		else respond("out of limit break\n", sender);
	}

	else if(StrContains(cmd, "arm?"))
	{
		if (roboticArm.active == TRUE)
		{
			sprintf(str,
					"arm=%d %d %d %d %d %de\n",
					roboticArm.servo[0],
					roboticArm.servo[1],
					roboticArm.servo[2],
					roboticArm.servo[3],
					roboticArm.servo[4],
					roboticArm.servo[5]);
			respond(str, sender);
		}
		else respond("NO\n", sender);
	}

	else if(StrContains(cmd, "arm"))
	{
		roboticArm = createArm();
		setDefaultPositions(&roboticArm);
		respond("OK\n", sender);
	}

	else if(StrContains(cmd, "whoami"))
	{
		if (sender == USB_SENDER)
			respond("USB_SENDER\n", sender);
		else if (sender == UART3_SENDER)
			respond("UART3_SENDER\n", sender);
		else
			respond("UNKNOWN_SENDER\n", sender);
	}

	else if(StrContains(cmd, "reset"))
	{
		respond("Bye.\n", sender);
		NVIC_SystemReset();
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
		case UART3_SENDER:
			HAL_UART_Transmit(&huart3, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
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
  int distSenCount = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  roboticArm.active = FALSE;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_USB_Device_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart3, &temp_rx_byte, 1); // Enable UART3 interrupt
  NVIC_SetPriority(USART3_IRQn, 0);  // Set UART3 interrupt to the highest priority

  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //Servo 1

  HAL_GPIO_WritePin(GPIOB, FLT_1, 1);
  HAL_GPIO_WritePin(GPIOB, FLT_2, 1);
  HAL_GPIO_WritePin(GPIOB, FLT_3, 1);
  HAL_GPIO_WritePin(GPIOB, FLT_4, 1);
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

		else if (received_uart3_command)  	// Check if received commands via uart3
		{
			read_UART3();
		}

		else								// if not, read sensors
		{

			distanceSensor[distSenCount] = distance_sensor(distSenCount + 1);
			distSenCount++;
			if (distSenCount > 3)
			{
				Vbat = measureBattery();
				distSenCount = 0;
			}
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1922-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
	// calculating prescaler:  = APB1[Hz]/50[Hz]/Counter_Period - 1
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1922-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 96-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
