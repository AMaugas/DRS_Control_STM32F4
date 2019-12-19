/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "HerkulexServo.h"
#include "servoControl.h"
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
unsigned char buttonPressed;
unsigned char receivedUART;
unsigned char passed;
uint8_t rData[9] = {0};
uint32_t nLoop;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void resetToFactoryDefault(uint8_t id);
void askStat(uint8_t id);
void reboot(uint8_t id);
void changeID(uint8_t oldID, uint8_t newID);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
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
	buttonPressed = 0;
	receivedUART = 0;
	passed = 0;

	HerkulexServoBus *herkulexBus = initializeServoBus(&huart5);
	// HerkulexServo *chevilleDroite = initializeServo(herkulexBus, 0x32);
	// HerkulexServo *genouDroit = initializeServo(herkulexBus, 0x31);
	HerkulexServo *oldServo = initializeServo(herkulexBus, 0xFD);
	HerkulexServo *newServo = initializeServo(herkulexBus, 0xCA);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_UART5_Init();
	MX_USART2_UART_Init();
	MX_DMA_Init();
	/* USER CODE BEGIN 2 */
	// setTorqueOn(chevilleDroite);
	// setTorqueOn(genouDroit);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (buttonPressed)
		{
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

			//			setPosition(chevilleDroite, 512, 100, HerkulexLed_Blue);
			//			setPosition(genouDroit, 512, 100, HerkulexLed_Blue);
			writeEep(oldServo, HerkulexEepRegister_ID, 0xCA);
			writeRam(oldServo, HerkulexRamRegister_ID, 0xCA);
			writeEep(newServo, HerkulexEepRegister_BaudRate, 0x22);
			//		  passed = 0;
			buttonPressed = 0;
		}

		//	  if (!passed)
		//	  {
		//		  HAL_UART_Receive_IT(&huart5, rData, sizeof(rData));
		//		  passed = 1;
		//	  }
		//
		//	  if (receivedUART)
		//	  {
		//		  printf("receivedUart: \n\r");
		//		  for (uint8_t i = 0; i < sizeof(rData); i++)
		//		  {
		//			  printf("0x%.2X ", rData[i]);
		//		  }
		//		  printf("\n\r");
		//		  receivedUART = 0;
		//	  }

		//	  if (receivedUART)

		//	  {
		//		  uint16_t bufferSize = huart5.RxXferSize;
		//
		//		  for (uint16_t i = 0; i < bufferSize; i++)
		//		  {
		//			  uint8_t character = huart5.pRxBuffPtr[i];
		//			  printf("Coucou");
		//		  }
		//		  receivedUART = 0;
		//	  }

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); // PORTA, PA5
		//	  HAL_Delay(500);
		//	  nLoop++;
		//	  printf("nLoop == %lu \n\r", nLoop);
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the CPU, AHB and APB busses clocks 
  */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks 
  */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART2 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	buttonPressed = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);
	/* NOTE: This function should not be modified, when the callback is needed,
	 * the HAL_UART_RxCpltCallback could be implemented in the user file
	 * */
	receivedUART = 1;
}

void resetToFactoryDefault(uint8_t id)
{
	uint8_t data_len = 0x09;
	uint8_t cmd = 0x08;
	uint8_t checksum1;
	uint8_t checksum2;
	uint8_t data1 = 0x00;
	uint8_t data2 = 0x00;

	uint8_t tData[0x09];

	checksum1 = data_len ^ id ^ cmd ^ data1 ^ data2;
	checksum1 = checksum1 & 0xFE;

	checksum2 = (~checksum1) & 0xFE;

	tData[0] = 0xFF;
	tData[1] = 0xFF;
	tData[2] = data_len;
	tData[3] = id;
	tData[4] = cmd;
	tData[5] = checksum1;
	tData[6] = checksum2;
	tData[7] = data1;
	tData[8] = data2;

	printf("\n\rData sent: ");
	for (uint8_t i = 0; i < sizeof(tData); i++)
	{
		printf("%.2X ", tData[i]);
	}
	printf("\n\r");

	HAL_UART_Transmit(&huart5, tData, sizeof(tData), 1000);
}

void askStat(uint8_t id)
{
	const uint8_t data_len = 0x07;
	uint8_t cmd = 0x07;
	uint8_t checksum1;
	uint8_t checksum2;

	uint8_t tData[data_len];

	checksum1 = data_len ^ id ^ cmd;
	checksum1 = checksum1 & 0xFE;

	checksum2 = (~checksum1) & 0xFE;

	tData[0] = 0xFF;
	tData[1] = 0xFF;
	tData[2] = data_len;
	tData[3] = id;
	tData[4] = cmd;
	tData[5] = checksum1;
	tData[6] = checksum2;

	printf("\n\rData sent: ");
	for (uint8_t i = 0; i < sizeof(tData); i++)
	{
		printf("%.2X ", tData[i]);
	}
	printf("\n\r");

	HAL_UART_Transmit(&huart5, tData, sizeof(tData), 1000);
}

void reboot(uint8_t id)
{
	const uint8_t data_len = 0x07;
	uint8_t cmd = 0x09;
	uint8_t checksum1;
	uint8_t checksum2;

	uint8_t tData[data_len];

	checksum1 = data_len ^ id ^ cmd;
	checksum1 = checksum1 & 0xFE;

	checksum2 = (~checksum1) & 0xFE;

	tData[0] = 0xFF;
	tData[1] = 0xFF;
	tData[2] = data_len;
	tData[3] = id;
	tData[4] = cmd;
	tData[5] = checksum1;
	tData[6] = checksum2;

	printf("\n\rData sent: ");
	for (uint8_t i = 0; i < sizeof(tData); i++)
	{
		printf("%.2X ", tData[i]);
	}
	printf("\n\r");

	HAL_UART_Transmit(&huart5, tData, sizeof(tData), 1000);
}

void changeID(uint8_t oldID, uint8_t newID)
{
	const uint8_t data_len = 0x0A;
	uint8_t cmd = 0x01;
	uint8_t checksum1;
	uint8_t checksum2;
	uint8_t data1 = 0x06;
	uint8_t data2 = 0x01;
	uint8_t data3 = newID;

	uint8_t tData[data_len];

	checksum1 = data_len ^ oldID ^ cmd ^ data1 ^ data2 ^ data3;
	checksum1 = checksum1 & 0xFE;

	checksum2 = (~checksum1) & 0xFE;

	tData[0] = 0xFF;
	tData[1] = 0xFF;
	tData[2] = data_len;
	tData[3] = oldID;
	tData[4] = cmd;
	tData[5] = checksum1;
	tData[6] = checksum2;
	tData[7] = data1;
	tData[8] = data2;
	tData[9] = data3;

	printf("\n\rData sent: ");
	for (uint8_t i = 0; i < sizeof(tData); i++)
	{
		printf("%.2X ", tData[i]);
	}
	printf("\n\r");

	HAL_UART_Transmit(&huart5, tData, sizeof(tData), 1000);

	cmd = 0x03;
	data1 = 0x00;
	checksum1 = data_len ^ oldID ^ cmd ^ data1 ^ data2 ^ data3;
	checksum1 = checksum1 & 0xFE;

	checksum2 = (~checksum1) & 0xFE;

	printf("\n\rData sent: ");
	for (uint8_t i = 0; i < sizeof(tData); i++)
	{
		printf("%.2X ", tData[i]);
	}
	printf("\n\r");

	HAL_UART_Transmit(&huart5, tData, sizeof(tData), 1000);

	askStat(newID);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
