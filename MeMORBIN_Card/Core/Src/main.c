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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MEM_ID 0x81
#define MEM_READ 0x52
#define MEM_WRITE 0x57
#define IDLE 0x00

#define BUFFER_SIZE 200
#define SEND_BUFFER_SIZE 145
#define RECEIVE_BUFFER_SIZE 148

#define MEMCARD_BAD_CHECKSUM 0x4E
#define MEMCARD_BAD_SECTOR 0xFF
#define MEMCARD_WRITE_GOOD 0x47
#define MEMCARD_READ_GOOD 0x47

#define TIMEOUT 500

#define READ_COMMAND 'R'
#define WRITE_COMMAND 'W'

typedef enum Command{
	READ,
	WRITE,
	NONE
} Command;

typedef enum State{
	START,
	WAITING_FOR_COMMAND,
	RECEIVED_COMMAND,
	WAITING_FOR_DATA,
	DONE
} State;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//Vou usar variáveis globais mesmo por enquanto
uint8_t command;
Command current_command = NONE;
uint8_t addr_buffer[2];
uint8_t data_buffer[128];
uint8_t buffer[130];
State state = START;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t get_checksum(uint8_t* address, uint8_t* data);
uint8_t write_to_memory(uint8_t* address, uint8_t* data);
uint8_t read_from_memory(uint8_t* address, uint8_t* received_data);
uint8_t check_write_errors(uint8_t* data);
uint8_t check_read_errors(uint8_t* data);
void transmit_uart_message(char*message, uint8_t size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void hex2int(uint8_t* ch){
	uint8_t msb, lsb;
    if (ch[0] >= '0' && ch[0] <= '9')
        msb = ch[0] - '0';
    else if (ch[0] >= 'A' && ch[0] <= 'F')
        msb =  ch[0] - 'A' + 10;
    else
        msb = ch[0] - 'a' + 10;

    if (ch[1] >= '0' && ch[1] <= '9')
        lsb = ch[1] - '0';
    else if (ch[1] >= 'A' && ch[1] <= 'F')
        lsb =  ch[1] - 'A' + 10;
    else
        lsb = ch[1] - 'a' + 10;

    uint16_t tmp = msb*16+lsb;

    ch[0] = ((uint8_t*) &tmp)[0];
    ch[1] = ((uint8_t*) &tmp)[1];
}

uint8_t get_checksum(uint8_t* address, uint8_t* data){
	uint8_t check_sum = address[0]^address[1];

	int i;
	for(i = 0; i < 128; i++)
		check_sum ^= data[i];

	return check_sum;
}

uint8_t check_write_errors(uint8_t* data){
	return data[137];
}

uint8_t write_to_memory(uint8_t* address, uint8_t* data){
  uint8_t send_buffer[SEND_BUFFER_SIZE];
  uint8_t receive_buffer[SEND_BUFFER_SIZE];

  send_buffer[0] = MEM_ID;
  send_buffer[1] = MEM_WRITE;

  send_buffer[2] = IDLE;
  send_buffer[3] = IDLE;

  send_buffer[4] = address[0];
  send_buffer[5] = address[1];

  int i;
  for(i = 0; i < 128; i++)
    send_buffer[6+i] = data[i];

  send_buffer[134] = get_checksum(address, data);

  for(i = 0; i < 10; i++)
    send_buffer[135+i] = IDLE;

  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  HAL_SPI_TransmitReceive(&hspi1, send_buffer, receive_buffer, SEND_BUFFER_SIZE, TIMEOUT);

  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  return check_write_errors(receive_buffer);
}

uint8_t check_read_errors(uint8_t* data){
	return data[139];
}

uint8_t read_from_memory(uint8_t* address, uint8_t* received_data){
  uint8_t send_buffer[RECEIVE_BUFFER_SIZE];
  uint8_t receive_buffer[RECEIVE_BUFFER_SIZE];

  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  send_buffer[0] = MEM_ID;
  send_buffer[1] = MEM_READ;

  send_buffer[2] = IDLE;
  send_buffer[3] = IDLE;

  send_buffer[4] = address[0];
  send_buffer[5] = address[1];

  send_buffer[6] = IDLE;
  send_buffer[7] = IDLE;
  send_buffer[8] = IDLE;
  send_buffer[9] = IDLE;

  int i;
  for(i = 0; i < 130; i++)
    send_buffer[10+i] = IDLE;

  for(i = 0; i < 8; i++)
    send_buffer[140+i] = IDLE;

  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  HAL_SPI_TransmitReceive(&hspi1, send_buffer, receive_buffer, RECEIVE_BUFFER_SIZE, TIMEOUT);

  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  for(i=0; i < 128; i++)
	  received_data[i] = receive_buffer[10+i];

  return check_read_errors(receive_buffer);
}

void transmit_uart_message(char* message, uint8_t size){
	// deixei em uma funcao por enquanto mesmo sendo so isso para
	// caso queiramos mudar a formatacao das mensagens depois
	HAL_UART_Transmit(&huart1, (uint8_t*) message, size, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, (uint8_t*) "\n\r", 2, HAL_MAX_DELAY);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(current_command == NONE){
		if(command == READ_COMMAND)
			current_command = READ;
		else if(command == WRITE_COMMAND)
			current_command = WRITE;

		state = current_command == NONE ? START : RECEIVED_COMMAND;

	}else if(current_command == WRITE){
		addr_buffer[0] = buffer[0];
		addr_buffer[1] = buffer[1];
		hex2int(addr_buffer);

		int i;
		for(i = 0; i < 128; i++)
			data_buffer[i] = buffer[2+i];

		state = DONE;
	}else if(current_command == READ){
		addr_buffer[0] = buffer[0];
		addr_buffer[1] = buffer[1];
		hex2int(addr_buffer);

		state = DONE;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t status = 0;
  int receive_size = 0;
  while (1)
  {
    switch(state){
      case START:
    	transmit_uart_message("Esperando Comando", 17);
        HAL_UART_Receive_IT(&huart1, &command, 1);
        state = WAITING_FOR_COMMAND;
        break;

      case WAITING_FOR_COMMAND:
    	  break;

      case RECEIVED_COMMAND:
    	  transmit_uart_message("Comando Recebido", 16);
    	  char comm = '0'+current_command;
    	  transmit_uart_message(&comm, 1);

    	  receive_size = current_command == READ ? 2 : 130;
          HAL_UART_Receive_IT(&huart1, buffer, receive_size);
          state = WAITING_FOR_DATA;
    	  transmit_uart_message("Esperando Dados", 15);
          break;

      case WAITING_FOR_DATA:
    	  break;

      case DONE:
    	transmit_uart_message("Dados Recebidos", 15);
    	char stat[3];
      	if(current_command == READ){
      	    status = read_from_memory(addr_buffer, data_buffer);
      	    sprintf(stat, "%02X", status);
      	    transmit_uart_message("STATUS:", 7);
      	    transmit_uart_message(stat, 1);
      	    transmit_uart_message("DATA:", 5);
      	    transmit_uart_message((char*) data_buffer, 128);
      	}else if(current_command == WRITE){
      	   status = write_to_memory(addr_buffer, data_buffer);
     	   sprintf(stat, "%02X", status);
      	   transmit_uart_message("STATUS:", 7);
      	   transmit_uart_message(stat, 1);
      	 }

      	current_command = NONE;
      	state = START;
      	break;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA8
                           PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_8
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

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
