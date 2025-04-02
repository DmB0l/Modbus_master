/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "modbus.h"
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODBUS_SILENT_PERIOD_MS 4  // 3.5 символа при 9600 бод ≈ 4 мс
#define RESPONSE_TIMEOUT_MS 500    // Таймаут ожидания ответа (500 мс)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t request_buffer[MODBUS_MAX_ADU_SIZE];
uint16_t req_length;

volatile uint32_t start_time = 0;

uint8_t rx_buffer[256];                // Буфер для приема данных
volatile uint8_t rx_real_size = 0;
volatile uint8_t func_code = 0;
volatile uint8_t rx_index = 0;

volatile uint8_t response_received = 0;  // Флаг завершения приема ответа
volatile uint32_t response_timer = 0;    // Таймер для отслеживания таймаута
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Send_Message(void);
void PrintHexArray8(const uint8_t *array, int length);
void PrintHexArray16(const uint16_t *array, int length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	start_time = HAL_GetTick();

	HAL_UART_Receive_IT(&huart1, rx_buffer, MODBUS_MIN_ADU_SIZE);

	// Начальная отправка команд
	//    uint8_t coils_data[] = { 0xCD, 0x01 }; // 11001101 00000001 (10 бит)
	//    ModbusFrame fc0f = { .slave_id = 1, .function_code = FC_WRITE_MULT_COILS, .address = 20, .quantity = 10, .data = coils_data, .data_length = 2 };
	//    modbus_create_request(&fc0f, request_buffer, &req_length);
	//    send_message();
	//    while (!response_received && (HAL_GetTick() - response_timer < RESPONSE_TIMEOUT_MS)); // Ожидание ответа
	//    response_received = 0;
	//
	//    uint8_t regs_data[] = { 0x00, 0x0A, 0x00, 0x0B, 0x00, 0x0C }; // 10, 11, 12
	//    ModbusFrame fc10 = { .slave_id = 1, .function_code = FC_WRITE_MULT_REG, .address = 10, .quantity = 3, .data = regs_data, .data_length = 6 };
	//    modbus_create_request(&fc10, request_buffer, &req_length);
	//    send_message();
	//    while (!response_received && (HAL_GetTick() - response_timer < RESPONSE_TIMEOUT_MS)); // Ожидание ответа
	//    response_received = 0;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// 15. Write Multiple Coils (FC 0F)
		uint8_t coils_data_loop[] = { 0xCD, 0x01 };
		ModbusFrame fc0f_loop = { .slave_id = 1, .function_code = FC_WRITE_MULT_COILS, .address = 20, .quantity = 10, .data = coils_data_loop,
				.data_length = 2 };
		modbus_create_request(&fc0f_loop, request_buffer, &req_length);
		Send_Message();
		while (!response_received && (HAL_GetTick() - response_timer < RESPONSE_TIMEOUT_MS)); // Ожидание ответа
		response_received = 0;

		// 16. Write Multiple Registers (FC 10)
		uint8_t regs_data_loop[] = { 0x00, 0x0A, 0x00, 0x0B, 0x00, 0x0C };
		ModbusFrame fc10_loop = { .slave_id = 1, .function_code = FC_WRITE_MULT_REG, .address = 10, .quantity = 3, .data = regs_data_loop,
				.data_length = 6 };
		modbus_create_request(&fc10_loop, request_buffer, &req_length);
		Send_Message();
		while (!response_received && (HAL_GetTick() - response_timer < RESPONSE_TIMEOUT_MS)); // Ожидание ответа
		response_received = 0;

		// Проверка результата записи coils (FC 01)
		ModbusFrame fc01_check = { .slave_id = 1, .function_code = FC_READ_COILS, .address = 20, .quantity = 10, .data = NULL, .data_length = 0 };
		modbus_create_request(&fc01_check, request_buffer, &req_length);
		Send_Message();
		while (!response_received && (HAL_GetTick() - response_timer < RESPONSE_TIMEOUT_MS)); // Ожидание ответа
		response_received = 0;

		// Проверка результата записи регистров (FC 03)
		ModbusFrame fc03_check = { .slave_id = 1, .function_code = FC_READ_HOLDING_REG, .address = 10, .quantity = 3, .data = NULL, .data_length = 0 };
		modbus_create_request(&fc03_check, request_buffer, &req_length);
		Send_Message();
		while (!response_received && (HAL_GetTick() - response_timer < RESPONSE_TIMEOUT_MS)); // Ожидание ответа
		response_received = 0;

		HAL_Delay(1000); // Основная задержка между циклами
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_GPIOA_CLK_ENABLE();

	HAL_GPIO_WritePin(DREnable_GPIO_Port, DREnable_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = DREnable_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DREnable_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		start_time = HAL_GetTick();
		if (func_code == 0) {
			rx_real_size = 0;
			func_code = rx_buffer[1];
			if (func_code == 0x01 || func_code == 0x02 || func_code == 0x03 || func_code == 0x04) {
				uint8_t ost_size = rx_buffer[2] - 1 + 2;
				rx_real_size = MODBUS_MIN_ADU_SIZE + ost_size;
				if (rx_real_size > sizeof(rx_buffer)) {
					func_code = 0;
					rx_real_size = 0;
					HAL_UART_Receive_IT(&huart1, rx_buffer, MODBUS_MIN_ADU_SIZE);
					return;
				}
				HAL_UART_Receive_IT(&huart1, &rx_buffer[MODBUS_MIN_ADU_SIZE], ost_size);
			} else if (func_code == 0x05 || func_code == 0x06 || func_code == 0x0F || func_code == 0x10) {
				uint8_t ost_size = 4;
				rx_real_size = ost_size + MODBUS_MIN_ADU_SIZE;
				HAL_UART_Receive_IT(&huart1, &rx_buffer[MODBUS_MIN_ADU_SIZE], ost_size);
			}
		} else {
			func_code = 0;

			printf("Reseived Response from slave: ");
			PrintHexArray8(rx_buffer, rx_real_size);

			uint16_t value_count;
			uint16_t *values = modbus_process_response_from_slave(rx_buffer, rx_real_size, &value_count);
			rx_real_size = 0;

			if (values != NULL) {
				printf("Reseived values from slave: ");
				PrintHexArray16(values, value_count);

				free(values);
			}

			response_received = 1; // Устанавливаем флаг завершения приема
			HAL_UART_Receive_IT(&huart1, rx_buffer, MODBUS_MIN_ADU_SIZE);
		}
	}
}

void Send_Message(void) {
	//    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY) {
	printf("Sending from master to slave: ");
	PrintHexArray8(request_buffer, req_length);
	HAL_GPIO_WritePin(GPIOA, DREnable_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, request_buffer, req_length, 100);
	response_timer = HAL_GetTick(); // Запускаем таймер ожидания ответа
	HAL_GPIO_WritePin(GPIOA, DREnable_Pin, GPIO_PIN_RESET);
	HAL_UART_Receive_IT(&huart1, rx_buffer, MODBUS_MIN_ADU_SIZE);
	//    }
}

void PrintHexArray8(const uint8_t *array, int length) {
	for (int i = 0; i < length; i++) {
		printf("%02X ", array[i]); // %02X для HEX в верхнем регистре
	}
	printf("\n");
}

void PrintHexArray16(const uint16_t *array, int length) {
	for (uint16_t i = 0; i < length; i++) {
		printf("%04X ", array[i]); // Вывод uint16_t в HEX
	}
	printf("\n");
}

// Перенаправление printf в UART2
int _write(int file, uint8_t *ptr, int len) {  // Лучше использовать uint8_t*
	HAL_UART_Transmit(&huart2, ptr, len, HAL_MAX_DELAY);
	return len;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}

/* USER CODE BEGIN Header */
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
