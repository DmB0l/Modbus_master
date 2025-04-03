#ifndef FLASH_MEMORY_H
#define FLASH_MEMORY_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// Инициализация параметров флеш-памяти
void Flash_Init(void);

// Функции записи в последнюю страницу флеш-памяти
HAL_StatusTypeDef Flash_Write_Last_Page_16bit(uint16_t *data, uint32_t size);
HAL_StatusTypeDef Flash_Write_Last_Page_32bit(uint32_t *data, uint32_t size);

// Функции чтения из последней страницы флеш-памяти
HAL_StatusTypeDef Flash_Read_Last_Page_16bit(uint16_t *buffer, uint32_t size);
HAL_StatusTypeDef Flash_Read_Last_Page_32bit(uint32_t *buffer, uint32_t size);

// Получение адреса последней страницы (для отладки или информации)
uint32_t Flash_Get_Last_Page_Address(void);

#endif /* FLASH_MEMORY_H */
