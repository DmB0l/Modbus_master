#include "flashmemory.h"
#include <stdio.h>

// Глобальные переменные для параметров флеш-памяти
static uint32_t flash_size_kb;         // Размер флеш-памяти в КБ
static uint32_t flash_page_size;       // Размер страницы в байтах
static uint32_t flash_end_addr;        // Адрес конца флеш-памяти
static uint32_t flash_last_page_addr;  // Адрес последней страницы

/**
 * @brief Инициализация параметров флеш-памяти
 */
void Flash_Init(void) {
    // Считываем размер флеш-памяти в КБ
    flash_size_kb = *(__IO uint16_t *)(0x1FFFF7E0);
    printf("Flash size: %lu KB\n", flash_size_kb);

    // Определяем размер страницы
    flash_page_size = (flash_size_kb <= 128) ? 1024 : 2048;
    printf("Page size: %lu bytes\n", flash_page_size);

    // Вычисляем конец флеш-памяти
    flash_end_addr = 0x08000000 + (flash_size_kb * 1024) - 1;
    printf("Flash end address: 0x%08lX\n", flash_end_addr);

    // Вычисляем адрес последней страницы
    flash_last_page_addr = flash_end_addr - flash_page_size + 1;
    printf("Last page address: 0x%08lX\n", flash_last_page_addr);
}

/**
 * @brief Записывает 16-битные данные в последнюю страницу флеш-памяти
 * @param data Указатель на данные для записи
 * @param size Размер данных в байтах (должен быть кратен 2)
 * @retval HAL_StatusTypeDef Статус операции
 */
HAL_StatusTypeDef Flash_Write_Last_Page_16bit(uint16_t *data, uint32_t size) {
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0;

    // Проверяем размер данных
    if (size > flash_page_size || (size % 2 != 0)) {
        printf("Error: Size must be <= %lu bytes and multiple of 2\n", flash_page_size);
        return HAL_ERROR;
    }

    // Разблокируем флеш-память
    HAL_FLASH_Unlock();

    // Настраиваем стирание последней страницы
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = flash_last_page_addr;
    eraseInitStruct.NbPages = 1;

    // Стираем последнюю страницу
    status = HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
    if (status != HAL_OK) {
        printf("Flash erase failed: %d\n", status);
        HAL_FLASH_Lock();
        return status;
    }

    // Записываем данные (16-битные полуслова)
    for (uint32_t i = 0; i < size / 2; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_last_page_addr + (i * 2), data[i]);
        if (status != HAL_OK) {
            printf("Flash write failed at 0x%08lX: %d\n", flash_last_page_addr + (i * 2), status);
            HAL_FLASH_Lock();
            return status;
        }
    }

    HAL_FLASH_Lock();
    // printf("Flash write (16-bit) successful\n");
    return HAL_OK;
}

/**
 * @brief Записывает 32-битные данные в последнюю страницу флеш-памяти
 * @param data Указатель на данные для записи
 * @param size Размер данных в байтах (должен быть кратен 4)
 * @retval HAL_StatusTypeDef Статус операции
 */
HAL_StatusTypeDef Flash_Write_Last_Page_32bit(uint32_t *data, uint32_t size) {
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0;

    // Проверяем размер данных
    if (size > flash_page_size || (size % 4 != 0)) {
        printf("Error: Size must be <= %lu bytes and multiple of 4\n", flash_page_size);
        return HAL_ERROR;
    }

    // Разблокируем флеш-память
    HAL_FLASH_Unlock();

    // Настраиваем стирание последней страницы
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = flash_last_page_addr;
    eraseInitStruct.NbPages = 1;

    // Стираем последнюю страницу
    status = HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
    if (status != HAL_OK) {
        printf("Flash erase failed: %d\n", status);
        HAL_FLASH_Lock();
        return status;
    }

    // Записываем данные (32-битные слова)
    for (uint32_t i = 0; i < size / 4; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_last_page_addr + (i * 4), data[i]);
        if (status != HAL_OK) {
            printf("Flash write failed at 0x%08lX: %d\n", flash_last_page_addr + (i * 4), status);
            HAL_FLASH_Lock();
            return status;
        }
    }

    HAL_FLASH_Lock();
    // printf("Flash write (32-bit) successful\n");
    return HAL_OK;
}

/**
 * @brief Считывает 16-битные данные из последней страницы флеш-памяти
 * @param buffer Указатель на буфер для данных
 * @param size Размер данных в байтах (должен быть кратен 2)
 * @retval HAL_StatusTypeDef Статус операции
 */
HAL_StatusTypeDef Flash_Read_Last_Page_16bit(uint16_t *buffer, uint32_t size) {
    if (size > flash_page_size || (size % 2 != 0)) {
        printf("Error: Size must be <= %lu bytes and multiple of 2\n", flash_page_size);
        return HAL_ERROR;
    }

    for (uint32_t i = 0; i < size / 2; i++) {
        buffer[i] = *(__IO uint16_t *)(flash_last_page_addr + (i * 2));
    }

    // printf("Flash read (16-bit) successful\n");
    return HAL_OK;
}

/**
 * @brief Считывает 32-битные данные из последней страницы флеш-памяти
 * @param buffer Указатель на буфер для данных
 * @param size Размер данных в байтах (должен быть кратен 4)
 * @retval HAL_StatusTypeDef Статус операции
 */
HAL_StatusTypeDef Flash_Read_Last_Page_32bit(uint32_t *buffer, uint32_t size) {
    if (size > flash_page_size || (size % 4 != 0)) {
        printf("Error: Size must be <= %lu bytes and multiple of 4\n", flash_page_size);
        return HAL_ERROR;
    }

    for (uint32_t i = 0; i < size / 4; i++) {
        buffer[i] = *(__IO uint32_t *)(flash_last_page_addr + (i * 4));
    }

    // printf("Flash read (32-bit) successful\n");
    return HAL_OK;
}

/**
 * @brief Возвращает адрес последней страницы флеш-памяти
 * @retval uint32_t Адрес последней страницы
 */
uint32_t Flash_Get_Last_Page_Address(void) {
    return flash_last_page_addr;
}
