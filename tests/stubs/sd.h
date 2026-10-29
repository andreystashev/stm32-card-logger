#ifndef TEST_SD_H
#define TEST_SD_H
#include <stdint.h>
#define DEBUG 0
#define CT_BLOCK 8
#define SS_SD_SELECT() ((void)0)
#define SS_SD_DESELECT() ((void)0)
typedef int UART_HandleTypeDef;
typedef struct { uint8_t type; } sd_info_ptr;
void SD_PowerOn(void);
uint8_t sd_ini(void);
uint8_t SD_Read_Block(uint8_t *, uint32_t);
uint8_t SD_Write_Block(uint8_t *, uint32_t);
void SPI_Release(void);
uint8_t SPI_wait_ready(void);
#endif
#ifndef TEST_SD_LOW_LEVEL
#define TEST_SD_LOW_LEVEL
#include <stdio.h>
#include <string.h>
#define CT_MMC 1
#define CT_SD1 2
#define CT_SD2 4
#define SPI_BAUDRATEPRESCALER_128 128
#define HAL_OK 0
#define LD_ON ((void)0)
#define LD_OFF ((void)0)
typedef struct { struct { uint32_t BaudRatePrescaler; } Init; } SPI_HandleTypeDef;
int HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
int HAL_SPI_Init(SPI_HandleTypeDef *);
int HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
#endif
