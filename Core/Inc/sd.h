#ifndef SD_H_

#define SD_H_

//developed by Vladimir Radyuk telegram: @vv_msk2206

/*CONNECTION*/

//UART: A9 A10
//SD Module
//CS - A3
//SCK - B13
//MOSI - B15
//MOSO - B14
//VCC 5v from another STM32 source

/************/
#define DEBUG 0  //speed uart1 230400 for debug change 1 -> 0
//--------------------------------------------------

#include "stm32f1xx_hal.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

//--------------------------------------------------
//макросы для светодиода
#define CS_SD_GPIO_PORT GPIOA
#define CS_SD_PIN GPIO_PIN_3
#define SS_SD_SELECT() HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_RESET)
#define SS_SD_DESELECT() HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_SET)
#define LD_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //RED
#define LD_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); //RED
//-------------------------------------------------

/* Card type flags (CardType) */
//макросы типов карты
#define CT_MMC 0x01 /* MMC ver 3 */
#define CT_SD1 0x02 /* SD ver 1 */
#define CT_SD2 0x04 /* SD ver 2 */
#define CT_SDC (CT_SD1|CT_SD2) /* SD */
#define CT_BLOCK 0x08 /* Block addressing */

//--------------------------------------------------

//тип карты
typedef struct sd_info {

  volatile uint8_t type;

} sd_info_ptr;


void SPI_Release(void);
uint8_t SPI_wait_ready(void);
void SD_PowerOn(void);
uint8_t SD_Read_Block (uint8_t *buff, uint32_t lba);
uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba);
uint8_t sd_ini(void);




#endif /* SD_H_ */
