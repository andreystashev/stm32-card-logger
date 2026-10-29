#include <assert.h>
#include <stdio.h>
#include "sd.h"

volatile uint16_t Timer1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;
static uint8_t input[1024];
static unsigned position;
static unsigned initializations;
static int initializing;
static uint32_t speeds[2];
int HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *out, uint8_t *in,
                           uint16_t length, uint32_t timeout)
{
    (void)spi; (void)out; (void)timeout;
    assert(length == 1 && position < sizeof(input));
    if (initializing) assert(spi->Init.BaudRatePrescaler == 128);
    *in = input[position++];
    return HAL_OK;
}
int HAL_SPI_Init(SPI_HandleTypeDef *spi)
{
    assert(initializations < 2);
    speeds[initializations++] = spi->Init.BaudRatePrescaler;
    return HAL_OK;
}
int HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *b, uint16_t n, uint32_t t)
{
    (void)u; (void)b; (void)n; (void)t;
    return HAL_OK;
}
#include "../Core/Src/sd.c"

int main(void)
{
    uint8_t block[512] = {0};
    memset(input, 0xFF, sizeof(input));
    input[8] = 0; /* CMD24 accepted. */
    input[525] = 0x05; /* Data accepted. */
    assert(SD_Write_Block(block, 4) == 0);
    position = 0;
    input[525] = 0x0D; /* Write error must not be mistaken for accepted. */
    assert(SD_Write_Block(block, 4) == 6);

    position = 0;
    memset(input, 0xFF, sizeof(input));
    input[18] = 1; /* CMD0 accepted after the initial 80 clock pulses. */
    input[28] = 1; /* CMD8 accepted; following OCR is invalid. */
    hspi2.Init.BaudRatePrescaler = 4;
    initializing = 1;
    assert(sd_ini() != 0 && sdinfo.type == 0);
    initializing = 0;
    assert(initializations == 2 && speeds[0] == 128 && speeds[1] == 4);
    assert(hspi2.Init.BaudRatePrescaler == 4);
    puts("SD response checks passed");
}
