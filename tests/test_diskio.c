#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include "sd.h"
#include "ff_gen_drv.h"

UART_HandleTypeDef huart1;
char str1[60];
sd_info_ptr sdinfo;
static int init_result;
static unsigned calls, fail_at;
static uint32_t addresses[3];
static const uint8_t *pointers[3];
void SD_PowerOn(void) {}
uint8_t sd_ini(void) { return init_result; }
void SPI_Release(void) {}
uint8_t SPI_wait_ready(void) { return 0xFF; }
static uint8_t transfer(uint8_t *buffer, uint32_t address)
{
    assert(calls < 3);
    addresses[calls] = address;
    pointers[calls] = buffer;
    calls++;
    return calls == fail_at ? 5 : 0;
}
uint8_t SD_Read_Block(uint8_t *b, uint32_t a) { return transfer(b, a); }
uint8_t SD_Write_Block(uint8_t *b, uint32_t a) { return transfer(b, a); }

#include "../FATFS/Target/user_diskio.c"

int main(void)
{
    uint8_t buffer[1536] = {0};
    assert(USER_read(0, buffer, 1, 1) == RES_NOTRDY);
    assert(USER_initialize(1) == STA_NOINIT);
    assert(USER_initialize(0) == 0);
    assert(USER_read(0, NULL, 0, 1) == RES_PARERR);
    assert(USER_write(0, buffer, 0, 0) == RES_PARERR);

    sdinfo.type = CT_BLOCK;
    assert(USER_read(0, buffer, 4, 3) == RES_OK);
    assert(calls == 3 && addresses[0] == 4 && addresses[2] == 6);
    assert(pointers[1] == buffer + 512 && pointers[2] == buffer + 1024);
    calls = 0;
    sdinfo.type = 0;
    assert(USER_write(0, buffer, 4, 3) == RES_OK);
    assert(calls == 3 && addresses[0] == 2048 && addresses[2] == 3072);
    assert(pointers[2] == buffer + 1024);

    calls = 0; fail_at = 1;
    assert(USER_read(0, buffer, 0, 1) == RES_ERROR);
    assert(calls == 1);
    calls = 0;
    assert(USER_write(0, buffer, 0, 3) == RES_ERROR);
    assert(calls == 1);
    assert(USER_ioctl(0, GET_SECTOR_SIZE, NULL) == RES_PARERR);
    WORD size = 0;
    assert(USER_ioctl(0, GET_SECTOR_SIZE, &size) == RES_OK && size == 512);
    assert(USER_ioctl(0, CTRL_SYNC, NULL) == RES_OK);
    init_result = 1;
    assert(USER_initialize(0) == STA_NOINIT);
    assert(USER_write(0, buffer, 0, 1) == RES_NOTRDY);
    puts("disk I/O checks passed");
}
