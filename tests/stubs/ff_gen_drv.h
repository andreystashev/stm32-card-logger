#ifndef TEST_FF_GEN_DRV_H
#define TEST_FF_GEN_DRV_H
#include <stdint.h>
#define _USE_WRITE 1
#define _USE_IOCTL 1
typedef uint8_t BYTE;
typedef uint16_t WORD;
typedef uint32_t DWORD;
typedef unsigned int UINT;
typedef BYTE DSTATUS;
typedef enum { RES_OK, RES_ERROR, RES_WRPRT, RES_NOTRDY, RES_PARERR } DRESULT;
#define STA_NOINIT 1
#define STA_PROTECT 4
#define CTRL_SYNC 0
#define GET_SECTOR_SIZE 2
typedef struct {
    DSTATUS (*initialize)(BYTE);
    DSTATUS (*status)(BYTE);
    DRESULT (*read)(BYTE, BYTE *, DWORD, UINT);
    DRESULT (*write)(BYTE, const BYTE *, DWORD, UINT);
    DRESULT (*ioctl)(BYTE, BYTE, void *);
} Diskio_drvTypeDef;
#endif
