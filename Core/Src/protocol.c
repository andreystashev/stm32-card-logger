#include "protocol.h"




date Data;

/*
 * Побитовая проверка
 * uint16_t ui16 = 0x0611;

    uint8_t a0 = (ui16 >> 0) & 0x00ff;
    uint8_t a1 = (ui16 >> 8) & 0x00ff;

    // uint8_t byte_1 = in_val;
    // uint8_t byte_2 = in_val >> 8;
    if((a0 & 0x11) && (a1 & 0x06)) cout << "yes";
    else cout << "no";
//0×0613
void check_command(uint8_t mass)
{
	/*конструкция НЕ ВЕРНАЯ ИСПРОЛЬЗУЕТСЯ ДЛЯ ДЕМОНСТРАЦИИ*/
//	for(uint8_t i = 0; i < 512; i++)
//	{
//		(i == 0x06 && i+1 == 0x16) i += 2;
//	}
//	Data.bvm_info.
//}
