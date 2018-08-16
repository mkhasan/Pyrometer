#ifndef _CRC_H
#define _CRC_H

#include "stm32f1xx_hal.h"

#define POLYNOM4 0x19
#define POLYNOM8 0x1a9

int crcInit(uint32_t _plynom4, uint32_t _polynom8);
uint8_t crcCalc(uint32_t const message, int const msgLen, uint32_t polynom);
#endif