#ifndef __PERLS_COMMON_MAGIC_H__
#define __PERLS_COMMON_MAGIC_H__

#include <stdint.h>

uint64_t
magic64 (uint8_t B7, uint8_t B6, uint8_t B5, uint8_t B4, 
         uint8_t B3, uint8_t B2, uint8_t B1, uint8_t B0);

uint32_t
magic32 (uint8_t B3, uint8_t B2, uint8_t B1, uint8_t B0);

uint16_t
magic16 (uint8_t B1, uint8_t B0);

uint8_t
magic8 (uint8_t B0);

#endif //__PERLS_COMMON_MAGIC_H__
