#include <stdio.h>
#include <stdint.h>
#include "magic.h"

uint64_t
magic64 (uint8_t B7, uint8_t B6, uint8_t B5, uint8_t B4, 
         uint8_t B3, uint8_t B2, uint8_t B1, uint8_t B0)
{
    uint64_t magic = 
        (((uint64_t)B7)<<56) + (((uint64_t)B6)<<48) +
        (((uint64_t)B5)<<40) + (((uint64_t)B4)<<32) +
        (((uint64_t)B3)<<24) + (((uint64_t)B2)<<16) +
        (((uint64_t)B1)<<8)  + (((uint64_t)B0)<<0);
    return magic;
}

uint32_t
magic32 (uint8_t B3, uint8_t B2, uint8_t B1, uint8_t B0)
{
    uint32_t magic = 
        (((uint32_t)B3)<<24) + (((uint32_t)B2)<<16) + 
        (((uint32_t)B1)<<8)  + (((uint32_t)B0)<<0);
    return magic;
}

uint16_t
magic16 (uint8_t B1, uint8_t B0)
{
    uint16_t magic = 
        (((uint16_t)B1)<<8)  + (((uint16_t)B0)<<0);
    return magic;
}

uint8_t
magic8 (uint8_t B0)
{
    uint16_t magic = (((uint16_t)B0)<<0);
    return magic;
}

