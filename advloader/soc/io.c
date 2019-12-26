/*
*  io.c
*/
#include "types.h"
/* ========================================================================== */
/*                      		Function Definitions                          */
/* ========================================================================== */

uint32_t HW_RD_REG32(uint32_t addr)
{
    uint32_t regVal = *(volatile uint32_t *) addr;
#ifndef MEM_BARRIER_DISABLE
    asm("    dsb");
#endif
    return (regVal);
}

void HW_WR_REG32(uint32_t addr, uint32_t value)
{
    *(volatile uint32_t *) addr = value;
#ifndef MEM_BARRIER_DISABLE
    asm("    dsb");
#endif
    return;
}

uint16_t HW_RD_REG16(uint32_t addr)
{
    uint16_t regVal = *(volatile uint16_t *) addr;
#ifndef MEM_BARRIER_DISABLE
    asm("    dsb");
#endif
    return (regVal);
}

void HW_WR_REG16(uint32_t addr, uint16_t value)
{
    *(volatile uint16_t *) addr = value;
#ifndef MEM_BARRIER_DISABLE
    asm("    dsb");
#endif
    return;
}

uint8_t HW_RD_REG8(uint32_t addr)
{
    uint8_t regVal = *(volatile uint8_t *) addr;
#ifndef MEM_BARRIER_DISABLE
    asm("    dsb");
#endif
    return (regVal);
}

void HW_WR_REG8(uint32_t addr, uint8_t value)
{
    *(volatile uint8_t *) addr = value;
#ifndef MEM_BARRIER_DISABLE
    asm("    dsb");
#endif
    return;
}

void HW_WR_FIELD32_RAW(uint32_t addr,
                                     uint32_t mask,
                                     uint32_t shift,
                                     uint32_t value)
{
    uint32_t regVal = *(volatile uint32_t *) addr;
    regVal &= (~mask);
    regVal |= (value << shift) & mask;
    *(volatile uint32_t *) addr = regVal;
#ifndef MEM_BARRIER_DISABLE
    asm("    dsb");
#endif
    return;
}

void HW_WR_FIELD16_RAW(uint32_t addr,
                                     uint16_t mask,
                                     uint32_t shift,
                                     uint16_t value)
{
    uint16_t regVal = *(volatile uint16_t *) addr;
    regVal &= (~mask);
    regVal |= (value << shift) & mask;
    *(volatile uint16_t *) addr = regVal;
#ifndef MEM_BARRIER_DISABLE
    asm("    dsb");
#endif
    return;
}

void HW_WR_FIELD8_RAW(uint32_t addr,
                                    uint8_t mask,
                                    uint32_t shift,
                                    uint8_t value)
{
    uint8_t regVal = *(volatile uint8_t *) addr;
    regVal &= (~mask);
    regVal |= (value << shift) & mask;
    *(volatile uint8_t *) addr = regVal;
#ifndef MEM_BARRIER_DISABLE
    asm("    dsb");
#endif
    return;
}

uint32_t HW_RD_FIELD32_RAW(uint32_t addr,
                                         uint32_t mask,
                                         uint32_t shift)
{
    uint32_t regVal = *(volatile uint32_t *) addr;
    regVal = (regVal & mask) >> shift;
#ifndef MEM_BARRIER_DISABLE
    asm("    dsb");
#endif
    return (regVal);
}

uint16_t HW_RD_FIELD16_RAW(uint32_t addr,
                                         uint16_t mask,
                                         uint32_t shift)
{
    uint16_t regVal = *(volatile uint16_t *) addr;
    regVal = (regVal & mask) >> shift;
#ifndef MEM_BARRIER_DISABLE
    asm("    dsb");
#endif
    return (regVal);
}

uint8_t HW_RD_FIELD8_RAW(uint32_t addr,
                                       uint8_t mask,
                                       uint32_t shift)
{
    uint8_t regVal = *(volatile uint8_t *) addr;
    regVal = (regVal & mask) >> shift;
#ifndef MEM_BARRIER_DISABLE
    asm("    dsb");
#endif
    return (regVal);
}
