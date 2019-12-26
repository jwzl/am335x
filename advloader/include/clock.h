#ifndef _CLOCK_H_
#define _CLOCK_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
* Core Pll init
*/
void core_pll_config(uint32_t dpllMult,
                            uint32_t dpllDiv,
                            uint32_t dpllPostDivM4,
                            uint32_t dpllPostDivM5,
                            uint32_t dpllPostDivM6);

/*
* display pll init
*/
void display_pll_config(uint32_t dpllMult,
                               uint32_t dpllDiv,
                               uint32_t dpllPostDivM2);

/*
* per pll init
*/
void per_pll_config(uint32_t dpllMult,
                           uint32_t dpllDiv,
                           uint32_t sigmaDelatDiv,
                           uint32_t dpllPostDivM2);

/*
* DDR dpll init
*/
void ddr_pll_config(uint32_t dpllMult,
                           uint32_t dpllDiv,
                           uint32_t dpllPostDivM2,
                           uint32_t dpllPostDivM4);

/*
* mpu dpll init
*/
void mpu_pll_config(uint32_t dpllMult,
                           uint32_t dpllDiv,
                           uint32_t dpllPostDivM2);
#ifdef __cplusplus
}
#endif
#endif /*_CLOCK_H_*/
