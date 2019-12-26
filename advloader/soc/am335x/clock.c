#include <types.h>
#include "hw/hw_types.h"
#include "am335x/hw_cm_cefuse.h"
#include "am335x/hw_cm_device.h"
#include "am335x/hw_cm_dpll.h"
#include "am335x/hw_cm_mpu.h"
#include "am335x/hw_cm_per.h"
#include "am335x/hw_cm_rtc.h"
#include "am335x/hw_cm_wkup.h"
#include "hw/soc_am335x.h"
#include "hw/hw_control_am335x.h"

/*
* Core Pll init
*/
void core_pll_config(uint32_t dpllMult,
                            uint32_t dpllDiv,
                            uint32_t dpllPostDivM4,
                            uint32_t dpllPostDivM5,
                            uint32_t dpllPostDivM6)
{
    /* Enable the Core PLL */

    /* Put the PLL in bypass mode */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_CORE,
        CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN,
        CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN_SHIFT,
        CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN_DPLL_MN_BYP_MODE);

    /* Wait for DPLL to go in to bypass mode */

    while(0U == HW_RD_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_CORE,
        CM_WKUP_CM_IDLEST_DPLL_CORE_ST_MN_BYPASS,
        CM_WKUP_CM_IDLEST_DPLL_CORE_ST_MN_BYPASS_SHIFT));

    /* Set the multipler and divider values for the PLL */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_CORE,
        CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_MULT,
        CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_MULT_SHIFT,
        dpllMult);
    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_CORE,
        CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_DIV,
        CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_DIV_SHIFT,
        dpllDiv);

    /* Configure the High speed dividers */

    /* Set M4 divider */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M4_DPLL_CORE,
        CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_DIV,
        CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_DIV_SHIFT,
        dpllPostDivM4);

    /* Set M5 divider */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M5_DPLL_CORE,
        CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_DIV,
        CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_DIV_SHIFT,
        dpllPostDivM5);

    /* Set M6 divider */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M6_DPLL_CORE,
        CM_WKUP_CM_DIV_M6_DPLL_CORE_HSDIVIDER_CLKOUT3_DIV,
        CM_WKUP_CM_DIV_M6_DPLL_CORE_HSDIVIDER_CLKOUT3_DIV_SHIFT,
        dpllPostDivM6);

    /* Now LOCK the PLL by enabling it */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_CORE,
        CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN,
        CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN_SHIFT,
        CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN);

    while(0U == HW_RD_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_CORE,
                        CM_WKUP_CM_IDLEST_DPLL_CORE_ST_DPLL_CLK,
                        CM_WKUP_CM_IDLEST_DPLL_CORE_ST_DPLL_CLK_SHIFT));
}

/*
* display pll init
*/
void display_pll_config(uint32_t dpllMult,
                               uint32_t dpllDiv,
                               uint32_t dpllPostDivM2)
{
    /* Put the PLL in bypass mode */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DISP,
        CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN,
        CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN_SHIFT,
        CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN_DPLL_MN_BYP_MODE);

     /* Wait for DPLL to go in to bypass mode */

    while(0U == HW_RD_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_DISP,
        CM_WKUP_CM_IDLEST_DPLL_DISP_ST_MN_BYPASS,
        CM_WKUP_CM_IDLEST_DPLL_DISP_ST_MN_BYPASS_SHIFT));

    /* Set the multipler and divider values for the PLL */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_DISP,
        CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_MULT,
        CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_MULT_SHIFT,
        dpllMult);
    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_DISP,
        CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_DIV,
        CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_DIV_SHIFT,
        dpllDiv);

    /* Set the CLKOUT2 divider */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_DISP,
        CM_WKUP_CM_DIV_M2_DPLL_DISP_DPLL_CLKOUT_DIV,
        CM_WKUP_CM_DIV_M2_DPLL_DISP_DPLL_CLKOUT_DIV_SHIFT,
        dpllPostDivM2);

    /* Now LOCK the PLL by enabling it */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DISP,
        CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN,
        CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN_SHIFT,
        CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN);

    while(0U == HW_RD_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_DISP,
        CM_WKUP_CM_IDLEST_DPLL_DISP_ST_DPLL_CLK,
        CM_WKUP_CM_IDLEST_DPLL_DISP_ST_DPLL_CLK_SHIFT));
}

/*
* per pll init
*/
void per_pll_config(uint32_t dpllMult,
                           uint32_t dpllDiv,
                           uint32_t sigmaDelatDiv,
                           uint32_t dpllPostDivM2)
{
    /* Put the PLL in bypass mode */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_PER,
        CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN,
        CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN_SHIFT,
        CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN_DPLL_MN_BYP_MODE);

    /* Wait for DPLL to go in to bypass mode */

    while(0U == HW_RD_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_PER,
        CM_WKUP_CM_IDLEST_DPLL_PER_ST_MN_BYPASS,
        CM_WKUP_CM_IDLEST_DPLL_PER_ST_MN_BYPASS_SHIFT));

    /* Set the multipler and divider values for the PLL */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_PERIPH,
        CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_MULT,
        CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_MULT_SHIFT,
        dpllMult);
    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_PERIPH,
        CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_DIV,
        CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_DIV_SHIFT,
        dpllDiv);

    /* Set the CLKOUT2 divider */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_PER,
        CM_WKUP_CM_DIV_M2_DPLL_PER_DPLL_CLKOUT_DIV,
        CM_WKUP_CM_DIV_M2_DPLL_PER_DPLL_CLKOUT_DIV_SHIFT,
        dpllPostDivM2);
    /* Now LOCK the PLL by enabling it */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_PER,
        CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN,
        CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN_SHIFT,
        CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN);

    while(0U == HW_RD_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_PER,
        CM_WKUP_CM_IDLEST_DPLL_PER_ST_DPLL_CLK,
        CM_WKUP_CM_IDLEST_DPLL_PER_ST_DPLL_CLK_SHIFT));

}

/*
* DDR dpll init
*/
void ddr_pll_config(uint32_t dpllMult,
                           uint32_t dpllDiv,
                           uint32_t dpllPostDivM2,
                           uint32_t dpllPostDivM4)
{
    /* Put the PLL in bypass mode */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DDR,
        CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN,
        CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN_SHIFT,
        CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_DPLL_MN_BYP_MODE);

    /* Wait for DPLL to go in to bypass mode */

    while(0U == HW_RD_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_DDR,
        CM_WKUP_CM_IDLEST_DPLL_DDR_ST_MN_BYPASS,
        CM_WKUP_CM_IDLEST_DPLL_DDR_ST_MN_BYPASS_SHIFT));

    /* Set the multipler and divider values for the PLL */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_DDR,
        CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT,
        CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT_SHIFT,
        dpllMult);
    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_DDR,
        CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV,
        CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV_SHIFT,
        dpllDiv);

    /* Set the CLKOUT2 divider */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_DDR,
        CM_WKUP_CM_DIV_M2_DPLL_DDR_DPLL_CLKOUT_DIV,
        CM_WKUP_CM_DIV_M2_DPLL_DDR_DPLL_CLKOUT_DIV_SHIFT,
        dpllPostDivM2);

    /* Now LOCK the PLL by enabling it */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DDR,
        CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN,
        CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN_SHIFT,
        CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN);


    while(0U == HW_RD_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_DDR,
        CM_WKUP_CM_IDLEST_DPLL_DDR_ST_DPLL_CLK,
        CM_WKUP_CM_IDLEST_DPLL_DDR_ST_DPLL_CLK_SHIFT));
}

/*
* mpu dpll init
*/
void mpu_pll_config(uint32_t dpllMult,
                           uint32_t dpllDiv,
                           uint32_t dpllPostDivM2)
{
    /* Put the PLL in bypass mode */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU,
        CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN,
        CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_SHIFT,
        CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_DPLL_MN_BYP_MODE);

    /* Wait for DPLL to go in to bypass mode */

    while(0U == HW_RD_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_MPU,
        CM_WKUP_CM_IDLEST_DPLL_MPU_ST_MN_BYPASS,
        CM_WKUP_CM_IDLEST_DPLL_MPU_ST_MN_BYPASS_SHIFT));

    /* Set the multiplier and divider values for the PLL */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_MPU,
        CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT,
        CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT_SHIFT,
        dpllMult);
    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_MPU,
        CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV,
        CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV_SHIFT,
        dpllDiv);

    /* Set the CLKOUT2 divider */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_MPU,
        CM_WKUP_CM_DIV_M2_DPLL_MPU_DPLL_CLKOUT_DIV,
        CM_WKUP_CM_DIV_M2_DPLL_MPU_DPLL_CLKOUT_DIV_SHIFT,
        dpllPostDivM2);


    /* Now LOCK the PLL by enabling it */

    HW_WR_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU,
        CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN,
        CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_SHIFT,
        CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN);

    while(0U == HW_RD_FIELD32_RAW(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_MPU,
        CM_WKUP_CM_IDLEST_DPLL_MPU_ST_DPLL_CLK,
        CM_WKUP_CM_IDLEST_DPLL_MPU_ST_DPLL_CLK_SHIFT));
}

