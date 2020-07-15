/**
 * \file   board_am335x.c
 *
 * \brief  This file contains the implementation of board information for
 *         AM335x based boards.
 *
 * \copyright Copyright (C) 2013-2017 Texas Instruments Incorporated -
 *             http://www.ti.com/
 */

/*
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "types.h"
#include "stdlib.h"
#include "string.h"
#include "error.h"
#include "debug.h"
#include "board.h"
#include "platform.h"
#include "device.h"
#include "chipdb.h"
#include "board_priv.h"
#include "console_utils.h"
#include "i2c_utils.h"
#include "board_am335x.h"

#include "types.h"
#include "hw/hw_types.h"
#include "soc_control.h"
#include "board.h"
#include "am335x/ddr.h"
#include "am335x/clock.h"
#include "soc.h"
#include "wdt.h"
#include "gpio_app.h"
#include "hw/soc_am335x.h"
#include "am335x/hw_cm_wkup.h"
#include "am335x/hw_cm_per.h"
#include "hw/hw_control_am335x.h"
#include "console_utils.h"
#include "sbl_am335x_platform_ddr.h"
#include "sbl_platform.h"
#include "pmic_device.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void am335x_setup_pinmux(uint32_t offset, uint32_t value){
	HW_WR_REG32((SOC_CONTROL_REGS + offset), value);
}

/* PAD Control Fields */
#define SLEWCTRL        (0x1 << 6)
#define RXACTIVE        (0x1 << 5)
#define PULLUP_EN       (0x1 << 4) /* Pull UP Selection */
#define PULLUDEN        (0x0 << 3) /* Pull up enabled */
#define PULLUDDIS       (0x1 << 3) /* Pull up disabled */
#define MODE(val)		val

int32_t BoardAm335xInit(const boardInitParams_t *pInitPrms)
{
    /* Do any initialization/configuration required for AM335X boards. */
	// VTT gpio pinmux, gpmc_oen_ren pin, gpio2_3; 
	am335x_setup_pinmux(0x894, MODE(7)|PULLUP_EN);

	// SPI BUS
	// OFFSET(spi0_sclk), (MODE(0) | RXACTIVE | PULLUDEN)
	am335x_setup_pinmux(0x950, (MODE(0) | RXACTIVE | PULLUDEN));
	//spi0_d0, (MODE(0) | RXACTIVE | PULLUDEN | PULLUP_EN)
	am335x_setup_pinmux(0x954, (MODE(0) | RXACTIVE | PULLUDEN | PULLUP_EN));
	//OFFSET(spi0_d1), (MODE(0) | RXACTIVE | PULLUDEN)
	am335x_setup_pinmux(0x958, (MODE(0) | RXACTIVE | PULLUDEN));
	//OFFSET(spi0_cs0), (MODE(0) | RXACTIVE | PULLUDEN | PULLUP_EN)
	am335x_setup_pinmux(0x95C, (MODE(0) | RXACTIVE | PULLUDEN | PULLUP_EN));

	//MMC0: SDCard
	//{OFFSET(mmc0_dat3), (MODE(0) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x8F0, (MODE(0) | RXACTIVE | PULLUP_EN));	/* MMC0_DAT3 */
	//OFFSET(mmc0_dat2), (MODE(0) | RXACTIVE | PULLUP_EN)
	am335x_setup_pinmux(0x8F4, (MODE(0) | RXACTIVE | PULLUP_EN));	/* MMC0_DAT2 */
	//{OFFSET(mmc0_dat1), (MODE(0) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x8F8, (MODE(0) | RXACTIVE | PULLUP_EN));	/* MMC0_DAT1 */
	//{OFFSET(mmc0_dat0), (MODE(0) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x8FC, (MODE(0) | RXACTIVE | PULLUP_EN));	/* MMC0_DAT0 */
	//{OFFSET(mmc0_clk), (MODE(0) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x900, (MODE(0) | RXACTIVE | PULLUP_EN));	/* MMC0_CLK */
	//{OFFSET(mmc0_cmd), (MODE(0) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x904, (MODE(0) | RXACTIVE | PULLUP_EN));	/* MMC0_CMD */
	//{OFFSET(mcasp0_aclkr), (MODE(4) | RXACTIVE)},		
	am335x_setup_pinmux(0x9A0, (MODE(4) | RXACTIVE));				/* MMC0_WP */
	//{OFFSET(spi0_cs1), (MODE(5) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x960, (MODE(5) | RXACTIVE | PULLUP_EN));	/* MMC0_CD */

	//MMC1: eMMC.
	//{OFFSET(gpmc_ad7), (MODE(1) | RXACTIVE | PULLUP_EN)},		
	am335x_setup_pinmux(0x81C, (MODE(1) | RXACTIVE | PULLUP_EN));	/* MMC1_DAT7 */
	//{OFFSET(gpmc_ad6), (MODE(1) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x818, (MODE(1) | RXACTIVE | PULLUP_EN));	/* MMC1_DAT6 */
	//{OFFSET(gpmc_ad5), (MODE(1) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x814, (MODE(1) | RXACTIVE | PULLUP_EN));	/* MMC1_DAT5 */
	//{OFFSET(gpmc_ad4), (MODE(1) | RXACTIVE | PULLUP_EN)},		
	am335x_setup_pinmux(0x810, (MODE(1) | RXACTIVE | PULLUP_EN));	/* MMC1_DAT4 */
	//{OFFSET(gpmc_ad3), (MODE(1) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x80C, (MODE(1) | RXACTIVE | PULLUP_EN));	/* MMC1_DAT3 */	
	//{OFFSET(gpmc_ad2), (MODE(1) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x808, (MODE(1) | RXACTIVE | PULLUP_EN));	/* MMC1_DAT2 */
	//{OFFSET(gpmc_ad1), (MODE(1) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x804, (MODE(1) | RXACTIVE | PULLUP_EN));	/* MMC1_DAT1 */
	//{OFFSET(gpmc_ad0), (MODE(1) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x800, (MODE(1) | RXACTIVE | PULLUP_EN));	/* MMC1_DAT0 */
	//{OFFSET(gpmc_csn1), (MODE(2) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x880, (MODE(2) | RXACTIVE | PULLUP_EN));	/* MMC1_CLK */
	//{OFFSET(gpmc_csn2), (MODE(2) | RXACTIVE | PULLUP_EN)},	
	am335x_setup_pinmux(0x884, (MODE(2) | RXACTIVE | PULLUP_EN));	/* MMC1_CMD */

    return (S_PASS);
}

#ifndef DDRLESS
boardId_t BoardGetIdAm335x(void)
{
    return BOARD_UNKNOWN;
}

uint32_t BoardGetBaseBoardRevAm335x(void)
{
    return (1);
}

const char * BoardGetBaseBoardRevNameAm335x(void)
{
    return ("rsb4220");
}
#endif

uint32_t BoardGetDcRevAm335x(void)
{
    return (101);
}

const char * BoardGetDcRevNameAm335x(void)
{
    return ("v1.0");
}

const boardI2cData_t * BoardGetI2cDataAm335x(void)
{
    return (NULL);
}

const boardData_t *BoardGetDataAm335x(void)
{
    return NULL;
}

/** \brief BEAGLEBONE DDR2 configuration */
//static const sblPlatformDdrCfg_t AM335X_DDR2_CONFIG = {};


static  sblPlatformDdrCfg_t AM335X_DDR3_CONFIG =
{
    {
        DDR3_CONTROL_DDR_ADDRCTRL_IOCTRL_VALUE, /* ddr address io control */
        DDR3_CONTROL_DDR_DATA_IOCTRL_VALUE, /* ddr data io control */
        DDR3_CONTROL_DDR_IOCTRL_VALUE, /* ddr io control */
    }, /* ddrCtrl_t */
    {
        AM335x_DDR3_EMIF_DDR_PHY_CTRL_1, /* ddr phy control */
        AM335x_DDR3_EMIF_DDR_PHY_CTRL_1_DY_PWRDN, /* ddr phy control dynamic power down */

        AM335x_DDR3_EMIF_SDRAM_TIM_1_VALUE, /* sdram timing1 */
        AM335x_DDR3_EMIF_SDRAM_TIM_2_VALUE, /* sdram timing2 */
        AM335x_DDR3_EMIF_SDRAM_TIM_3_VALUE, /* sdram timing3 */
        AM335x_DDR3_EMIF_SDRAM_CONFIG_VALUE, /* sdram config */

        AM335x_DDR3_EMIF_SDRAM_REF_CTRL_VAL1, /* sdram ref control value 1 */
        DDR_NOTAPPLICABLE_VALUE,/* sdram ref control value 2 */
        DDR3_ZQ_CONFIG_VALUE /* ZQ config */
    }, /* ddrEmif_t */
    {
        AM335x_DDR3_CMD_SLAVE_RATIO, /* slave ratio */
        DDR_NOTAPPLICABLE_VALUE, /* slave force */
        DDR_NOTAPPLICABLE_VALUE, /* slave delay */
        AM335x_DDR3_CMD_INVERT_CLKOUT,	/* invert clockout */

        AM335x_DDR3_DATA_RD_DQS_SLAVE_RATIO, /* rd dqs slave ratio */
        AM335x_DDR3_DATA_WR_DQS_SLAVE_RATIO, /* wr dqs slave ratio */
        AM335x_DDR3_DATA_FIFO_WE_SLAVE_RATIO, /* fifo we slave ratio */
        AM335x_DDR3_DATA_WR_DATA_SLAVE_RATIO  /* wr data slave ratio */
    } /* ddrEmifPhy_t */
};

static void ddr_vtt_enable(void){
	gpioAppPinObj_t vtt_gpio_pin = {
    	3U,  /* pin number */
    	2U,  /* Instance number */
   		0x481AC000,  /* Instance Address */
    	{
        	GPIO_DIRECTION_OUTPUT, /* dir */
        	FALSE,  /* debounceEnable */
        	0U,     /* debounceTime */
        	FALSE,  /* intrEnable */
        	0U,     /* intrType */
        	0U,     /* intrLine */
        	FALSE,  /* wakeEnable */
        	0U,     /* wakeLine */
    	}, /* gpioAppPinCfg_t */
	};
	CONSOLEUtilsPrintf("Vtt enable... \n");

	GPIOAppInit(&vtt_gpio_pin);
	GPIOPinWrite(vtt_gpio_pin.instAddr, vtt_gpio_pin.pinNum, GPIO_PIN_HIGH);
}

static void board_ddr_init(uint32_t dynPwrDown){
	sblPlatformDdrCfg_t * ddrCfg = &AM335X_DDR3_CONFIG;
	uint32_t memType = SBL_PLATFORM_MEM_TYPE_DDR3;

	ddr_vtt_enable();
	
	/* EMIF Initialization */
	am335x_emif_init();
	am335x_ddr_config(dynPwrDown, ddrCfg, memType);
}

/*
* Board power init.
*/
static void  board_power_init(void){
	int32_t retVal = E_FAIL;
	int8_t mpuVddVol ;

	/*
	 * Depending on MPU clock and PG we will need a different
	 * VDD to drive at that speed.
	 */
	mpuVddVol = PMICGetMpuVdd(CONFIG_MPU_FREQ_M);
	
	/* Tell the TPS65910 to use i2c */
	retVal = PMICInit(CONFIG_I2C_INST_NUM,  CONFIG_PMIC_CTRL_ADDR);
	if(retVal != 0){
		CONSOLEUtilsPrintf(" Init Pmic failed r\n");
		return;	
	}

	/* First update MPU voltage. */
	retVal = PMICSetRailVoltage(PMIC_MODULE_MPU, mpuVddVol);
	if(retVal != 0){
		CONSOLEUtilsPrintf(" Set MPU volatage failed r\n");
		return;	
	}
	setup_mpu_pll();
	/* Second, update the CORE voltage. */
	retVal = PMICSetRailVoltage(PMIC_MODULE_CORE, 
				PMICGetMpuVdd(0));
	if(retVal != 0){
		CONSOLEUtilsPrintf(" Set Core volatage failed r\n");
		return;	
	}
	setup_core_pll();
}

/*
* Board pll init.
*/
static void  board_pll_init(uint8_t  early){
	uint32_t crystalFreqSel = 0U;
	uint32_t inputClk = 0U;
	uint32_t mpuDpllDiv = 0U;
	uint32_t mpuDpllMult = 0U;
	uint32_t mpuDpllPostDivM2 = 0U;
	uint32_t coreDpllDiv = 0U;
	uint32_t coreDpllMult = 0U;
	uint32_t coreDpllPostDivM4 = 0U;
	uint32_t coreDpllPostDivM5 = 0U;
	uint32_t coreDpllPostDivM6 = 0U;
	uint32_t perDpllDiv = 0U;
	uint32_t perDpllMult = 0U;
	uint32_t perSigmaDelatDiv = 0U;
	uint32_t perDpllPostDivM2 = 0U;
	uint32_t ddrDpllDiv = 0U;
	uint32_t ddrDpllMult = 0U;
	uint32_t ddrDpllPostDivM2 = 0U;
	uint32_t ddrDpllPostDivM4 = 0U;
	uint32_t dispDpllDiv = 0U;
	uint32_t dispDpllMult = 0U;
	uint32_t dispDpllPostDivM2 = 0U;


	/* Get input clock frequency. */
	crystalFreqSel = HW_RD_FIELD32_RAW(SOC_CONTROL_REGS + CONTROL_STATUS,
							CONTROL_STATUS_SYSBOOT1,
							CONTROL_STATUS_SYSBOOT1_SHIFT);

	switch(crystalFreqSel){
	case 0U:{
		inputClk = 19U;
		break;
	}

	case 1U: {
		/*
		* we are always use Xin clock 24MHz as input clock.
		*/	
		inputClk = 24U;
		break;
	}

	case 2U:{
		inputClk = 25U;
		break;
	}

	case 3U:{
		inputClk = 26U;
		break;
	}

	default:
		break;
    }

    /* Need to check for clock frequency from board data */

    /* EVM independent pll configuration. */

    /*
     * mpuDpllMult = mpuClk/mpuDpllDiv
     * mpuDpllPostDivM2 = mpuM2Clk/mpuClk
     */

    /*
     * coreDpllMult = coreClk/coreDpllDiv
     * coreDpllPostDivM4 = coreM4Clk/coreClk
     * coreDpllPostDivM5 = coreM5Clk/coreClk
     * coreDpllPostDivM6 = coreM6Clk/coreClk
     */

    /*
     * perDpllMult = perClk/perDpllDiv
     * perDpllPostDivM2 = perM2Clk/perClk
     * perSigmaDeltaDiv = (perClk/256U) + 1U
     */

     /*
      * dispDpllMult = dispClk/dispDpllDiv
      * dispDpllPostDivM2 = dispM2Clk/dispClk
      */

	/*
	*	mpuclk = Xin * mpuDpllMult / (mpuDpllDiv +1)/mpuDpllPostDivM2
	*/
#if CONFIG_MPU_FREQ_M == 600
	mpuDpllMult = 600U;
#else 
#if CONFIG_MPU_FREQ_M == 1000
	mpuDpllMult = 1000U;
#else
	mpuDpllMult = 800U;
#endif
#endif	
	if(early)
		mpuDpllMult = 500U;
		
	mpuDpllDiv = inputClk -1;
	mpuDpllPostDivM2 = 1U;

	if(early){
		coreDpllMult = 50U;
		coreDpllDiv = inputClk -1;
		coreDpllPostDivM4 = 1U;
		coreDpllPostDivM5 = 1U;
		coreDpllPostDivM6 = 1U;
	}else{
		coreDpllMult = 1000U;
		coreDpllDiv = inputClk -1;
		coreDpllPostDivM4 = 5U;
		coreDpllPostDivM5 = 4U;
		coreDpllPostDivM6 = 2U;
	}

	if(early){
		perDpllDiv = inputClk -1;
		perDpllMult = 960U;
		perSigmaDelatDiv = (((perDpllMult /(perDpllDiv + 1)) * inputClk / 250U) + 1U);
		perDpllPostDivM2 = 5U;

		dispDpllDiv = 0U;
		dispDpllMult = 2U;
		dispDpllPostDivM2 = 1U;
	}

	if(early){
#if 0
		/* DDR clock = 266Mhz*/
		ddrDpllMult = 266U;
#else
		/* DDR clock = 400Mhz*/
		ddrDpllMult = 400U;
#endif	
		ddrDpllDiv  = inputClk -1;
		ddrDpllPostDivM2 = 1U;
		ddrDpllPostDivM4 = 1U;
		CONSOLEUtilsPrintf("\rDDR Clock:              [%dMHz]\r\n", ddrDpllMult);
	}
	
	mpu_pll_config(mpuDpllMult, mpuDpllDiv, mpuDpllPostDivM2);
	CONSOLEUtilsPrintf("MPU PLL Clock:          [%dMHz]\r\n", mpuDpllMult);
	
	core_pll_config(coreDpllMult, coreDpllDiv, coreDpllPostDivM4,
							coreDpllPostDivM5, coreDpllPostDivM6);
	CONSOLEUtilsPrintf("Core PLL Clock:         [%dMHz]\r\n", coreDpllMult);
	
	if(early){
		CONSOLEUtilsPrintf("Peripheral PLL Clock:   [%dMHz]\r\n", perDpllMult);	
		per_pll_config(perDpllMult, perDpllDiv, perSigmaDelatDiv,
							perDpllPostDivM2);	
#ifndef DDRLESS
		ddr_pll_config(ddrDpllMult, ddrDpllDiv, ddrDpllPostDivM2,
							ddrDpllPostDivM4);

		/*
     		* Display PLL initialization can be skipped for applications
     		* not use display.
     		*/
		display_pll_config(dispDpllMult, dispDpllDiv, dispDpllPostDivM2);
#endif	
	}
}

void setup_mpu_pll(void){
	uint32_t crystalFreqSel = 0U;
	uint32_t inputClk = 0U;

	/* Get input clock frequency. */
	crystalFreqSel = HW_RD_FIELD32_RAW(SOC_CONTROL_REGS + CONTROL_STATUS,
							CONTROL_STATUS_SYSBOOT1,
							CONTROL_STATUS_SYSBOOT1_SHIFT);

	switch(crystalFreqSel){
	case 0U:{
		inputClk = 19U;
		break;
	}

	case 1U: {
		/*
		* we are always use Xin clock 24MHz as input clock.
		*/	
		inputClk = 24U;
		break;
	}

	case 2U:{
		inputClk = 25U;
		break;
	}

	case 3U:{
		inputClk = 26U;
		break;
	}

	default:
		break;
    }

	mpu_pll_config(CONFIG_MPU_FREQ_M, inputClk -1, 1);
	CONSOLEUtilsPrintf("MPU PLL Clock:          [%dMHz]\r\n", CONFIG_MPU_FREQ_M);
}

void setup_core_pll(void){
	uint32_t crystalFreqSel = 0U;
	uint32_t inputClk = 0U;

	/* Get input clock frequency. */
	crystalFreqSel = HW_RD_FIELD32_RAW(SOC_CONTROL_REGS + CONTROL_STATUS,
							CONTROL_STATUS_SYSBOOT1,
							CONTROL_STATUS_SYSBOOT1_SHIFT);

	switch(crystalFreqSel){
	case 0U:{
		inputClk = 19U;
		break;
	}

	case 1U: {
		/*
		* we are always use Xin clock 24MHz as input clock.
		*/	
		inputClk = 24U;
		break;
	}

	case 2U:{
		inputClk = 25U;
		break;
	}

	case 3U:{
		inputClk = 26U;
		break;
	}

	default:
		break;
    }

	core_pll_config(1000, inputClk -1, 5, 4, 2);
	CONSOLEUtilsPrintf("Core PLL Clock:         [%dMHz]\r\n", 1000);
}


#ifdef PRU_WKUP
static void PRU_ICSS_PRCM_Enable(void)
{
	const void * prussPruDramBase = (void *)SOC_PRU_ICSS_DATA_RAM0;
	const void * prussSharedDramBase = (void *)SOC_PRU_ICSS_SHARED_RAM;

	/* Reset the PRU ICSS and enable clock */
	PRCMModuleEnable(CHIPDB_MOD_ID_PRU_ICSS, 0, FALSE);
	memset((void *)prussPruDramBase, 0, (16*1024));
	memset((void *)prussSharedDramBase, 0, (12*1024));
}
#endif

/*
* am335x platform init function.
*/
void  board_early_init(void){
	uint32_t deviceVersion = 0U;
	uint32_t dynPwrDown = FALSE;

	deviceVersion = (HW_RD_REG32(SOC_CONTROL_REGS + CONTROL_DEVICE_ID) >>
                    CONTROL_DEVICE_ID_DEVREV_SHIFT);

	if(deviceVersion > 1U){
		dynPwrDown = TRUE;
	}

	/* This should be override by specific board.*/
	/* Initialize the UART console */
	CONSOLEUtilsInit();
	CONSOLEUtilsPrintf("\r\n\r\n################  Welocome the advloader. ################\r\n\r\n");

	BOARDInit(NULL);

	/* Print SoC & Board Information. */
	SOCPrintInfo();
	BOARDPrintInfo();

	/* 
	 * WDT1 is already running when the bootloader gets control
	 * Disable it to avoid "random" resets
	 * default 2 minites. 	
	 */
	HW_WR_REG32((SOC_WDT_1_REGS + WDT_WSPR) , 0xAAAAU);	
	while(HW_RD_REG32(SOC_WDT_1_REGS + WDT_WWPS) != 0x00U);

	HW_WR_REG32((SOC_WDT_1_REGS + WDT_WSPR) , 0x5555U);
	while(HW_RD_REG32(SOC_WDT_1_REGS + WDT_WWPS) != 0x00U);

	/* Set the PLL0 to generate 800MHz for ARM */
	board_pll_init(1);

	/* Enable the control module */
	HW_WR_REG32((SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL),
				CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE);
	
	/* DDR Initialization */
#ifndef DDRLESS
	CONSOLEUtilsPrintf("DDR:\n");
	board_ddr_init(dynPwrDown);
	CONSOLEUtilsPrintf("DDR init sucessful. \n");
#endif 

#ifdef PRU_WKUP
	PRU_ICSS_PRCM_Enable();
#endif

    /* Enable clock for Timer4 for SYS/BIOS application usage */
	/*enableModule(SOC_CM_PER_REGS, CM_PER_TIMER4_CLKCTRL,
					CM_PER_L4LS_CLKSTCTRL,
					CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER4_GCLK);
*/
	/* Init the board power. */
	board_power_init();
	
	int i;
	for(i = 0 ; i < 10000; i++){
		CONSOLEUtilsPuts("...");
	}
	/* later pll setup*/
	//board_pll_init(0);
	CONSOLEUtilsPuts("\r\n\r\n");
}
