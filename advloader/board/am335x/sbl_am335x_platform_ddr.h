/**
 *  \file  sbl_am335x_platform_ddr.h
 *
 *  \brief This file contain functions which configure EMIF and DDR.
 *
 *  \copyright Copyright (C) 2013-2017 Texas Instruments Incorporated -
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

#ifndef BL_PLATFORM_DDR_H_
#define BL_PLATFORM_DDR_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if 1
/**
 * DDR parameters as per the structure requirement
 */

#define DDR_NOTAPPLICABLE_VALUE 			(0x00000000U)

/**
 * DDR3 configurations for BEAGLEBONEBLACK and others.
 */




#define DDR3_EMIF_DDR_PHY_CTRL_VALUE                    (0x0E084007U)


/**
 * DDR3 configurations for BEAGLEBONEBLACK.
 */





/**
 *DDR Emif registers.
 */
#define AM335x_DDR3_EMIF_DDR_PHY_CTRL_1        	(0x07U)
#define AM335x_DDR3_EMIF_DDR_PHY_CTRL_1_DY_PWRDN   	(0x00100000U)

#define AM335x_DDR3_EMIF_SDRAM_TIM_1_VALUE         	(0x0AAAE51B)
#define AM335x_DDR3_EMIF_SDRAM_TIM_2_VALUE         	(0x26437FDA)
#define AM335x_DDR3_EMIF_SDRAM_TIM_3_VALUE         	(0x501F83FF)

#define AM335x_DDR3_EMIF_SDRAM_CONFIG_VALUE        	(0x61C05332)

#define AM335x_DDR3_EMIF_SDRAM_REF_CTRL_VAL1      	(0x00000C30)
#define DDR3_ZQ_CONFIG_VALUE                        (0x50074BE4)
/**
 * DDR EMIF physical configuration for command
 */
#define AM335x_DDR3_CMD_SLAVE_RATIO            (0x80)
#define AM335x_DDR3_CMD_INVERT_CLKOUT          (0x0)
/**
 * DDR EMIF physical configuration for data
 */
#define AM335x_DDR3_DATA_RD_DQS_SLAVE_RATIO    (0x39)
#define AM335x_DDR3_DATA_WR_DQS_SLAVE_RATIO    (0x3C)
#define AM335x_DDR3_DATA_FIFO_WE_SLAVE_RATIO   (0xB8)
#define AM335x_DDR3_DATA_WR_DATA_SLAVE_RATIO   (0x74)

#define DDR3_CONTROL_DDR_ADDRCTRL_IOCTRL_VALUE		(0x18BU)
#define DDR3_CONTROL_DDR_DATA_IOCTRL_VALUE		(0x18BU)
#define DDR3_CONTROL_DDR_IOCTRL_VALUE			(0xefffffffU)

#endif

#ifdef __cplusplus
}
#endif

#endif /* #ifndef BL_PLATFORM_DDR_H_ */
