/**
 * \file   pmic_device.c
 *
 * \brief  Source file containing the abstraction of functional implementation
 *         for PMIC device to configure a rail.
 *
 *  \copyright Copyright (C) 2013 Texas Instruments Incorporated -
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
#include "hw/hw_types.h"
#include "error.h"
#include "device.h"
#include "ethernet.h"
#include "cpsw.h"
#include "am335x/chipdb_defs.h"
#include "chipdb.h"
#include "i2c_utils.h"
#include "board.h"
#include "pmic_device_tps65217.h"
#include "pmic_device_tps65910.h"
#include "pmic_device_tps65218.h"
#include "pmic_device.h"
#include "console_utils.h"
#include "delay_utils.h"

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/** \brief Macro value to identify the Output source for MPU rail. */
#define PMIC_MPU_RAIL_MASK              (0x000000FFU)

/** \brief Macro value to identify the Output source for CORE rail. */
#define PMIC_CORE_RAIL_MASK             (0x0000FF00U)

/** \brief Macro value identifying the shift value to identify Core Rail. */
#define PMIC_CORE_RAIL_SHIFT            (8U)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static uint32_t gBusNum;
static uint32_t gSlaveAddr;


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


int32_t PMICInit(uint32_t busNum, uint32_t slaveAddr)
{
	int32_t retVal = E_FAIL;
	i2cUtilsInitParams_t i2cUtilsParams = I2CUTILSINITPARAMS_DEFAULT;

	gBusNum = busNum;
	gSlaveAddr = slaveAddr;
	
	retVal = I2CUtilsInit(gBusNum, &i2cUtilsParams);

	if(S_PASS == retVal) {
		retVal = PMICDevTps65910Init(gBusNum, gSlaveAddr);
	}

	return retVal;
}

int32_t PMICSetRailVoltage(uint32_t module, uint8_t voltage)
{
	int32_t retVal = E_FAIL;
	
	retVal = PMICDevTps65910SetOutputVoltage(gBusNum, gSlaveAddr,
									module,  voltage);
            
	return retVal;
}

uint32_t PMICGetRailVoltage(uint32_t module)
{
	uint32_t retVal = 0U;

	retVal = PMICDevTps65910GetOutputVoltage(gBusNum, gSlaveAddr,
                                			module);

	return retVal;
}
uint8_t PMICGetMpuVdd(uint32_t frequency){
	return PMICDevTps65910GetMpuVdd(frequency);
}
