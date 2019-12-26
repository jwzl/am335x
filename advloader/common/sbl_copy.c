/**
 *  \file  sbl_platform_uart.c
 *
 * \brief  This file contain functions which compute ASCII for standard
 *         operations.
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


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <types.h>
#include "error.h"
#include "console_utils.h"
#include "sbl.h"
#include "sbl_platform.h"
#include "hw/hw_types.h"
#include "sbl_mmcsd.h"
#include "sbl_uart.h"
#include "sbl_qspi.h"
#include "sbl_nand.h"
#include "sbl_mcspi.h"
#include "sbl_copy.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                            Global Variables Declarations                   */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * \brief This function copies Image
 *
 * \param  none
 *
 * \return none
 */
int32_t SBLCopyImage(uint32_t *pEntryPoint, uint32_t* imageSize)
{
	uint32_t instNum = 0U;
	int32_t status ;

	*imageSize = 0;

	/* boot form MCSPI flash.*/
	CONSOLEUtilsPrintf("boot form MCSPI flash..........\r\n");
	status = SBLMcspiCopy(pEntryPoint, instNum); 
	if(status == S_PASS){
		return status;
	}
	/* boot form SD card*/
	CONSOLEUtilsPrintf("boot form SDCard..........\r\n");
	status = SBLMmcsdCopy(pEntryPoint, imageSize, 0);
	status =  (status != TRUE	) ? ~ S_PASS : S_PASS;
	if(status == S_PASS){	
		return status;
	}
	/* boot form eMMC*/
	CONSOLEUtilsPrintf("boot form eMMC flash..........\r\n");
	status = SBLMmcsdCopy(pEntryPoint, imageSize, 1);
	status =  (status != TRUE	) ? ~ S_PASS : S_PASS;
	if(status == S_PASS){	
		return status;
	}

	return status;
}

/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */




/* -------------------------------------------------------------------------- */
/*                 Deprecated Function Declarations                           */
/* -------------------------------------------------------------------------- */
