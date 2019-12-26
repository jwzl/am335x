/**
 *  \file  asciiutils.c
 *
 * \brief  This file contain functions which compute ASCII for standard
 *         operations.
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

#include <string.h>
#include "types.h"
#include "edma.h"
#include "console_utils.h"
#include "mmcsd_lib.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Width of the data response buffer. */
#define DATA_RESPONSE_WIDTH       (SOC_CACHELINE_SIZE)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                            Global Variables Declarations                   */
/* ========================================================================== */

/* Cache size aligned data buffer (minimum of 64 bytes) for command response */
#ifdef __TMS470__
#pragma DATA_ALIGN(dataBuffer, SOC_CACHELINE_SIZE);
static uint8_t dataBuffer[DATA_RESPONSE_WIDTH];

#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = SOC_CACHELINE_SIZE
static uint8_t dataBuffer[DATA_RESPONSE_WIDTH];

#elif defined(gcc)
static uint8_t dataBuffer[DATA_RESPONSE_WIDTH]
                               __attribute__((aligned(SOC_CACHELINE_SIZE)));

#else
#error "Unsupported compiler\n\r"
#endif

/* frequency bases */
/* divided by 10 to be nice to platforms without floating point */
static const int fbase[] = {
	10000,
	100000,
	1000000,
	10000000,
};

/* Multiplier values for TRAN_SPEED.  Multiplied by 10 to be nice
 * to platforms without floating point.
 */
static const int multipliers[] = {
	0,	/* reserved */
	10,
	12,
	13,
	15,
	20,
	25,
	30,
	35,
	40,
	45,
	50,
	55,
	60,
	70,
	80,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static void udelay(uint32_t loopcnt)
{
	uint32_t i;

	for (i = 0; i<loopcnt; i++) {
		asm("   NOP");
	}
}

uint32_t MMCSDLibCmdSend(mmcsdLibCtrlInfo_t *pCtrl, mmcsdLibCmd_t *pCmd)
{
    return pCtrl->pfnCmdSend(pCtrl, pCmd);
}

uint32_t MMCSDLibAppCmdSend(mmcsdLibCtrlInfo_t *pCtrl, mmcsdLibCmd_t *pCmd)
{
    uint32_t status = FALSE;
    mmcsdLibCmd_t cmdApp;

    if(NULL != pCtrl)
    {
        status = TRUE;
    }

    if(TRUE == status)
    {
        /* APP cmd should be preceeded by a CMD55 */
        cmdApp.idx = MMCSD_LIB_CMD(55U);
        cmdApp.flags = 0U;
        cmdApp.arg = pCtrl->pCard->rca << 16U;
        status = MMCSDLibCmdSend(pCtrl, &cmdApp);
    }

    /* return safely, since we cannot continue if CMD55 fails */
    if (FALSE != status)
    {
        status = MMCSDLibCmdSend(pCtrl, pCmd);
    }

    return status;
}

uint32_t MMCSDLibBusWidthSet(mmcsdLibCtrlInfo_t *pCtrl)
{
    mmcsdLibCardInfo_t *pCard = pCtrl->pCard;
    uint32_t status = FALSE;
    mmcsdLibCmd_t cmdApp;

    if(NULL != pCtrl)
    {
        status = TRUE;
    }

    if(TRUE == status)
    {
        cmdApp.idx = MMCSD_LIB_CMD(6U);
        cmdApp.arg = MMCSD_LIB_BUS_WIDTH_1BIT;
        cmdApp.flags = 0U;

        if (pCtrl->busWidth & MMCSD_LIB_BUS_WIDTH_4BIT)
        {
            if (pCard->busWidth & MMCSD_LIB_BUS_WIDTH_4BIT)
            {
                cmdApp.arg = MMCSD_LIB_BUS_WIDTH_4BIT;
            }
        }

        cmdApp.arg = cmdApp.arg >> 1U;
        status = MMCSDLibAppCmdSend(pCtrl, &cmdApp);
    }

    if (TRUE == status)
    {
        if (0U == cmdApp.arg)
        {
            pCtrl->pfnBusWidthConfig(pCtrl, MMCSD_LIB_BUS_WIDTH_1BIT);
        }
        else
        {
            pCtrl->pfnBusWidthConfig(pCtrl, MMCSD_LIB_BUS_WIDTH_4BIT);
        }
    }

    return status;
}

uint32_t MMCSDLibTranSpeedSet(mmcsdLibCtrlInfo_t *pCtrl)
{
    mmcsdLibCardInfo_t *pCard = pCtrl->pCard;
    uint32_t speed;
    uint32_t status = FALSE;
    mmcsdLibCmd_t cmd;

    if(NULL != pCtrl)
    {
        status = TRUE;
    }

    if(TRUE == status)
    {
        pCtrl->pfnXferSetup(pCtrl, 1U, dataBuffer, 64U, 1U);

        cmd.idx = MMCSD_LIB_CMD(6U);
        cmd.arg = ((MMCSD_LIB_SWITCH_MODE & MMCSD_LIB_CMD6_GRP1_SEL) |
            (MMCSD_LIB_CMD6_GRP1_HS));
        cmd.flags = MMCSD_LIB_CMDRSP_READ | MMCSD_LIB_CMDRSP_DATA;
        cmd.nBlks = 1U;
        cmd.pData = (signed char*)dataBuffer;
        status = MMCSDLibCmdSend(pCtrl, &cmd);
    }

    if (FALSE != status)
    {
        status = pCtrl->pfnXferStatusGet(pCtrl);
    }

    if (FALSE != status)
    {
        /* Invalidate the data cache. */
#if 0
        CacheDataInvalidateBuff((uint32_t) dataBuffer, DATA_RESPONSE_WIDTH);
#endif

        speed = pCard->tranSpeed;

        if ((dataBuffer[16U] & 0xFU) == MMCSD_LIB_CMD6_GRP1_HS)
        {
            pCard->tranSpeed = MMCSD_LIB_TRANSPEED_50MBPS;
        }
        else
        {
            pCard->tranSpeed = MMCSD_LIB_TRANSPEED_25MBPS;
        }

        if (MMCSD_LIB_TRANSPEED_50MBPS == speed)
        {
            status = pCtrl->pfnBusFreqConfig(pCtrl, 50000000U);
            pCtrl->opClk = 50000000U;
        }
        else
        {
            status = pCtrl->pfnBusFreqConfig(pCtrl, 25000000U);
            pCtrl->opClk = 25000000U;
        }

        if(FALSE != status)
        {
            status = FALSE;
        }
    }

    return status;
}

uint32_t MMCSDLibCardReset(mmcsdLibCtrlInfo_t *pCtrl)
{
    uint32_t status = FALSE;
    mmcsdLibCmd_t cmd;

    if(NULL != pCtrl)
    {
        status = TRUE;
    }

    if(TRUE == status)
    {
        cmd.idx = MMCSD_LIB_CMD(0U);
        cmd.flags = MMCSD_LIB_CMDRSP_NONE;
        cmd.arg = 0U;
        status = MMCSDLibCmdSend(pCtrl, &cmd);
    }
	if(TRUE == status){
		udelay(2000);
	}
    return status;
}

uint32_t MMCSDLibStopCmdSend(mmcsdLibCtrlInfo_t *pCtrl)
{
    uint32_t status = FALSE;
    mmcsdLibCmd_t cmd;

    if(NULL != pCtrl)
    {
        status = TRUE;
    }

    if(TRUE == status)
    {
        cmd.idx  = MMCSD_LIB_CMD(12U);
        cmd.flags = MMCSD_LIB_CMDRSP_BUSY;
        cmd.arg = 0U;
        MMCSDLibCmdSend(pCtrl, &cmd);

        /* Get transfer status */
        status = pCtrl->pfnXferStatusGet(pCtrl);
    }

    return status;
}

static uint32_t mmc_send_if_cond(mmcsdLibCtrlInfo_t *pCtrl){
	uint32_t status = FALSE;
    mmcsdLibCmd_t cmd;

	if(NULL == pCtrl){
		return FALSE;
	}

	cmd.idx = MMCSD_LIB_CMD(8U);
    cmd.flags = 0;
	/* We set the bit if the host supports voltages between 2.7 and 3.6 V */
	cmd.arg = (MMCSD_LIB_CHECK_PATTERN | MMCSD_LIB_VOLT_2P7_3P6);
    status = MMCSDLibAppCmdSend(pCtrl, &cmd);
	if(status != TRUE){
		return status;
	}

	pCtrl->pCard->sdVer = SD_VERSION_2;
	return TRUE;
}

uint32_t MMCSDLibCardTypeCheck(mmcsdLibCtrlInfo_t *pCtrl)
{
	int timeout = 1000;
    uint32_t status = FALSE;
    mmcsdLibCmd_t cmd;

    if(NULL != pCtrl)
    {
        status = TRUE;
		/* Test for SD version 2 */
		mmc_send_if_cond(pCtrl);
    }

    if(TRUE == status)
    {
		do {
       		 /*
        	 * Card type can be found by sending CMD55. If the pCard responds,
        	 * it is a SD pCard. Else, we assume it is a MMC Card
        	 */
			cmd.idx = MMCSD_LIB_CMD(41U);
        	cmd.flags = 0;

			cmd.arg = MMCSD_LIB_OCR_HIGH_CAPACITY | MMCSD_LIB_OCR_VDD_WILDCARD;
			if(pCtrl->pCard->sdVer == SD_VERSION_2)
				cmd.arg |=0x40000000;
	
        	status = MMCSDLibAppCmdSend(pCtrl, &cmd);
			if(status != TRUE){
				return status;
			}	
			udelay(1000);

		} while ((!(cmd.rsp[0] & 0x80000000)) && timeout--);

		if (timeout <= 0)
			return FALSE;

		if(pCtrl->pCard->sdVer != SD_VERSION_2)
			pCtrl->pCard->sdVer = SD_VERSION_1_0;

		pCtrl->pCard->ocr = cmd.rsp[0U];
		pCtrl->pCard->rca = 0;
        pCtrl->pCard->highCap = (pCtrl->pCard->ocr & MMCSD_LIB_OCR_HIGH_CAPACITY) ? 1U : 0U;
		return TRUE;			
    }

    return status;
}

uint32_t MMCSDLibCtrlInit(mmcsdLibCtrlInfo_t *pCtrl)
{
    return pCtrl->pfnCtrlInit(pCtrl);
}

uint32_t MMCSDLibCardPresent(mmcsdLibCtrlInfo_t *pCtrl)
{
    return pCtrl->pfnCardPresent(pCtrl);
}

void MMCSDLibIntrEnable(mmcsdLibCtrlInfo_t *pCtrl)
{
    pCtrl->pfnIntrEnable(pCtrl);
}

static uint32_t mmc_send_op_cond(mmcsdLibCtrlInfo_t *pCtrl){
	uint32_t status = FALSE;
	int timeout = 10000;	
    mmcsdLibCmd_t cmd;

	if(NULL == pCtrl)
		return FALSE;
	
	/* CMD0 - reset pCard */
    MMCSDLibCardReset(pCtrl);

	/* CMD1 */
	cmd.idx = MMCSD_LIB_CMD(1U);
    cmd.flags = 0;
    cmd.arg = 0;
    status = MMCSDLibCmdSend(pCtrl, &cmd);
	if(status != TRUE)
		return status;

	udelay(1000);	

	do{
		cmd.idx = MMCSD_LIB_CMD(1U);
    	cmd.flags = 0;
    	//cmd.arg = MMCSD_LIB_OCR_HIGH_CAPACITY | MMCSD_LIB_OCR_VDD_WILDCARD;
		cmd.arg = (pCtrl->voltages & (cmd.rsp[0] & 0x007FFF80))|(cmd.rsp[0] & 0x60000000);

		if (pCtrl->host_caps & MMC_MODE_HC)
			cmd.arg |= 0x40000000;
		
    	status = MMCSDLibCmdSend(pCtrl, &cmd);
		if(status != TRUE)
			return status;	

		udelay(1000);
	}while(!(cmd.rsp[0] & 0x80000000) && timeout--);

	if (timeout <= 0)
		return FALSE;

	pCtrl->pCard->sdVer = MMC_VERSION_UNKNOWN;
	pCtrl->ocr = cmd.rsp[0];
	pCtrl->pCard->highCap =  ((pCtrl->ocr & 0x40000000) == 0x40000000);
	pCtrl->pCard->rca = 0;

	return TRUE;
}

static uint32_t mmc_send_status(mmcsdLibCtrlInfo_t *pCtrl)
{
	int timeout = 1000;
	uint32_t status = FALSE;
	uint32_t retries = 5;
    mmcsdLibCmd_t cmd;

	cmd.idx = MMCSD_LIB_CMD(13U);
    cmd.flags = 0;
    cmd.arg = pCtrl->pCard->rca << 16;

	do {
		status = MMCSDLibCmdSend(pCtrl, &cmd);
		if(status == TRUE){
			if ((cmd.rsp[0] & (1 << 8)) &&
			    (cmd.rsp[0] & (0xf << 9)) !=
			     (7 << 9))
				break;
			else if (cmd.rsp[0] & (~0x0206BF7F)) {
				return FALSE;
			}
		}else if (--retries < 0)
			return FALSE;

		udelay(1000);
		
	}while(timeout--);

	return TRUE;
}

static int sd_switch(mmcsdLibCtrlInfo_t *pCtrl, int mode, int group, unsigned char value)
{
	//mmcsdLibCardInfo_t *pCard = pCtrl->pCard;
	//uint32_t status = FALSE;
	mmcsdLibCmd_t cmd;

	cmd.idx = MMCSD_LIB_CMD(6U);
    cmd.arg = (mode << 31) | 0xffffff;
	cmd.arg &= ~(0xf << (group * 4));
	cmd.arg |= value << (group * 4);
    cmd.flags =  MMCSD_LIB_CMDRSP_READ | MMCSD_LIB_CMDRSP_DATA;
	
    cmd.nBlks = 1U;
    cmd.pData = (signed char*)dataBuffer;
    return MMCSDLibCmdSend(pCtrl, &cmd);
}

static uint32_t mmc_change_freq(mmcsdLibCtrlInfo_t *pCtrl){
	mmcsdLibCardInfo_t *pCard = pCtrl->pCard;
	uint32_t status = FALSE;
	mmcsdLibCmd_t cmd;
	char cardtype;	
	//int timeout;

	pCard->card_caps = 0;
	/* Only version 4 supports high-speed */
	if (pCard->sdVer < MMC_VERSION_4)
		return TRUE;

	pCard->card_caps = (MMCSD_LIB_BUS_WIDTH_4BIT |MMCSD_LIB_BUS_WIDTH_8BIT);
	pCtrl->pfnXferSetup(pCtrl, 1U, dataBuffer, 512U, 1U);
		
	cmd.idx = MMCSD_LIB_CMD(8U);
	cmd.flags = MMCSD_LIB_CMDRSP_READ | MMCSD_LIB_CMDRSP_DATA;
	cmd.arg = 0;
	cmd.nBlks = 1U;
	cmd.pData = (signed char*)dataBuffer;
	
	status = MMCSDLibCmdSend(pCtrl,&cmd);
	if(status != TRUE)
		return status;
		
	status = pCtrl->pfnXferStatusGet(pCtrl);
	if(status != TRUE)
		return status;

	cardtype = dataBuffer[196U] & 0x3f;
	if(cardtype & (1 << 0))
		pCard->card_caps |= MMC_MODE_HS;

	if(cardtype & (1 << 1))
		pCard->card_caps |= MMC_MODE_HS_52MHz;

	if(cardtype & (1 << 2))
		pCard->card_caps |= MMC_MODE_DDR_52MHz;
	
	if(cardtype & (1 << 4))
		pCard->card_caps |= MMC_MODE_DDR_52MHz;

	/* MMC Switch bus frequency*/
	if(pCard->card_caps & MMC_MODE_HS){
		cmd.idx = MMCSD_LIB_CMD(6U);
		cmd.flags = 0;
		cmd.arg = (0x03 << 24) | (185 << 16) | (1 << 8);

		status = MMCSDLibCmdSend(pCtrl,&cmd);
		if(status != TRUE)
			return status;

		status = mmc_send_status(pCtrl);	
		if(status != TRUE)
			return status;
		
		/* Now check to see that it worked */
		pCtrl->pfnXferSetup(pCtrl, 1U, dataBuffer, 512U, 1U);
		
		cmd.idx = MMCSD_LIB_CMD(8U);
		cmd.flags = MMCSD_LIB_CMDRSP_READ | MMCSD_LIB_CMDRSP_DATA;
		cmd.arg = 0;
		cmd.nBlks = 1U;
		cmd.pData = (signed char*)dataBuffer;
	
		status = MMCSDLibCmdSend(pCtrl,&cmd);
		if(status != TRUE)
			return status;
		
		status = pCtrl->pfnXferStatusGet(pCtrl);
		if(status != TRUE)
			return status;
		
		/* No high-speed support */
		if(!dataBuffer[185U])
			return TRUE;
		
	}else if(pCard->card_caps & MMC_MODE_HS200){
		CONSOLEUtilsPrintf("Currently not support\r\n");
	}

	return status;	
}

static uint32_t sd_change_freq(mmcsdLibCtrlInfo_t *pCtrl){
	mmcsdLibCardInfo_t *pCard = pCtrl->pCard;
	uint32_t status = FALSE;
	uint32_t temp = 0;
	mmcsdLibCmd_t cmd;
	int timeout;
		
	cmd.idx = MMCSD_LIB_CMD(55U);
	cmd.flags = 0;
	cmd.arg = pCard->rca << 16U;

	status = MMCSDLibCmdSend(pCtrl,&cmd);
	if(status != TRUE)
		return status;

	/* Read the SCR to find out if this card supports higher speeds */
	cmd.idx = MMCSD_LIB_CMD(51U);
	cmd.flags = MMCSD_LIB_CMDRSP_READ | MMCSD_LIB_CMDRSP_DATA;
	cmd.arg = pCard->rca << 16U;

	timeout = 3;
retry_scr:
	pCtrl->pfnXferSetup(pCtrl, 1U, dataBuffer, 8U, 1U);
	cmd.nBlks = 1U;
	cmd.pData = (signed char*)dataBuffer;

	
	status = MMCSDLibCmdSend(pCtrl, &cmd);
	if (status != TRUE) {
		if (timeout--)
			goto retry_scr;

		return status;
	}

	status = pCtrl->pfnXferStatusGet(pCtrl);
	if(status != TRUE)
		return status;

	pCard->raw_scr[0U] = (dataBuffer[3U] << 24U) | (dataBuffer[2U] << 16U) | \
                               (dataBuffer[1U] << 8U) | (dataBuffer[0U]);
	pCard->raw_scr[1U] = (dataBuffer[7U] << 24U) | (dataBuffer[6U] << 16U) | \
                               (dataBuffer[5U] << 8U) | (dataBuffer[4U]);

	switch(MMCSD_LIB_CARD_VERSION(pCard)){
	case 0:
		pCard->sdVer= SD_VERSION_1_0;
		CONSOLEUtilsPrintf("SD version 1.0\r\n");
		break;
	case 1:
		pCard->sdVer = SD_VERSION_1_10;
		CONSOLEUtilsPrintf("SD version 1.10\r\n");
		break;
	case 2:
		pCard->sdVer = SD_VERSION_2;
		CONSOLEUtilsPrintf("SD version 2.0\r\n");
		break;
	default:
		pCard->sdVer = SD_VERSION_1_0;
		CONSOLEUtilsPrintf("SD version 1.0\r\n");
		break;
	} 	

	if(pCard->raw_scr[0U] & 0x00040000){
		CONSOLEUtilsPrintf("4 bit width.\r\n");
		pCard->busWidth = MMCSD_LIB_CARD_BUSWIDTH(pCard);
	}
	
	/* Version 1.0 doesn't support switching */
	if (pCard->sdVer == SD_VERSION_1_0)
		return TRUE;

	timeout = 4;
	while (timeout--) {
		pCtrl->pfnXferSetup(pCtrl, 1U, dataBuffer, 64U, 1U);
		//SWITCH Check
		status = sd_switch(pCtrl, 0, 0, 1);
		if(status != TRUE)
			return status;
		
		status = pCtrl->pfnXferStatusGet(pCtrl);
		if(status != TRUE)
			return status;

		temp = (dataBuffer[31U] << 24U) | (dataBuffer[30U] << 16U) | \
                               (dataBuffer[29U] << 8U) | (dataBuffer[28U]);
		/* The high-speed function is busy.  Try again */
		if(!(temp & 0x00020000))
			break;
	}

	/* If high-speed isn't supported, we return */
	temp = (dataBuffer[15U] << 24U) | (dataBuffer[14U] << 16U) | \
                               (dataBuffer[13U] << 8U) | (dataBuffer[12U]);	
	if(!(temp & 0x00020000))
		return TRUE;
	
	pCtrl->pfnXferSetup(pCtrl, 1U, dataBuffer, 64U, 1U);
	status = sd_switch(pCtrl, 1, 0, 1);
	if(status != TRUE)
		return status;

	status = pCtrl->pfnXferStatusGet(pCtrl);
	if(status != TRUE)
		return status;

	return status;
}

static uint32_t mmc_select_bus_width(mmcsdLibCtrlInfo_t *pCtrl){
	mmcsdLibCardInfo_t *pCard = pCtrl->pCard;
	mmcsdLibCmd_t cmd;
	uint32_t status = FALSE;
	uint8_t ext_csd[512]={0};
	// In HS mode.
	int i =2;

	pCtrl->pfnXferSetup(pCtrl, 1U, dataBuffer, 512U, 1U);
	cmd.idx = MMCSD_LIB_CMD(8U);
	cmd.flags = MMCSD_LIB_CMDRSP_READ | MMCSD_LIB_CMDRSP_DATA;
	cmd.arg = 0;
	cmd.nBlks = 1U;
	cmd.pData = (signed char*)dataBuffer;

	status = MMCSDLibCmdSend(pCtrl, &cmd);
	if(status != TRUE)
		return status;
		
	status = pCtrl->pfnXferStatusGet(pCtrl);
	if(status != TRUE)
		return status;
	
	memcpy(ext_csd, dataBuffer, 512);
		
	/* MMC Switch bus width*/
	for(i = 2; i>= 0 ; i--){ 
		cmd.idx = MMCSD_LIB_CMD(6U);
		cmd.flags = 0;
		cmd.arg = (0x03 << 24) | (183 << 16) | (i << 8);

		status = MMCSDLibCmdSend(pCtrl,&cmd);
		if(status != TRUE)
			continue;

		status = mmc_send_status(pCtrl);	
		if(status != TRUE)
			continue;
		
		if(i == 2){
				//CONSOLEUtilsPrintf("Set 8 bit bus\r\n");
				pCtrl->pfnBusWidthConfig(pCtrl, MMCSD_LIB_BUS_WIDTH_8BIT);
				pCard->busWidth = MMCSD_LIB_BUS_WIDTH_8BIT;
		}else if (i == 1){
				//CONSOLEUtilsPrintf("Set 4 bit bus\r\n");
				pCtrl->pfnBusWidthConfig(pCtrl, MMCSD_LIB_BUS_WIDTH_4BIT);
				pCard->busWidth = MMCSD_LIB_BUS_WIDTH_4BIT;
		}else{
				//CONSOLEUtilsPrintf("Set 1 bit bus\r\n");
				pCtrl->pfnBusWidthConfig(pCtrl, MMCSD_LIB_BUS_WIDTH_1BIT);
				pCard->busWidth = MMCSD_LIB_BUS_WIDTH_1BIT;
		}

		//Check if it work
		pCtrl->pfnXferSetup(pCtrl, 1U, dataBuffer, 512U, 1U);
		cmd.idx = MMCSD_LIB_CMD(8U);
		cmd.flags = MMCSD_LIB_CMDRSP_READ | MMCSD_LIB_CMDRSP_DATA;
		cmd.arg = 0;
		cmd.nBlks = 1U;
		cmd.pData = (signed char*)dataBuffer;

		status = MMCSDLibCmdSend(pCtrl, &cmd);
		if(status != TRUE)
			return status;
		
		status = pCtrl->pfnXferStatusGet(pCtrl);
		if(status != TRUE)
			return status;

		/* Only compare read only fields */
		if(ext_csd[160U] == dataBuffer[160U] &&
			ext_csd[221U] == dataBuffer[221U] &&
			ext_csd[192U] == dataBuffer[192U] &&
			ext_csd[224U] == dataBuffer[224U] &&
			memcmp(&ext_csd[212U], &dataBuffer[212U], 4) == 0){
			break;
		}
	}
	
	return status;
}
static uint32_t mmc_startup(mmcsdLibCtrlInfo_t *pCtrl){
	mmcsdLibCardInfo_t *pCard = pCtrl->pCard;
	uint32_t status = FALSE;
	uint32_t mult, freq;
	uint32_t cmult, csize;
	mmcsdLibCmd_t cmd;

	/* Send CMD2, to get the pCard identification register */
	cmd.idx = MMCSD_LIB_CMD(2U);
	cmd.flags = MMCSD_LIB_CMDRSP_136BITS;
	cmd.arg = 0U;

	status = MMCSDLibCmdSend(pCtrl, &cmd);
	if(status != TRUE)
		return status;
	
	memcpy(pCard->raw_cid, cmd.rsp, 16U);
	/*
	 * For MMC cards, set the Relative Address.
	 * For SD cards, get the Relatvie Address.
	 * This also puts the cards into Standby State
	 */
	cmd.idx = MMCSD_LIB_CMD(3U);
	cmd.flags = 0U;
	cmd.arg = 0U;

	status = MMCSDLibCmdSend(pCtrl,&cmd);
	if(status != TRUE)
		return status;
	
	if(pCard->cardType == MMCSD_LIB_CARD_SD)
		pCard->rca = MMCSD_LIB_RCA_ADDR(cmd.rsp[0U]);
	
	
	/* Send CMD9, to get the pCard specific data */
   	if(pCard->cardType != MMCSD_LIB_CARD_MMC){
		cmd.idx = MMCSD_LIB_CMD(9U);
    	cmd.flags = MMCSD_LIB_CMDRSP_136BITS;
    	cmd.arg = pCard->rca << 16U;

    	status = MMCSDLibCmdSend(pCtrl, &cmd);
		if(status != TRUE)
			return status;
	
		memcpy(pCard->raw_csd, cmd.rsp, 16U);
		if (MMCSD_LIB_CARD_CSD_VERSION(pCard)){
			pCard->tranSpeed = MMCSD_LIB_CARD1_TRANSPEED(pCard);
			pCard->blkLen = 1U << (MMCSD_LIB_CARD1_RDBLKLEN(pCard));
			pCard->size = MMCSD_LIB_CARD1_SIZE(pCard);
			pCard->nBlks = pCard->size / pCard->blkLen;
		}else{
			pCard->tranSpeed = MMCSD_LIB_CARD0_TRANSPEED(pCard);
			pCard->blkLen = 1U << (MMCSD_LIB_CARD0_RDBLKLEN(pCard));
			pCard->nBlks = MMCSD_LIB_CARD0_NUMBLK(pCard);
			pCard->size = MMCSD_LIB_CARD0_SIZE(pCard);
		}
	}else {
		/* Waiting for the ready status */
		cmd.idx = MMCSD_LIB_CMD(9U);
    	cmd.flags = MMCSD_LIB_CMDRSP_136BITS;
    	cmd.arg = pCard->rca << 16U;

    	status = MMCSDLibCmdSend(pCtrl, &cmd);
		if(status != TRUE)
			return status;
	
		mmc_send_status(pCtrl);
		//memcpy(pCard->raw_csd, cmd.rsp, 16U);
		pCard->raw_csd[0] = cmd.rsp[3];
		pCard->raw_csd[1] = cmd.rsp[2];
		pCard->raw_csd[2] = cmd.rsp[1];
		pCard->raw_csd[3] = cmd.rsp[0];

		if(pCard->cardType == MMCSD_LIB_CARD_MMC){
			int version = (pCard->raw_csd[0] >> 26) & 0xf;

			switch (version) {
			case 0:
				pCard->sdVer= MMC_VERSION_1_2;
				break;
			case 1:
				pCard->sdVer = MMC_VERSION_1_4;
				break;
			case 2:
				pCard->sdVer = MMC_VERSION_2_2;
				break;
			case 3:
				pCard->sdVer = MMC_VERSION_3;
				break;
			case 4:
				pCard->sdVer = MMC_VERSION_4;
				break;
			default:
				pCard->sdVer = MMC_VERSION_1_2;
				break;
			}
		}

		freq = fbase[(pCard->raw_csd[0] & 0x7)];
		mult = multipliers[((pCard->raw_csd[0] >> 3) & 0xf)];
		pCard->tranSpeed = freq * mult;
		pCard->blkLen = 1U << ((pCard->raw_csd[1] >> 16) & 0xf);

		pCard->size = MMCSD_LIB_CARD1_SIZE(pCard);
	}

	if (pCard->highCap){
		csize = (pCard->raw_csd[1] & 0x3f) << 16
			| (pCard->raw_csd[2] & 0xffff0000) >> 16;
		cmult = 8;
	}else {
		csize = (pCard->raw_csd[1] & 0x3ff) << 2
			| (pCard->raw_csd[2] & 0xc0000000) >> 30;
		cmult = (pCard->raw_csd[2] & 0x00038000) >> 15;

		pCard->nBlks = MMCSD_LIB_CARD0_NUMBLK(pCard);

		/* Set data block length to 512 (for byte addressing pCards) */	
		cmd.idx = MMCSD_LIB_CMD(16U);
        cmd.flags = MMCSD_LIB_CMDRSP_NONE;
        cmd.arg = 512U;
        status = MMCSDLibCmdSend(pCtrl,&cmd);
		if(status != TRUE)
			return status;

		pCard->blkLen = 512;
	}
	
	pCard->size = (csize + 1) << (cmult + 2);
	pCard->size *=pCard->blkLen;

	if(pCard->blkLen > 512)
		pCard->blkLen = 512;

	pCard->nBlks = pCard->size / pCard->blkLen;

	/* Select the pCard */
	cmd.idx = MMCSD_LIB_CMD(7U);
	cmd.flags = MMCSD_LIB_CMDRSP_BUSY;
	cmd.arg = pCard->rca << 16U;

	status = MMCSDLibCmdSend(pCtrl,&cmd);
	if(status != TRUE)
		return status;

	// for MMC card
	if((pCard->cardType == MMCSD_LIB_CARD_MMC) && 
					(pCard->sdVer >= MMC_VERSION_4)){
		pCtrl->pfnXferSetup(pCtrl, 1U, dataBuffer, 512U, 1U);
		cmd.idx = MMCSD_LIB_CMD(8U);
		cmd.flags = MMCSD_LIB_CMDRSP_READ | MMCSD_LIB_CMDRSP_DATA;
		cmd.arg = 0;
		cmd.nBlks = 1U;
		cmd.pData = (signed char*)dataBuffer;

		status = MMCSDLibCmdSend(pCtrl, &cmd);
		if(status == TRUE){
			status = pCtrl->pfnXferStatusGet(pCtrl);
			if(status != TRUE)
				return status;
	
			if(dataBuffer[192U] >= 2){
				uint32_t capacity;
				capacity = dataBuffer[212U]<< 0 |
								dataBuffer[213U]<< 8 |
								dataBuffer[214U]<< 16 |
								dataBuffer[215U]<< 24;

				capacity *= 512;
				if ((capacity >> 20) > 2 * 1024)
					pCard->size =  capacity;    
			}

			switch(dataBuffer[192U]){
			case 1:
				pCard->sdVer = MMC_VERSION_4_1;
				CONSOLEUtilsPrintf("MMC version 4.1\r\n");
				break;
			case 2:
				pCard->sdVer = MMC_VERSION_4_2;
				CONSOLEUtilsPrintf("MMC version 4.2\r\n");
				break;
			case 3:
				pCard->sdVer = MMC_VERSION_4_3;
				CONSOLEUtilsPrintf("MMC version 4.3\r\n");
				break;
			case 5:
				pCard->sdVer = MMC_VERSION_4_41;
				CONSOLEUtilsPrintf("MMC version 4.4.1\r\n");
				break;
			case 6:
				pCard->sdVer = MMC_VERSION_4_5;
				CONSOLEUtilsPrintf("MMC version 4.5\r\n");
				break;	
			case 7:
				pCard->sdVer = MMC_VERSION_5_0;
				CONSOLEUtilsPrintf("MMC version 5.0\r\n");
				break;		  
			} 
		}

	}
	
	if(pCard->cardType != MMCSD_LIB_CARD_MMC)
		status = sd_change_freq(pCtrl);
	else
		status = mmc_change_freq(pCtrl);
	
	if(status != TRUE)
		return status;

	if(pCard->cardType == MMCSD_LIB_CARD_SD){
		/* SD*/
		if (pCtrl->busWidth & MMCSD_LIB_BUS_WIDTH_4BIT){
            if (pCard->busWidth & MMCSD_LIB_BUS_WIDTH_4BIT){
				//CONSOLEUtilsPrintf("seitch to 4 bit\r\n");
				cmd.idx = MMCSD_LIB_CMD(6U);
        		cmd.arg = 2;
        		cmd.flags = 0U;
        		status = MMCSDLibAppCmdSend(pCtrl, &cmd);
				if(status != TRUE)
					return status;
				
				pCtrl->pfnBusWidthConfig(pCtrl, MMCSD_LIB_BUS_WIDTH_4BIT);
			}
		}
	
		/* Set speed */
		if (MMCSD_LIB_TRANSPEED_50MBPS == pCard->tranSpeed){
			//CONSOLEUtilsPrintf("Switch to 50MHz\r\n");
    		status = pCtrl->pfnBusFreqConfig(pCtrl, 50000000U);
    		pCtrl->opClk = 50000000U;
		}else{
			//CONSOLEUtilsPrintf("Switch to 25MHz\r\n");
			status = pCtrl->pfnBusFreqConfig(pCtrl, 25000000U);
			pCtrl->opClk = 25000000U;
			pCard->tranSpeed = MMCSD_LIB_TRANSPEED_25MBPS;
		}
	}else{
		/* MMC*/
		if(pCard->sdVer >= MMC_VERSION_4){
			if(pCard->card_caps & MMC_MODE_HS200){
				return TRUE;
			}
			
			/* High Speed is set, there are two types: 52MHz and 26MHz */
			if (pCard->card_caps & MMC_MODE_HS){
				if(pCard->card_caps & MMC_MODE_HS_52MHz){
					/* 52MHz*/
					//CONSOLEUtilsPrintf("Switch to 52MHz\r\n");
					status = pCtrl->pfnBusFreqConfig(pCtrl, 52000000U);//52000000U);
					pCtrl->opClk = 52000000U;
					pCard->tranSpeed = 0x5AU;
				}else{
					/* 26MHz*/
					//CONSOLEUtilsPrintf("Switch to 26MHz\r\n");
					status = pCtrl->pfnBusFreqConfig(pCtrl, 26000000U);
					pCtrl->opClk = 26000000U;
					pCard->tranSpeed = 0x32U;
				}
			}	

			status = mmc_select_bus_width(pCtrl);
			if(status != TRUE)
				return status;
		}
	}
	
	return status;
}

uint32_t MMCSDLibCardInit(mmcsdLibCtrlInfo_t *pCtrl)
{
	mmcsdLibCardInfo_t *pCard = pCtrl->pCard;
   // uint32_t retry = 0xFFFFU;
	uint32_t status = FALSE;
   // mmcsdLibCmd_t cmd;

	if(NULL != pCtrl){
		status = TRUE;
	}

	if(TRUE == status){
		memset(pCtrl->pCard, 0U, sizeof(mmcsdLibCardInfo_t));
		pCard->pCtrl = pCtrl;
		/* CMD0 - reset pCard */
		status = MMCSDLibCardReset(pCtrl);
	}

	if(FALSE != status){
		/* Returns TRUE for a SD pCard, FALSE for a non-SD pCard */
		status = MMCSDLibCardTypeCheck(pCtrl);
	}

    /* SD Card */
	if(FALSE != status){
		pCtrl->pCard->cardType = MMCSD_LIB_CARD_SD;
		CONSOLEUtilsPrintf("SDCard\r\n");

	}else{
		/* Detect the mmc device*/
		status = mmc_send_op_cond(pCtrl);
		if(status != TRUE){
			/*Card did not respond to voltage select! */
			CONSOLEUtilsPrintf("Card did not respond to voltage select\r\n");
			return FALSE;	/* Unusable Card */
		}
		CONSOLEUtilsPrintf("eMMC Card\r\n");
		pCtrl->pCard->cardType = MMCSD_LIB_CARD_MMC;
	}

	
	return mmc_startup(pCtrl);
}

uint32_t MMCSDLibWriteCmdSend(mmcsdLibCtrlInfo_t *pCtrl, void *pBuff, uint32_t block,
                               uint32_t nBlks)
{
    mmcsdLibCardInfo_t *pCard = pCtrl->pCard;
    uint32_t status = FALSE;
    uint32_t address;
    mmcsdLibCmd_t cmd;

    if(NULL != pCtrl)
    {
        status = TRUE;
    }

    if(TRUE == status)
    {
        /*
         * Address is in blks for high cap pCards and in actual bytes
         * for standard capacity pCards
         */
        if (FALSE != pCard->highCap)
        {
            address = block;
        }
        else
        {
            address = block * pCard->blkLen;
        }

        /* Clean the data cache. */
#if 0
        CacheDataCleanBuff((uint32_t) pBuff, (512U * nBlks));
#endif
        pCtrl->pfnXferSetup(pCtrl, 0U, pBuff, 512U, nBlks);

        cmd.flags = MMCSD_LIB_CMDRSP_WRITE | MMCSD_LIB_CMDRSP_DATA;
        cmd.arg = address;
        cmd.nBlks = nBlks;

        if (nBlks > 1U)
        {
            cmd.idx = MMCSD_LIB_CMD(25U);
            cmd.flags |= MMCSD_LIB_CMDRSP_ABORT;
        }
        else
        {
            cmd.idx = MMCSD_LIB_CMD(24U);
        }

        status = MMCSDLibCmdSend(pCtrl, &cmd);
    }

    if(FALSE != status)
    {
        status = pCtrl->pfnXferStatusGet(pCtrl);
    }

    if(FALSE != status)
    {
        /* Send a STOP */
        if (cmd.nBlks > 1U)
        {
            status = MMCSDLibStopCmdSend(pCtrl);
        }
    }

    return status;
}

uint32_t MMCSDLibReadCmdSend(mmcsdLibCtrlInfo_t *pCtrl, void *pBuff, uint32_t block,
                              uint32_t nBlks)
{
    mmcsdLibCardInfo_t *pCard = pCtrl->pCard;
    uint32_t status = FALSE;
    uint32_t address;
    mmcsdLibCmd_t cmd;

    if(NULL != pCtrl)
    {
        status = TRUE;
    }

    if(TRUE == status)
    {
        /*
         * Address is in blks for high cap pCards and in actual bytes
         * for standard capacity pCards
         */
        if (FALSE != pCard->highCap)
        {
            address = block;
        }
        else
        {
            address = block * pCard->blkLen;
        }

        pCtrl->pfnXferSetup(pCtrl, 1U, pBuff, 512U, nBlks);

        cmd.flags = MMCSD_LIB_CMDRSP_READ | MMCSD_LIB_CMDRSP_DATA;
        cmd.arg = address;
        cmd.nBlks = nBlks;

        if (nBlks > 1U)
        {
            cmd.flags |= MMCSD_LIB_CMDRSP_ABORT;
            cmd.idx = MMCSD_LIB_CMD(18U);
        }
        else
        {
            cmd.idx = MMCSD_LIB_CMD(17U);
        }

        status = MMCSDLibCmdSend(pCtrl, &cmd);
    }

    if(FALSE != status)
    {
        status = pCtrl->pfnXferStatusGet(pCtrl);
    }

    if(FALSE != status)
    {
        /* Send a STOP */
        if (cmd.nBlks > 1U)
        {
            status = MMCSDLibStopCmdSend(pCtrl);
        }
    }

    if(FALSE != status)
    {
        /* Invalidate the data cache. */
#if 0
        CacheDataInvalidateBuff((uint32_t) pBuff, (512U * nBlks));
#endif
    }

    return status;
}

/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/*                 Deprecated Function Declarations                           */
/* -------------------------------------------------------------------------- */
