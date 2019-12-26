/**
 *  \file  console_utils.c
 *
 *  \brief  This file contains generic wrapper functions for console utilities
 *          which allow user to configure the console type and redirect all the
 *          console Input/Output to the console type selected.
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

#include <stdio.h>
#include <error.h>
#include <types.h>
#include <misc.h>
#include <console_utils.h>
#include "console_utils_uart.h"

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

int32_t CONSOLEUtilsInit(void)
{
    /*
     * Initializes the UART with appropriate Baud rate and Line
     * characteristics for use as console.
     */
    return CONSOLEUtilsUartInit();
}

int32_t CONSOLEUtilsGetc(void)
{
    int32_t retChar = '\0';

    retChar = CONSOLEUtilsUartGetChar();

    return retChar;
}

int32_t CONSOLEUtilsPutc(int32_t character)
{
    int32_t retChar = '\0';
  
    retChar = CONSOLEUtilsUartPutChar(character);
  
    return retChar;
}

uint8_t *CONSOLEUtilsGets(uint8_t *pRxBuffer, int32_t size)
{
    uint8_t *pBuf = (void *)NULL;

    if(NULL != pRxBuffer)
    {
            pBuf = CONSOLEUtilsUartGetStr(pRxBuffer, size);
    }

    return pBuf;
}

int32_t CONSOLEUtilsPuts(const char *pString)
{
    int32_t retStatus = E_FAIL;

    retStatus = CONSOLEUtilsUartPutStr(pString);

    return retStatus;
}

#ifndef DDRLESS
int32_t CONSOLEUtilsPrintf(const char *pFormat, ...)
{
    va_list arg;
    int32_t retStatus = E_FAIL;

    /* Start the variable arguments processing. */
    va_start (arg, pFormat);
    retStatus = CONSOLEUtilsUartPrintFmt(pFormat, arg);
    /* End the variable arguments processing. */
    va_end(arg);
    return retStatus;
}
#endif

int32_t CONSOLEUtilsScanf(const char *pFormat, ...)
{
    va_list arg;
    int32_t retStatus = E_FAIL;

        /* Start the variable argument processing. */
        va_start(arg, pFormat);

        
         retStatus = CONSOLEUtilsUartScanFmt(pFormat, arg);
     

        /* End the variable argument processing. */
        va_end(arg);

    return retStatus;
}

int32_t CONSOLEUtilsGetNonBlockingInput(consoleUtilsBuf_t *pRxConsoleBuf,
                                        void *pVar,
                                        uint32_t dataType,
                                        uint32_t *pStatus)
{
    int32_t retStatus = E_FAIL;

    retStatus = CONSOLEUtilsUartGetNonBlockingInput(pRxConsoleBuf, pVar, dataType, pStatus);

    return retStatus;
}

/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/*                 Deprecated Function Definitions                            */
/* -------------------------------------------------------------------------- */

void ConsoleUtilsInit(void)
{
    /*
     * Initializes the UART with appropriate Baud rate and Line
     * characteristics for use as console.
     */
    CONSOLEUtilsUartInit();
}

void ConsoleUtilsPrintf(const char *string, ...)
{
    va_list arg;

    /* Start the variable arguments processing. */
    va_start (arg, string);

    CONSOLEUtilsUartPrintFmt(string, arg);
    /* End the variable arguments processing. */
    va_end(arg);
}

int32_t ConsoleUtilsScanf(const char *format, ...)
{
    va_list arg;
    int32_t inputStatus = -1;

    /* Start the variable argument processing. */
    va_start(arg, format);

    inputStatus = CONSOLEUtilsUartScanFmt(format, arg);
 
    /* End the variable argument processing. */
    va_end(arg);
    return inputStatus;
}

char* ConsoleUtilsGets(char *rxBuffer, int32_t size)
{
    char *pBuf = (void *)0;

     pBuf = (char *)CONSOLEUtilsUartGetStr((uint8_t *)rxBuffer, size);


    return pBuf;
}

void ConsoleUtilsPuts(char *string)
{
     CONSOLEUtilsUartPutStr(string);
}

uint8_t ConsoleUtilsGetChar(void)
{
    uint8_t byte = 0U;

    
    byte = CONSOLEUtilsUartGetChar();

    return byte;
}

void ConsoleUtilsPutChar(uint8_t byte)
{
	CONSOLEUtilsUartPutChar(byte);
}
