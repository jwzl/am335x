/*
 * Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 *
 *  \brief  Template application tasks file:
 *          This template application exercises multiple tasks and
 *          peripherals. The different task functions are run under
 *          separate Tasks in TI BIOS.
 *          The appTasksCreate function creates the different tasks.
 *          More tasks can be added in this function as required.
 */

/* Standard header files */
#include <string.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* Local template app header file */
#include "app.h"

/**********************************************************************
 ************************** Function prototypes ***********************
 **********************************************************************/
void biosTaskCreate(ti_sysbios_knl_Task_FuncPtr taskFunctionPtr,
                    char *taskName, int taskPriority, int stackSize);

/* Task functions */
void gpio_toggle_led_task(UArg arg0, UArg arg1);
void uart_task(UArg arg0, UArg arg1);
void spi_test_task(UArg arg0, UArg arg1);
void i2c_eeprom_read_and_display_task(UArg arg0, UArg arg1);
/* Add any additional task function prototypes here */

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
volatile uint32_t g_endTestTriggered = 0;

/**
 *  @brief Function appTasksCreate : Creates multiple application tasks
 *
 *  @param[in]         None
 *  @retval            none
 */
void appTasksCreate(void)
{

    /* Create multiple tasks with different task priority & stack size */
    appPrint("\n ======== Starting to create application tasks ========\n");

    /* Create task to exercise uart */
    biosTaskCreate(uart_task,
                   "uart_task",
                   8, 4096);

    /* Create task to test spi interface */
    biosTaskCreate(spi_test_task,
                   "spi_test_task",
                   6, 4096);

    /* Create task to test spi interface */
    biosTaskCreate(i2c_eeprom_read_and_display_task,
                   "i2c_eeprom_read_and_display_task",
                   7, 4096);

    /* Add additional tasks here */

    appPrint("\n ======== Application tasks created successfully ========\n");

}

/**
 *  @brief Function uart_task : This task scans UART port and prints
 *      back word entered. On "ESC" triggers end of test.
 *      Exercises reads and writes to UART port
 *
 *  @param[in]         arg0, arg1: Arguments ( Currently not used)
 *  @retval            none
 */
void uart_task(UArg arg0, UArg arg1)
{

    char buffPointer[1000];
    char echoPrompt[] = "\n uart_task :Enter a word or Esc to quit >";
    char echoPrompt1[] = "Data received is:";

    appPrint("\n uart_task task started\n");
    
    /* Wait for other tasks to settle */
    Task_sleep(100);

    memset(buffPointer, 0, sizeof(buffPointer));
    while (1) {
        /* Print prompt to UART port */
        UART_printf(echoPrompt);

        /* Scan input word from user */
        if (UART_scanFmt("%s", &buffPointer) != S_PASS)
        {
            appPrint("\n ERROR: UART_scanFmt failed");
            goto UART_TASK_EXIT;
        }

        /* Check on word entered here */
        /* If needed a command parser can be added here */
        switch(buffPointer[0]) {
        case '\0': /*Ignore empty string */
            break;

        case 27: /* Exit on ESC character */
            goto UART_TASK_EXIT;

        default:
            /* Display prompt  */
            UART_printf(echoPrompt1);
            /* Display received word */
            UART_printf(buffPointer);
            /* Print new line */
            UART_printf("\n");
            break;
        }

        /* Sleep to yield */
        Task_sleep(10);
    }

UART_TASK_EXIT:
    appPrint("\n uart_task task ended");
    /* Trigger end test to other tasks */
    g_endTestTriggered = 1;
    /* Wait a while */
    Task_sleep(100);
    /* Now exit application */
    BIOS_exit(0);
    Task_exit();
    return;
}

/**
 *  @brief Function spi_test_task : Execute read on SPI bus
 *
 *  @param[in]         arg0, arg1: Arguments ( Currently not used)
 *  @retval            none
 */
void spi_test_task(UArg arg0, UArg arg1)
{
    SPI_Params   spiParams;              /* SPI params structure */

    appPrint("\n spi_test task started");
    /* Default SPI configuration parameters */
    SPI_Params_init(&spiParams);

    /* TODO: Add SPI functionality test here */
    Task_sleep(10);
    appPrint("\n spi_test task ended");
    Task_exit();
}

/**
 *  @brief Function i2c_eeprom_read_and_display_task :
 *      Reads eeprom contents through I2C and prints Board version
 *
 *  @param[in]         arg0, arg1: Arguments ( Currently not used)
 *  @retval            none
 */
void i2c_eeprom_read_and_display_task(UArg arg0, UArg arg1)
{
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
    I2C_Transaction i2cTransaction;
    bool status;
    char txBuf[2] = {0x00, 0x00};
    char boardName[20];
    char boardVersion[20];

    appPrint("\n i2c_eeprom_read_and_display task started");

    /* Initialize parameters */
    I2C_Params_init(&i2cParams);

    /* Open I2C instance */
    handle = I2C_open(BOARD_I2C_EEPROM_INSTANCE, &i2cParams);

    /* Configure common parameters with I2C transaction */
    i2cTransaction.slaveAddress = BOARD_I2C_EEPROM_ADDR;
    i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
    i2cTransaction.writeCount = 2;

     /* Get board name */
     txBuf[0] = (char)(((uint32_t) 0xFF00 & BOARD_EEPROM_BOARD_NAME_ADDR)>>8);
     txBuf[1] = (char)((uint32_t) 0xFF & BOARD_EEPROM_BOARD_NAME_ADDR);
     i2cTransaction.readBuf = boardName;
     i2cTransaction.readCount = BOARD_EEPROM_BOARD_NAME_LENGTH;
     status = I2C_transfer(handle, &i2cTransaction);
     if (status == false)
     {
         I2C_close(handle);
         appPrint("\n ERROR: I2C_transfer failed");
         goto I2C_TEST_EXIT;
     }
     boardName[BOARD_EEPROM_BOARD_NAME_LENGTH] = '\0';
     appPrint("\n Board Name read: %s", boardName);

     /* Get board version */
     txBuf[0] = (char)(((uint32_t) 0xFF00 & BOARD_EEPROM_VERSION_ADDR)>>8);
     txBuf[1] = (char)((uint32_t) 0xFF & BOARD_EEPROM_VERSION_ADDR);
     i2cTransaction.readBuf = boardVersion;
     i2cTransaction.readCount = BOARD_EEPROM_VERSION_LENGTH;
     status = I2C_transfer(handle, &i2cTransaction);
     if (status == false)
     {
         I2C_close(handle);
         appPrint("\n ERROR: I2C_transfer failed");
         goto I2C_TEST_EXIT;
     }
     boardVersion[BOARD_EEPROM_VERSION_LENGTH] = '\0';
     appPrint("\n Board version read: %s", boardVersion);

    Task_sleep(10);

I2C_TEST_EXIT:
    appPrint("\n i2c_eeprom_read_and_display task ended");
    Task_exit();
}

/**
 *  @brief Function biosTaskCreate : Task create function
 *             This function is customized to control
 *             certain parameters of the tasks. More parameters
 *             can be controlled for each task; Refer to SYSBIOS
 *             API for other parameters that can be controlled.
 *  @retval              : 0: success ; -1: fail
 */
void biosTaskCreate(ti_sysbios_knl_Task_FuncPtr taskFunctionPtr,
                    char *taskName, int taskPriority, int stackSize)
{
    Task_Params taskParams;
    Error_Block eb;
    Task_Handle task;

    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.instance->name = taskName;
    taskParams.priority = taskPriority;
    taskParams.stackSize = stackSize;
    task = Task_create(taskFunctionPtr, &taskParams, &eb);
    if (task == NULL) {
       appPrint("%s: Task_create() failed! \n", taskName);
       BIOS_exit(0);
    }
    appPrint("\n %s task created.", taskName);

    return;
}
