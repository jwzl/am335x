/**
*
*  \brief  Template application main file:
*          The main code initializes the platform , calls function
*          to create application tasks and then starts BIOS.
*          The initialization include Board specific
*          initialization and initialization of used peripherals.
*          The application specific task create function used here are
*          defined in a separate file: app.c.
*/

/* Standard header files */
#include <string.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* Board header file */
#include <ti/board/board.h>

/* Local template app header file */
#include "app.h"

/**
 *  @brief Function appExitFunction : Just prints end of application
 *             This function is plugged into BIOS to run on exit.
 *  @retval              : 0: success ; -1: fail
 */
void appExitFunction(Int argument)
{
   appPrint("\n app ended\n");
}

/* 
* board_early_init : low level initial.
* init such as, clock, pinmux, power, etc.
*/
void board_early_init(void) {
	int status;
	Board_initCfg boardCfg;

	/* here is board specific initialization
    * Note that the Board_init is specific to the
    * platform. If running on a newly custom platform
    * the Board library need to be ported and customized
    * for the new platform.
    * See Details of customizing the board library in the
    * PDK/Processor SDK User guide */
    /* Set Board initialization flags: */
    boardCfg =
#ifndef SBL_BOOT
         /* Enabling Board Pinmux, clock when using without SBL boot
          * to act as stand alone application.
          */
		 BOARD_INIT_MODULE_CLOCK |
#endif
         /* The UART_STDIO initializes the default UART port on the board
          *  and support stdio like UART_printf which is used by appPrint
          */
         BOARD_INIT_UART_STDIO;

    /* Initialize Board */
    status = Board_init(boardCfg);
    if (status != BOARD_SOK) {
        appPrint("Error: Board_init failed: error %d\r\n", status);
		BIOS_exit(-1);
    }
	/* TODO: do board level pinmux */
	
	/* TODO: Enable other module clock. */

    appPrint("Board Init complete\r\n");
}
/*
*  board_late_init: Initializes peripherals needed by your app
*/
void board_late_init(void) {
    /* UART initialization: This is redundant as it is already done
     * as part of Board_init. Included here for completion */
    UART_init();
    appPrint("Uart Init complete\r\n");

    /* I2C initialization */
    I2C_init();
    appPrint("I2C Init complete\r\n");

    /* MCSPI initialization */
    MCSPI_init();
    appPrint("MCSPI Init complete\r\n");

    /* Add any additional peripherals to be initialized here */

    appPrint("======== Peripheral Initialization complete ========\r\n");

}


int main(void)
{
    /* Add exit function */
    System_atexit(appExitFunction);

    /* First step low level init*/
	board_early_init();

    /* Second step to Initialize peripherals */
    board_late_init();

    /* Third step is to create Application tasks */
    appTasksCreate();

    /* Fourth step is to Start BIOS */
    BIOS_start();
    return (0);
}
