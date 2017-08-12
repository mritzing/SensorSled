/*******************************************************************************
 System Tasks File

  File Name:
    system_tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled state
    machines.

  Description:
    This file contains source code necessary to maintain system's polled state
    machines.  It implements the "SYS_Tasks" function that calls the individual
    "Tasks" functions for all the MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "control.h"
#include "communication.h"
#include "ahrs.h"
#include "i2c.h"


// *****************************************************************************
// *****************************************************************************
// Section: Local Prototypes
// *****************************************************************************
// *****************************************************************************
 
static void _SYS_Tasks ( void );
static void _CONTROL_Tasks(void);
static void _COMMUNICATION_Tasks(void);
static void _AHRS_Tasks(void);
static void _I2C_Tasks(void);


// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/

void SYS_Tasks ( void )
{
    /* Create OS Thread for Sys Tasks. */
    xTaskCreate((TaskFunction_t) _SYS_Tasks,
                "Sys Tasks",
                1024, NULL, 1, NULL);

    /* Create OS Thread for CONTROL Tasks. */
    xTaskCreate((TaskFunction_t) _CONTROL_Tasks,
                "CONTROL Tasks",
                1024, NULL, 5, NULL);

    /* Create OS Thread for COMMUNICATION Tasks. */
    xTaskCreate((TaskFunction_t) _COMMUNICATION_Tasks,
                "COMMUNICATION Tasks",
                1024, NULL, 4, NULL);

    /* Create OS Thread for AHRS Tasks. */
    xTaskCreate((TaskFunction_t) _AHRS_Tasks,
                "AHRS Tasks",
                1024, NULL, 3, NULL);

    /* Create OS Thread for I2C Tasks. */
    xTaskCreate((TaskFunction_t) _I2C_Tasks,
                "I2C Tasks",
                1024, NULL, 2, NULL);

    /**************
     * Start RTOS * 
     **************/
    vTaskStartScheduler(); /* This function never returns. */
}


/*******************************************************************************
  Function:
    void _SYS_Tasks ( void )

  Summary:
    Maintains state machines of system modules.
*/

static void _SYS_Tasks ( void )
{
    while(1)
    {
        /* Maintain system services */
        SYS_DEVCON_Tasks(sysObj.sysDevcon);

        /* Maintain Device Drivers */

        /* Maintain Middleware */

        /* Task Delay */
    }
}


/*******************************************************************************
  Function:
    void _CONTROL_Tasks ( void )

  Summary:
    Maintains state machine of CONTROL.
*/

static void _CONTROL_Tasks(void)
{
    while(1)
    {
        CONTROL_Tasks();
    }
}


/*******************************************************************************
  Function:
    void _COMMUNICATION_Tasks ( void )

  Summary:
    Maintains state machine of COMMUNICATION.
*/

static void _COMMUNICATION_Tasks(void)
{
    while(1)
    {
        COMMUNICATION_Tasks();
    }
}


/*******************************************************************************
  Function:
    void _AHRS_Tasks ( void )

  Summary:
    Maintains state machine of AHRS.
*/

static void _AHRS_Tasks(void)
{
    while(1)
    {
        AHRS_Tasks();
    }
}


/*******************************************************************************
  Function:
    void _I2C_Tasks ( void )

  Summary:
    Maintains state machine of I2C.
*/

static void _I2C_Tasks(void)
{
    while(1)
    {
        I2C_Tasks();
    }
}


/*******************************************************************************
 End of File
 */

