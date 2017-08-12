/*******************************************************************************
 System Interrupts File

  File Name:
    system_int.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "control.h"
#include "communication.h"
#include "ahrs.h"
#include "i2c.h"
#include "system_definitions.h"
#include "debug.h"
#include "queue_message.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

void IntHandlerExternalInterruptInstance0(void)
{           
    DEBUG_MESSAGE(ERROR_EXTERNAL_INT_0);
    uint8_t data[1] = {0x00};
    QUEUE_MESSAGE message;
    message.type = PHOTOELECTRIC_STATE_MESSAGE;
    message.size = 1;
    message.data = &data[0];
    SendToControlIsr(&message);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_0);
    DEBUG_MESSAGE(ERROR_EXTERNAL_INT_CLEARED);
}

void IntHandlerExternalInterruptInstance1(void)
{           
    DEBUG_MESSAGE(ERROR_EXTERNAL_INT_1);
    uint8_t data[1] = {0x01};
    QUEUE_MESSAGE message;
    message.type = PHOTOELECTRIC_STATE_MESSAGE;
    message.size = 1;
    message.data = &data[0];
    SendToControlIsr(&message);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_1);    
    DEBUG_MESSAGE(ERROR_EXTERNAL_INT_CLEARED);
}

void IntHandlerExternalInterruptInstance2(void)
{           
    DEBUG_MESSAGE(ERROR_EXTERNAL_INT_2);
    uint8_t data[1] = {0x02};
    QUEUE_MESSAGE message;
    message.type = PHOTOELECTRIC_STATE_MESSAGE;
    message.size = 1;
    message.data = &data[0];
    SendToControlIsr(&message);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_2);    
    DEBUG_MESSAGE(ERROR_EXTERNAL_INT_CLEARED);
}

void IntHandlerExternalInterruptInstance3(void)
{           
    DEBUG_MESSAGE(ERROR_EXTERNAL_INT_3);
    uint8_t data[1] = {0x01};
    QUEUE_MESSAGE message;
    message.type = PUSHER_STATE_MESSAGE;
    message.size = 1;
    message.data = &data[0];
    SendToControlIsr(&message);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_3);
    DEBUG_MESSAGE(ERROR_EXTERNAL_INT_CLEARED);
}

 
void IntHandlerDrvUsartInstance0(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart0);
    DRV_USART_TasksReceive(sysObj.drvUsart0);
    DRV_USART_TasksError(sysObj.drvUsart0);

}
 
 
 


void IntHandlerDrvUsartInstance1(void)
{

    DRV_USART_TasksTransmit(sysObj.drvUsart1);
    DRV_USART_TasksReceive(sysObj.drvUsart1);
    DRV_USART_TasksError(sysObj.drvUsart1);

}
 
 
 
 

 

 
 
 
 void IntHandlerDrvI2CInstance0(void) 
{
    DRV_I2C_Tasks(sysObj.drvI2C0);
 
}
     
 
   

  
 
  
 



 

  
  
  
  
  
 

  
  
  
  
/*******************************************************************************
 End of File
*/

