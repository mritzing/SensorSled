#include "debug.h"

void DEBUG_Initialize()
{
    SYS_PORTS_DirectionSelect(PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT,PORT_CHANNEL_E, 0xFF);
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_E, 0xFF);
}

void DEBUG_MESSAGE(ERROR_CODE error)
{
    if (error != 0)
    {
        if (DEBUG_PORT_ENABLE == true) 
        {
            SYS_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, error);
        }
    }
}

void DEBUG_NEW_MESSAGE(char error)
{
    SYS_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, error);
}