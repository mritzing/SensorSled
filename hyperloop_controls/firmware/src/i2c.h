#ifndef _I2C_H
#define _I2C_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "control.h"
#include "queue_message.h"
#include "queue.h"
#include "timers.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

typedef enum
{
	I2C_STATE_INIT=0,
    I2C_STATE_WAIT_READY,
    I2C_STATE_SETUP,
    I2C_STATE_RUN,
    I2C_STATE_IDLE,
    I2C_STATE_ERROR,
} I2C_STATES;

typedef struct
{
    I2C_STATES state;
    TimerHandle_t timer_handle;
    QueueHandle_t receive_queue;
    DRV_HANDLE i2c_handle;
    DRV_I2C_BUFFER_HANDLE read_handle;
    DRV_I2C_BUFFER_HANDLE write_handle;
    QUEUE_MESSAGE receive_message;
    uint8_t slave_address;
    uint8_t raw_data[64];
} I2C_DATA;

void I2C_Initialize ( void );
void I2C_Tasks( void );
void I2C_Callback(DRV_I2C_BUFFER_EVENT, DRV_I2C_BUFFER_HANDLE, uintptr_t);
void SendToI2c(QUEUE_MESSAGE*);
void SendToI2cIsr(QUEUE_MESSAGE*);
void ReadSensorData(TimerHandle_t);

#endif /* _I2C_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END