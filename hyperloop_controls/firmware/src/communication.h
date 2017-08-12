#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "queue.h"
#include "queue_message.h"
#include "debug.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

typedef enum
{
    COMMUNICATION_STATE_INIT=0,
    COMMUNICATION_STATE_RUN,
    COMMUNICATION_STATE_ERROR
} COMMUNICATION_STATES;

typedef enum
{
    COMMUNICATION_READ_HEADER_1,
    COMMUNICATION_READ_HEADER_2,
    COMMUNICATION_READ_LENGTH,
    COMMUNICATION_READ_DATA
} COMMUNICATION_READ_STATE;

typedef struct
{
    COMMUNICATION_STATES state;
    QueueHandle_t receiveQ;
    DRV_HANDLE uart_handle;
    DRV_USART_BUFFER_HANDLE read_handle;
    DRV_USART_BUFFER_HANDLE write_handle;
    uint8_t read_buffer[50];
    uint8_t write_buffer[50];
    uint8_t read_size;
    QUEUE_MESSAGE receive_message;
    COMMUNICATION_READ_STATE read_state;
} COMMUNICATION_DATA;

void COMMUNICATION_Initialize(void);
void COMMUNICATION_Tasks(void);
void CommunicationUARTCallback(DRV_USART_BUFFER_EVENT, DRV_USART_BUFFER_HANDLE, uintptr_t);
void SendToCommunication(QUEUE_MESSAGE*);
void SendToCommunicationIsr(QUEUE_MESSAGE*);
void AddCommunicationRead(uint8_t);
void AddCommunicationWrite(QUEUE_MESSAGE*);


#endif /* _COMMUNICATION_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END