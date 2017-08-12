#ifndef _AHRS_H
#define _AHRS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "queue.h"
#include "queue_message.h"
#include "debug.h"
#include "control.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 
    
#define OFFSET_CALIBRATION_COUNT 100 // number of messages used to calculate the AHRS offset
    
typedef enum
{
	AHRS_STATE_INIT=0,
    AHRS_STATE_SEND_STOP,
    AHRS_STATE_SEND_START,
    AHRS_STATE_ACK_START,
    AHRS_STATE_WAIT_ALIGNMENT,
    AHRS_STATE_GET_ALIGNMENT,
    AHRS_STATE_SET_ALIGNMENT,
    AHRS_STATE_ACK_SET_PARAMS,
    AHRS_STATE_ACK_RESTART,
    AHRS_STATE_WAIT_UPDATED_ALIGNMENT,
    AHRS_STATE_CALCULATE_OFFSET,
    AHRS_STATE_RUN,
    AHRS_STATE_ERROR
} AHRS_STATES;

typedef enum
{
    AHRS_READ_HEADER_1,
    AHRS_READ_HEADER_2,
    AHRS_READ_LENGTH,
    AHRS_READ_DATA
} AHRS_READ_STATE;

typedef struct
{
    AHRS_STATES state;
    QueueHandle_t receiveQ;
    DRV_HANDLE uart_handle;
    DRV_USART_BUFFER_HANDLE read_handle;
    DRV_USART_BUFFER_HANDLE write_handle;
    uint8_t read_buffer[60];
    uint8_t write_buffer[60];
    uint8_t read_size;
    QUEUE_MESSAGE receive_message;
    QUEUE_MESSAGE send_message;
    AHRS_READ_STATE read_state;
    uint8_t offset_counter;
    int32_t accel_x_offset;
    int32_t accel_y_offset;
    int32_t accel_z_offset;
    uint8_t send_packet[10];
    float init_heading;
    float init_roll;
    float init_pitch;
} AHRS_DATA;

void AHRS_Initialize (void);
void AHRS_Tasks(void);
void AhrsUartCallback(DRV_USART_BUFFER_EVENT, DRV_USART_BUFFER_HANDLE, uintptr_t);
void SendToAhrs(QUEUE_MESSAGE*);
void SendToAhrsIsr(QUEUE_MESSAGE*);
void AddAhrsRead(uint8_t);
void AddAhrsWrite(uint8_t*, uint8_t);
void ParseAhrsOffset(uint8_t*);
void SendAhrsOffset();
void SendAhrsData(uint8_t*);


#endif /* _AHRS_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END