#include "ahrs.h"

AHRS_DATA ahrs_data;

void AHRS_Initialize ( void )
{
    ahrs_data.state = AHRS_STATE_INIT;
    
    // Set initial read state
    ahrs_data.read_state = AHRS_READ_HEADER_1;
    
    // Create receive queue
    ahrs_data.receiveQ = xQueueCreate(10, sizeof(QUEUE_MESSAGE));
    if (ahrs_data.receiveQ == NULL)
    {
        DEBUG_MESSAGE(ERROR_AHRS_QUEUE_NOT_CREATED);
        ahrs_data.state = AHRS_STATE_ERROR;
    }
    
    // Open UART and set callback
    ahrs_data.uart_handle = DRV_USART_Open(DRV_USART_INDEX_1, DRV_IO_INTENT_NONBLOCKING | DRV_IO_INTENT_READWRITE);
    DRV_USART_BufferEventHandlerSet(ahrs_data.uart_handle, AhrsUartCallback, (uintptr_t) NULL);    

    ahrs_data.send_message.type = AHRS_DATA_MESSAGE;
    ahrs_data.send_message.size = sizeof(ahrs_data.send_packet) / sizeof(ahrs_data.send_packet[0]);
    ahrs_data.send_message.data = &ahrs_data.send_packet[0];
    
    ahrs_data.init_heading = 0;
    ahrs_data.init_pitch = 0;
    ahrs_data.init_roll = 0;
}

void AHRS_Tasks ( void )
{
    switch ( ahrs_data.state )
    {
        case AHRS_STATE_INIT:
        {
            DEBUG_MESSAGE(ERROR_AHRS_STATE_INIT);
            TickType_t xLastWakeTime = xTaskGetTickCount();
            const TickType_t xFrequency = STARTUP_DELAY;
            vTaskDelayUntil(&xLastWakeTime, xFrequency);            
            AddAhrsRead(1);
            ahrs_data.state = AHRS_STATE_SEND_STOP;
            break;
        }
        case AHRS_STATE_SEND_STOP:
        {
            DEBUG_MESSAGE(ERROR_AHRS_STATE_SEND_STOP);
            ahrs_data.state = AHRS_STATE_SEND_START;
            uint8_t stop_command[] = {0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0xfe, 0x05, 0x01};
            AddAhrsWrite(&stop_command[0], sizeof(stop_command));         
            ahrs_data.state = AHRS_STATE_SEND_START;
            break;
        }
        case AHRS_STATE_SEND_START:
        {
            DEBUG_MESSAGE(ERROR_AHRS_STATE_SEND_START);
            TickType_t xLastWakeTime = xTaskGetTickCount();
            const TickType_t xFrequency = 1000;
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            
            uint8_t start_command[] = {0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x83, 0x8A, 0x00};
            AddAhrsWrite(&start_command[0], sizeof(start_command));            
            ahrs_data.read_state = AHRS_READ_HEADER_1;
            ahrs_data.state = AHRS_STATE_ACK_START;
            break;
        }
        case AHRS_STATE_ACK_START:
        {
            DEBUG_MESSAGE(ERROR_AHRS_STATE_ACK_START);
            if (xQueueReceive(ahrs_data.receiveQ, &ahrs_data.receive_message, portMAX_DELAY))
            {
                if (ahrs_data.receive_message.type == AHRS_READ)
                {
                    if (ahrs_data.receive_message.data[0] == 0x8A)
                    {
                        ahrs_data.state = AHRS_STATE_WAIT_ALIGNMENT;
                    }
                    else
                    {
                        ahrs_data.state = AHRS_STATE_ERROR;
                    }
                }
            }
            break;
        }
        case AHRS_STATE_WAIT_ALIGNMENT:
        {
            DEBUG_MESSAGE(ERROR_AHRS_STATE_WAIT_ALIGNMENT);
            if (xQueueReceive(ahrs_data.receiveQ, &ahrs_data.receive_message, portMAX_DELAY))
            {
                ahrs_data.state = AHRS_STATE_GET_ALIGNMENT;
            }
            break;
        }
        case AHRS_STATE_GET_ALIGNMENT:
        {
            uint8_t stop_command[] = {0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0xfe, 0x05, 0x01};
            AddAhrsWrite(&stop_command[0], sizeof(stop_command));
            
            TickType_t xLastWakeTime = xTaskGetTickCount();
            const TickType_t xFrequency = 1000;
            vTaskDelayUntil(&xLastWakeTime, xFrequency);

            uint8_t get_params_command[] = {0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x41, 0x48, 0x00};
            AddAhrsWrite(&get_params_command[0], sizeof(get_params_command));
            ahrs_data.state = AHRS_STATE_SET_ALIGNMENT;
            break;
        }
        case AHRS_STATE_SET_ALIGNMENT:
        {
            DEBUG_MESSAGE(ERROR_AHRS_STATE_SET_ALIGNMENT);
            if (xQueueReceive(ahrs_data.receiveQ, &ahrs_data.receive_message, portMAX_DELAY))
            {
                memcpy(&ahrs_data.receive_message.data[24], &ahrs_data.init_heading, 4 * sizeof(uint8_t));
                memcpy(&ahrs_data.receive_message.data[28], &ahrs_data.init_pitch, 4 * sizeof(uint8_t));
                memcpy(&ahrs_data.receive_message.data[32], &ahrs_data.init_roll, 4 * sizeof(uint8_t));
                
                int index = 36;
                for (index = 36; index < 50; index++)
                {
                    ahrs_data.receive_message.data[index] = 0x00;
                }
                
                uint8_t set_params_command[] = {0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x40, 0x47, 0x00};
                
                uint8_t param_header[] = {0xAA, 0x55, 0x00, 0x00, 0x38, 0x00};
                memcpy(&ahrs_data.write_buffer[0], &param_header[0], 6);
                memcpy(&ahrs_data.write_buffer[6], &ahrs_data.receive_message.data[0], 50);
                
                uint16_t checksum = 0;
                for (index = 2; index < 56; index++)
                {
                    checksum += ahrs_data.write_buffer[index];
                }
                ahrs_data.write_buffer[56] = checksum & 0xFF;
                ahrs_data.write_buffer[57] = (checksum >> 8) & 0xFF;                
                ahrs_data.write_buffer[6] = AHRS_MESSAGE_RATE;
                AddAhrsWrite(&set_params_command[0], sizeof(set_params_command));
                AddAhrsWrite(&ahrs_data.write_buffer[0], 58);
                
                ahrs_data.state = AHRS_STATE_ACK_SET_PARAMS;
            }
            break;
        }
        case AHRS_STATE_ACK_SET_PARAMS:
        {
            DEBUG_MESSAGE(ERROR_AHRS_STATE_ACK_SET_PARAMS);
            if (xQueueReceive(ahrs_data.receiveQ, &ahrs_data.receive_message, portMAX_DELAY))
            {
                TickType_t xLastWakeTime = xTaskGetTickCount();
                const TickType_t xFrequency = 1000;
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                
                uint8_t start_command[] = {0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x83, 0x8A, 0x00};
                AddAhrsWrite(&start_command[0], sizeof(start_command));       

                ahrs_data.state = AHRS_STATE_ACK_RESTART;
            }
            break;
        }
        case AHRS_STATE_ACK_RESTART:
        {
            DEBUG_MESSAGE(ERROR_AHRS_STATE_ACK_RESTART);
            if (xQueueReceive(ahrs_data.receiveQ, &ahrs_data.receive_message, portMAX_DELAY))
            {
                if (ahrs_data.receive_message.type == AHRS_READ)
                {
                    if (ahrs_data.receive_message.data[0] == 0x8A)
                    {
                        ahrs_data.state = AHRS_STATE_WAIT_UPDATED_ALIGNMENT;
                    }
                    else
                    {
                        ahrs_data.state = AHRS_STATE_ERROR;
                    }
                }
            }
            break;
        }
        case AHRS_STATE_WAIT_UPDATED_ALIGNMENT:
        {
            DEBUG_MESSAGE(ERROR_AHRS_STATE_WAIT_UPDATED_ALIGNMENT);
            if (xQueueReceive(ahrs_data.receiveQ, &ahrs_data.receive_message, portMAX_DELAY))
            {
                ahrs_data.state = AHRS_STATE_CALCULATE_OFFSET;
            }
            break;
        }
        case AHRS_STATE_CALCULATE_OFFSET:
        {
            DEBUG_MESSAGE(ERROR_AHRS_STATE_CALCULATE_OFFSET);
            if (xQueueReceive(ahrs_data.receiveQ, &ahrs_data.receive_message, portMAX_DELAY))
            {
                ParseAhrsOffset(&ahrs_data.receive_message.data[0]);
                ahrs_data.offset_counter++;
                if (ahrs_data.offset_counter == OFFSET_CALIBRATION_COUNT)
                {
                    SendAhrsOffset();
                    ahrs_data.state = AHRS_STATE_RUN;
                }
            }
            break;
        }
        case AHRS_STATE_RUN:
        {
            DEBUG_MESSAGE(ERROR_AHRS_STATE_RUN);
            if (xQueueReceive(ahrs_data.receiveQ, &ahrs_data.receive_message, portMAX_DELAY))
            {
                if (ahrs_data.receive_message.type == AHRS_READ)
                {
                    SendAhrsData(&ahrs_data.receive_message.data[0]);
                }
            }
            break;
        }
        case AHRS_STATE_ERROR:
        {
            DEBUG_MESSAGE(ERROR_AHRS_STATE_ERROR);
            break;
        }
        default:
        {
            break;
        }
    }
}

void AhrsUartCallback(DRV_USART_BUFFER_EVENT event, DRV_USART_BUFFER_HANDLE handle, uintptr_t context)
{
    DEBUG_MESSAGE(ERROR_AHRS_CALLBACK);
    switch (event)
    {
        case DRV_USART_BUFFER_EVENT_COMPLETE:
        {
            if (handle == ahrs_data.read_handle)
            {
                DEBUG_MESSAGE(ERROR_AHRS_READ);
                switch (ahrs_data.read_state)
                {
                    case AHRS_READ_HEADER_1:
                    {
                        DEBUG_MESSAGE(ERROR_AHRS_READ_HEADER_1);
                        uint8_t data = ahrs_data.read_buffer[0];
                        if (data = 0xAA)
                        {
                            ahrs_data.read_state = AHRS_READ_HEADER_2;
                            AddAhrsRead(1);
                        }
                        else
                        {
                            ahrs_data.read_state = AHRS_READ_HEADER_1;
                            AddAhrsRead(1);
                        }
                        break;
                    }
                    case AHRS_READ_HEADER_2:
                    {
                        DEBUG_MESSAGE(ERROR_ARHS_READ_HEADER_2);
                        uint8_t data = ahrs_data.read_buffer[0];
                        if (data = 0x55)
                        {
                            ahrs_data.read_state = AHRS_READ_LENGTH;
                            AddAhrsRead(4);
                        }
                        else
                        {
                            ahrs_data.read_state = AHRS_READ_HEADER_1;
                            AddAhrsRead(1);
                        }
                        break;
                    }
                    case AHRS_READ_LENGTH:
                    {
                        DEBUG_MESSAGE(ERROR_AHRS_READ_LENGTH);
                        ahrs_data.read_state = AHRS_READ_DATA;
                        ahrs_data.read_size = ahrs_data.read_buffer[2] - 4;
                        AddAhrsRead(ahrs_data.read_size);
                        break;
                    }
                    case AHRS_READ_DATA:
                    {
                        DEBUG_MESSAGE(ERROR_AHRS_READ_DATA);
                        ahrs_data.read_state = AHRS_READ_HEADER_1;
                        QUEUE_MESSAGE message;
                        message.type = AHRS_READ;
                        message.size = ahrs_data.read_size;
                        message.data = &ahrs_data.read_buffer[0];
                        SendToAhrsIsr(&message);
                        AddAhrsRead(1);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
            else if (handle == ahrs_data.write_handle) 
            {
                
            }
            break;
        }
        case DRV_USART_BUFFER_EVENT_ERROR:
        {
            ahrs_data.state = AHRS_STATE_ERROR;
            break;
        }
        default:
        {
            break;
        }
    }
}

void SendToAhrs(QUEUE_MESSAGE* message)
{
    DEBUG_MESSAGE(ERROR_AHRS_RECEIVE_MESSAGE);
    if (xQueueSendToBack(ahrs_data.receiveQ, message, 1) != pdPASS)
    {
        DEBUG_MESSAGE(ERROR_AHRS_FAILED_TO_QUEUE);
        ahrs_data.state = AHRS_STATE_ERROR;
    }
}

void SendToAhrsIsr(QUEUE_MESSAGE* message)
{
    DEBUG_MESSAGE(ERROR_AHRS_RECEIVE_MESSAGE);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendToBackFromISR(ahrs_data.receiveQ, message, &xHigherPriorityTaskWoken);
    if( xHigherPriorityTaskWoken )
    {
        taskYIELD ();
    }
}

void AddAhrsRead(uint8_t size)
{
    DRV_USART_BufferAddRead(ahrs_data.uart_handle, &ahrs_data.read_handle, &ahrs_data.read_buffer[0], size);
    if(DRV_USART_BUFFER_HANDLE_INVALID == ahrs_data.read_handle)
    {
        ahrs_data.state = AHRS_STATE_ERROR;
    }
}

void AddAhrsWrite(uint8_t* message, uint8_t size)
{
    DRV_USART_BufferAddWrite(ahrs_data.uart_handle, &ahrs_data.write_handle, &message[0], size);
    if(DRV_USART_BUFFER_HANDLE_INVALID == ahrs_data.write_handle)
    {
        ahrs_data.state = AHRS_STATE_ERROR;
    }
}

void ParseAhrsOffset(uint8_t* message)
{
    ahrs_data.accel_x_offset += (int16_t)(message[12] | ((int8_t)message[13]) << 8);
    ahrs_data.accel_y_offset += (int16_t)(message[14] | ((int8_t)message[15]) << 8);
    ahrs_data.accel_z_offset += (int32_t)(message[16] | ((int8_t)message[17]) << 8);
}

void SendAhrsOffset()
{
    ahrs_data.accel_x_offset = ahrs_data.accel_x_offset / OFFSET_CALIBRATION_COUNT;
    ahrs_data.accel_y_offset = ahrs_data.accel_y_offset / OFFSET_CALIBRATION_COUNT;
    ahrs_data.accel_z_offset = ahrs_data.accel_z_offset / OFFSET_CALIBRATION_COUNT;
    
    ahrs_data.send_packet[0] = (ahrs_data.accel_x_offset >> 0) & 0xFF;
    ahrs_data.send_packet[1] = (ahrs_data.accel_x_offset >> 8) & 0xFF;
    ahrs_data.send_packet[2] = (ahrs_data.accel_y_offset >> 0) & 0xFF;
    ahrs_data.send_packet[3] = (ahrs_data.accel_y_offset >> 8) & 0xFF;
    ahrs_data.send_packet[4] = (ahrs_data.accel_z_offset >> 0) & 0xFF;
    ahrs_data.send_packet[5] = (ahrs_data.accel_z_offset >> 8) & 0xFF;
    
    QUEUE_MESSAGE message;
    message.type = AHRS_START_MESSAGE;
    message.size = 6;
    message.data = &ahrs_data.send_packet[0];
    
    SendToControl(&message);
}

void SendAhrsData(uint8_t* data)
{
    ahrs_data.send_packet[0] = data[12];
    ahrs_data.send_packet[1] = data[13];
    ahrs_data.send_packet[2] = data[14];
    ahrs_data.send_packet[3] = data[15];
    ahrs_data.send_packet[4] = data[16];
    ahrs_data.send_packet[5] = data[17];
    ahrs_data.send_packet[6] = data[2];
    ahrs_data.send_packet[7] = data[3];
    ahrs_data.send_packet[8] = data[4];
    ahrs_data.send_packet[9] = data[5];
    
    SendToControl(&ahrs_data.send_message);
}