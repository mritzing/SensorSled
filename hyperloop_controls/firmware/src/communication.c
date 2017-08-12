#include "communication.h"

COMMUNICATION_DATA communication_data;

void COMMUNICATION_Initialize ( void )
{
    // Set initial state
    communication_data.state = COMMUNICATION_STATE_INIT;
    
    // Set initial read state
    communication_data.read_state = COMMUNICATION_READ_HEADER_1;
    
    // Create receive queue
    communication_data.receiveQ = xQueueCreate(10, sizeof(QUEUE_MESSAGE));
    if (communication_data.receiveQ == NULL)
    {
        DEBUG_MESSAGE(ERROR_COMMUNICATION_QUEUE_NOT_CREATED);
        communication_data.state = COMMUNICATION_STATE_ERROR;
    }
    
    // Open UART and set callback
    communication_data.uart_handle = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_NONBLOCKING | DRV_IO_INTENT_READWRITE);
    DRV_USART_BufferEventHandlerSet(communication_data.uart_handle, CommunicationUARTCallback, (uintptr_t) NULL);
}

void COMMUNICATION_Tasks ( void )
{
    switch ( communication_data.state )
    {
        case COMMUNICATION_STATE_INIT:
        {
            TickType_t xLastWakeTime = xTaskGetTickCount();
            const TickType_t xFrequency = STARTUP_DELAY;
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            communication_data.state = COMMUNICATION_STATE_RUN;
            AddCommunicationRead(1);
            QUEUE_MESSAGE message;
            message.type = COMMUNICATION_START_MESSAGE;
            SendToControl(&message);
            break;
        }
        case COMMUNICATION_STATE_RUN:
        {
            if (xQueueReceive(communication_data.receiveQ, &communication_data.receive_message, portMAX_DELAY))
            {
                if (communication_data.receive_message.type == CONTROL_DATA_MESSAGE)
                {
                    AddCommunicationWrite(&communication_data.receive_message);
                }
                if (communication_data.receive_message.type == CONTROL_ALERT_MESSAGE)
                {
                    AddCommunicationWrite(&communication_data.receive_message);
                }
                if (communication_data.receive_message.type == COMMUNICATION_READ)
                {
                    SendToControl(&communication_data.receive_message);
                }
            }
            break;
        }
        case COMMUNICATION_STATE_ERROR:
        {
            DEBUG_MESSAGE(ERROR_COMMUNICATION_ERROR);
            break;
        }
        default:
        {
            break;
        }
    }
}

void CommunicationUARTCallback(DRV_USART_BUFFER_EVENT event, DRV_USART_BUFFER_HANDLE handle, uintptr_t context)
{
    DEBUG_MESSAGE(ERROR_COMMUNICATION_CALLBACK);
    switch(event){
        case DRV_USART_BUFFER_EVENT_COMPLETE:
        {
            if (handle == communication_data.read_handle) // Read Complete
            {
                DEBUG_MESSAGE(ERROR_COMMUNICATION_READ);
                switch (communication_data.read_state)
                {
                    // Read header 0xAA
                    case COMMUNICATION_READ_HEADER_1:
                    {
                        DEBUG_MESSAGE(ERROR_COMMUNICATION_READ_HEADER_1);
                        uint8_t data = communication_data.read_buffer[0];
                        if (data == 0xAA)
                        {
                            communication_data.read_state = COMMUNICATION_READ_HEADER_2;
                            AddCommunicationRead(1);
                        }
                        else
                        {
                            communication_data.read_state = COMMUNICATION_READ_HEADER_1;
                            AddCommunicationRead(1);
                        }
                        break;
                    }
                    // Read header 0x55
                    case COMMUNICATION_READ_HEADER_2:
                    {
                        DEBUG_MESSAGE(ERROR_COMMUNICATION_READ_HEADER_2);
                        uint8_t data = communication_data.read_buffer[0];
                        if (data == 0x55)
                        {
                            communication_data.read_state = COMMUNICATION_READ_LENGTH;
                            AddCommunicationRead(4);
                        }
                        else
                        {
                            communication_data.read_state = COMMUNICATION_READ_HEADER_1;
                            AddCommunicationRead(1);
                        }
                        break;
                    }
                    // Read the length to the message
                    case COMMUNICATION_READ_LENGTH:
                    {
                        DEBUG_MESSAGE(ERROR_COMMUNICATION_READ_LENGTH);
                        communication_data.read_state = COMMUNICATION_READ_DATA;
                        communication_data.read_size = communication_data.read_buffer[2];
                        AddCommunicationRead(communication_data.read_size);
                        break;
                    }
                    // Read the data
                    case COMMUNICATION_READ_DATA:
                    {
                        DEBUG_MESSAGE(ERROR_COMMUNICATION_READ_DATA);
                        communication_data.read_state = COMMUNICATION_READ_HEADER_1;
                        QUEUE_MESSAGE message;
                        message.type = COMMUNICATION_READ;
                        message.size = communication_data.read_size;
                        message.data = &communication_data.read_buffer[0];
                        SendToCommunicationIsr(&message);
                        AddCommunicationRead(1);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
            else if (handle == communication_data.write_handle) // Write Complete
            {
                
            }
            break;
        }
        case DRV_USART_BUFFER_EVENT_ERROR:
        {
            DEBUG_MESSAGE(ERROR_COMMUNICATION_ERROR);
            break;
        }
        default:
        {
            break;
        }
    }
}

void SendToCommunication(QUEUE_MESSAGE* message)
{
    DEBUG_MESSAGE(ERROR_COMMUNICATION_RECEIVE_MESSAGE);
    if (xQueueSendToBack(communication_data.receiveQ, message, 1) != pdPASS)
    {
        DEBUG_MESSAGE(ERROR_COMMUNICATION_FAILED_TO_QUEUE);
        communication_data.state = COMMUNICATION_STATE_ERROR;
    }
}

void SendToCommunicationIsr(QUEUE_MESSAGE* message)
{
    DEBUG_MESSAGE(ERROR_COMMUNICATION_RECEIVE_MESSAGE);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendToBackFromISR(communication_data.receiveQ, message, &xHigherPriorityTaskWoken);
    if( xHigherPriorityTaskWoken )
    {
        taskYIELD ();
    }
}

void AddCommunicationRead(uint8_t size) {
    DRV_USART_BufferAddRead(communication_data.uart_handle, &communication_data.read_handle, &communication_data.read_buffer[0], size);
    if(DRV_USART_BUFFER_HANDLE_INVALID == communication_data.read_handle)
    {
        communication_data.state = COMMUNICATION_STATE_ERROR;
    }
}

void AddCommunicationWrite(QUEUE_MESSAGE* message)
{
    // Write message header
    uint8_t header[] = {0xAA, 0x55, (message->type & 0xFF), 0x00, (message->size & 0xFF), ((message->size >> 8) & 0xFF + 4)};
    DRV_USART_BufferAddWrite(communication_data.uart_handle, &communication_data.write_handle, &header[0], sizeof(header));
    if(DRV_USART_BUFFER_HANDLE_INVALID == communication_data.write_handle)
    {
        communication_data.state = COMMUNICATION_STATE_ERROR;
    }
    
    // Write message data
    DRV_USART_BufferAddWrite(communication_data.uart_handle, &communication_data.write_handle, &message->data[0], message->size);
    if(DRV_USART_BUFFER_HANDLE_INVALID == communication_data.write_handle)
    {
        communication_data.state = COMMUNICATION_STATE_ERROR;
    }
}