#include "i2c.h"

I2C_DATA i2c_data;

void I2C_Initialize ( void )
{
    i2c_data.state = I2C_STATE_INIT;
    i2c_data.i2c_handle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READ | DRV_IO_INTENT_WRITE | DRV_IO_INTENT_NONBLOCKING);
    DRV_I2C_BufferEventHandlerSet (i2c_data.i2c_handle, I2C_Callback, (uintptr_t) NULL);
    i2c_data.receive_queue = xQueueCreate(10, sizeof(QUEUE_MESSAGE));
    if (i2c_data.receive_queue == NULL)
    {
        DEBUG_MESSAGE(ERROR_I2C_QUEUE_NOT_CREATED);
        i2c_data.state = I2C_STATE_ERROR;
    }
    i2c_data.timer_handle = xTimerCreate("I2C Timer", I2C_MESSAGE_PERIOD / portTICK_PERIOD_MS, pdTRUE, (void*) 0, ReadSensorData);
}

void I2C_Tasks ( void )
{
    switch (i2c_data.state)
    {
        case I2C_STATE_INIT:
        {
            TickType_t xLastWakeTime = xTaskGetTickCount();
            const TickType_t xFrequency = STARTUP_DELAY;
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            i2c_data.state = I2C_STATE_WAIT_READY;
            break;
        }
        case I2C_STATE_WAIT_READY:
        {
            i2c_data.slave_address = 0x1D << 1;
            
            uint8_t status_register[] = {0x0C};
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &status_register[0], sizeof(status_register), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            uint8_t read_data[1];
            i2c_data.read_handle = DRV_I2C_BufferAddRead(i2c_data.i2c_handle, &(i2c_data.slave_address), &(read_data[0]), sizeof(read_data), NULL);
            while(!(DRV_I2C_BufferStatus(i2c_data.read_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE ));
            
            if (!(read_data[0] & 0x02))
            {
                i2c_data.slave_address = 0x1E << 1;
            
                uint8_t status_register[] = {0x0C};
                i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &status_register[0], sizeof(status_register), NULL);
                while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));

                uint8_t read_data[1];
                i2c_data.read_handle = DRV_I2C_BufferAddRead(i2c_data.i2c_handle, &(i2c_data.slave_address), &(read_data[0]), sizeof(read_data), NULL);
                while(!(DRV_I2C_BufferStatus(i2c_data.read_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE ));

                if (!(read_data[0] & 0x02))
                {
                    i2c_data.slave_address = 0x1F << 1;

                    uint8_t status_register[] = {0x0C};
                    i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &status_register[0], sizeof(status_register), NULL);
                    while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));

                    uint8_t read_data[1];
                    i2c_data.read_handle = DRV_I2C_BufferAddRead(i2c_data.i2c_handle, &(i2c_data.slave_address), &(read_data[0]), sizeof(read_data), NULL);
                    while(!(DRV_I2C_BufferStatus(i2c_data.read_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE ));
                
                    if (!(read_data[0] & 0x02))
                    {
                        i2c_data.slave_address = 0x2D << 1;

                        uint8_t status_register[] = {0x0C};
                        i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &status_register[0], sizeof(status_register), NULL);
                        while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));

                        uint8_t read_data[1];
                        i2c_data.read_handle = DRV_I2C_BufferAddRead(i2c_data.i2c_handle, &(i2c_data.slave_address), &(read_data[0]), sizeof(read_data), NULL);
                        while(!(DRV_I2C_BufferStatus(i2c_data.read_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE ));
                        
                        if (!(read_data[0] & 0x02))
                        {            
                            i2c_data.state = I2C_STATE_SETUP;
                            TickType_t xLastWakeTime = xTaskGetTickCount();
                            const TickType_t xFrequency = 10;
                            vTaskDelayUntil(&xLastWakeTime, xFrequency);
                        }
                    }
                }
            }
            break;
        }
        case I2C_STATE_SETUP:
        {
            i2c_data.slave_address = 0x1D << 1;
            
            // Set the ADC to shutdown mode
            uint8_t shutdown[] = {0x00, 0x00};
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &shutdown[0], sizeof(shutdown), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Advanced Configuration Register to use the internal voltage source and set it to Mode 1
            uint8_t configuration[] = {0x0B, 0x02};
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &configuration[0], sizeof(configuration), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Conversion Rate Register to use all channels
            uint8_t conversion[] = {0x07, 0x00};
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &conversion[0], sizeof(conversion), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Channel Disable Register to enable all channels
            uint8_t channel[] = {0x08, 0x00};
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &channel[0], sizeof(channel), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Interrupt Mask Register to disable the interrupt pin
            uint8_t inter[] = {0x03, 0xFF};
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &inter[0], sizeof(inter), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Start the ADC
            uint8_t start[] = {0x00, 0x01};
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &start[0], sizeof(start), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
    
            i2c_data.slave_address = 0x1E << 1;
            
            // Set the ADC to shutdown mode
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &shutdown[0], sizeof(shutdown), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Advanced Configuration Register to use the internal voltage source and set it to Mode 1
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &configuration[0], sizeof(configuration), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Conversion Rate Register to use all channels
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &conversion[0], sizeof(conversion), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Channel Disable Register to enable all channels
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &channel[0], sizeof(channel), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Interrupt Mask Register to disable the interrupt pin
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &inter[0], sizeof(inter), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Start the ADC
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &start[0], sizeof(start), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            i2c_data.slave_address = 0x1F << 1;
            
            // Set the ADC to shutdown mode
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &shutdown[0], sizeof(shutdown), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Advanced Configuration Register to use the internal voltage source and set it to Mode 1
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &configuration[0], sizeof(configuration), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Conversion Rate Register to use all channels
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &conversion[0], sizeof(conversion), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Channel Disable Register to enable all channels
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &channel[0], sizeof(channel), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Interrupt Mask Register to disable the interrupt pin
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &inter[0], sizeof(inter), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Start the ADC
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &start[0], sizeof(start), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            i2c_data.slave_address = 0x2D << 1;
            
            // Set the ADC to shutdown mode
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &shutdown[0], sizeof(shutdown), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Advanced Configuration Register to use the internal voltage source and set it to Mode 1
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &configuration[0], sizeof(configuration), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Conversion Rate Register to use all channels
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &conversion[0], sizeof(conversion), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Channel Disable Register to enable all channels
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &channel[0], sizeof(channel), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Program the Interrupt Mask Register to disable the interrupt pin
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &inter[0], sizeof(inter), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            // Start the ADC
            i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &start[0], sizeof(start), NULL);
            while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
            
            TickType_t xLastWakeTime = xTaskGetTickCount();
            const TickType_t xFrequency = 10;
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            
            QUEUE_MESSAGE message;
            message.type = I2C_START_MESSAGE;
            SendToControl(&message);
            
            if (xTimerStart(i2c_data.timer_handle, 0) == pdPASS)
            {
                i2c_data.state = I2C_STATE_RUN;
            }
            else
            {
                DEBUG_MESSAGE(ERROR_I2C_TIMER_NOT_STARTED);
                i2c_data.state = I2C_STATE_ERROR;
            }
            break;
        }
        case I2C_STATE_RUN:
        {
            if (xQueueReceive(i2c_data.receive_queue, &i2c_data.receive_message, portMAX_DELAY))
            {

            }
            break;
        }
        case I2C_STATE_IDLE:
        {
            break;
        }
        case I2C_STATE_ERROR:
        {
            DEBUG_MESSAGE(ERROR_I2C_ERROR);
            break;
        }
        default:
        {
            break;
        }
    }
}

void I2C_Callback(DRV_I2C_BUFFER_EVENT event, DRV_I2C_BUFFER_HANDLE handle, uintptr_t context)
{
    switch(event)
    {
        case DRV_I2C_SEND_STOP_EVENT:
        {
            DRV_I2C_StopEventSend(i2c_data.i2c_handle);
            break;
        }
        case DRV_I2C_SEND_RESTART_EVENT:
        {
            DRV_I2C_RestartEventSend(i2c_data.i2c_handle);           
            break;
        }
        case DRV_I2C_BUFFER_EVENT_COMPLETE:  
        {
            break;
        }
        case DRV_I2C_BUFFER_EVENT_ERROR:
        {
            i2c_data.state = I2C_STATE_ERROR;
            DEBUG_MESSAGE(ERROR_I2C_BUFFER_ERROR);
            break;
        }
        default:
        {
            break;         
        }
    }
}

void SendToI2c(QUEUE_MESSAGE* message)
{
    DEBUG_MESSAGE(ERROR_I2C_RECEIVE_MESSAGE);
    if (xQueueSendToBack(i2c_data.receive_queue, message, 1) != pdPASS)
    {
        DEBUG_MESSAGE(ERROR_I2C_FAILED_TO_QUEUE);
        i2c_data.state = I2C_STATE_ERROR;
    }
}

void SendToI2cIsr(QUEUE_MESSAGE* message)
{
    DEBUG_MESSAGE(ERROR_I2C_RECEIVE_MESSAGE);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendToBackFromISR(i2c_data.receive_queue, message, &xHigherPriorityTaskWoken);
    if( xHigherPriorityTaskWoken )
    {
        taskYIELD ();
    }
}

void ReadSensorData(TimerHandle_t handle)
{
    uint8_t channel[1];
    uint8_t index = 0;
    
    i2c_data.slave_address = 0x1D << 1;
    
    for (channel[0] = 0x20; channel[0] <= 0x27; channel[0]++)
    {
        i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &channel[0], sizeof(channel), NULL);
        while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));

        i2c_data.read_handle = DRV_I2C_BufferAddRead(i2c_data.i2c_handle, &i2c_data.slave_address, &i2c_data.raw_data[index], 2, NULL);
        while (!(DRV_I2C_BufferStatus(i2c_data.read_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
        index += 2;
    }
    
    i2c_data.slave_address = 0x1E << 1;
    
    for (channel[0] = 0x20; channel[0] <= 0x27; channel[0]++)
    {
        i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &channel[0], sizeof(channel), NULL);
        while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));

        i2c_data.read_handle = DRV_I2C_BufferAddRead(i2c_data.i2c_handle, &i2c_data.slave_address, &i2c_data.raw_data[index], 2, NULL);
        while (!(DRV_I2C_BufferStatus(i2c_data.read_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
        index += 2;
    }
    
    i2c_data.slave_address = 0x1F << 1;
    
    for (channel[0] = 0x20; channel[0] <= 0x27; channel[0]++)
    {
        i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &channel[0], sizeof(channel), NULL);
        while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));

        i2c_data.read_handle = DRV_I2C_BufferAddRead(i2c_data.i2c_handle, &i2c_data.slave_address, &i2c_data.raw_data[index], 2, NULL);
        while (!(DRV_I2C_BufferStatus(i2c_data.read_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
        index += 2;
    }
    
    i2c_data.slave_address = 0x2D << 1;
    
    for (channel[0] = 0x20; channel[0] <= 0x27; channel[0]++)
    {
        i2c_data.write_handle = DRV_I2C_BufferAddWrite(i2c_data.i2c_handle, &i2c_data.slave_address, &channel[0], sizeof(channel), NULL);
        while (!(DRV_I2C_BufferStatus(i2c_data.write_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));

        i2c_data.read_handle = DRV_I2C_BufferAddRead(i2c_data.i2c_handle, &i2c_data.slave_address, &i2c_data.raw_data[index], 2, NULL);
        while (!(DRV_I2C_BufferStatus(i2c_data.read_handle) == DRV_I2C_BUFFER_EVENT_COMPLETE));
        index += 2;
    }
    
    QUEUE_MESSAGE message;
    message.type = I2C_DATA_MESSAGE;
    message.size = sizeof(i2c_data.raw_data);
    message.data = &i2c_data.raw_data[0];
    
    SendToControl(&message);
}