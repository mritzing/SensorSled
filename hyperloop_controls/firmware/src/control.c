#include "control.h"

CONTROL_DATA control_data;

void CONTROL_Initialize(void) {
    control_data.state = CONTROL_STATE_INIT;

    // Create receive queue
    control_data.receiveQ = xQueueCreate(10, sizeof (QUEUE_MESSAGE));
    if (control_data.receiveQ == NULL) { // Queue not created
        control_data.state = CONTROL_STATE_ERROR;
        DEBUG_MESSAGE(ERROR_CONTROL_QUEUE_NOT_CREATED);
    }

    // Create timer to control the send message rate
    control_data.message_timer = xTimerCreate("Message Timer", MESSAGE_PERIOD / portTICK_PERIOD_MS, pdTRUE, (void*) 0, SendControlData);

    // Create a millisecond timer
    control_data.ms_timer = xTimerCreate("1 MS Timer", 1 / portTICK_PERIOD_MS, pdTRUE, (void*) 0, MilliTimerCallback);

    // Create timer for failsafe braking
    control_data.braking_start_timer = xTimerCreate("Braking Start Timer", MAX_BRAKING_TIME / portTICK_PERIOD_MS, pdFALSE, (void*) 0, BrakingTimerCallback);

    // Set the pod's total run time to 0 ms
    control_data.current_time_ms = 0;

    // Initialize braking port to output
    SYS_PORTS_DirectionSelect(PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_D, 1 << 9);

    //Initialize propulsion port to output 
    SYS_PORTS_DirectionSelect(PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, PORT_CHANNEL_A, 1 << 9);

    // Set the photoelectric status
    control_data.photoelectric_0 = PHOTOELECTRIC_STATUS_OFF;
    control_data.photoelectric_1 = PHOTOELECTRIC_STATUS_OFF;
    control_data.photoelectric_2 = PHOTOELECTRIC_STATUS_OFF;

    // Set the pusher status
    control_data.telemetry.pusher_status = PUSHER_STATUS_DETACHED;

    // Set the pod's initial kinematics
    ResetTelemetry();

    // Initialize the status of the threads to not running
    control_data.ahrs_thread_status = THREAD_NOT_RUNNING;
    control_data.i2c_thread_status = THREAD_NOT_RUNNING;
    control_data.communication_thread_status = THREAD_NOT_RUNNING;

    // Reset the missed message count
    control_data.missed_sensor_update = 0;

    // Set test to not running at startup
    control_data.test_running = false;
}

void CONTROL_Tasks(void) {
    switch (control_data.state) {
        case CONTROL_STATE_INIT:
        {
            DEBUG_MESSAGE(ERROR_CONTROL_STATE_INIT);

            // Startup delay for sensors to start
            TickType_t xLastWakeTime = xTaskGetTickCount();
            const TickType_t xFrequency = STARTUP_DELAY;
            vTaskDelayUntil(&xLastWakeTime, xFrequency);

            // Start millisecond timer
            if (xTimerStart(control_data.ms_timer, 0) != pdPASS) {
                DEBUG_MESSAGE(ERROR_CONTROL_MS_TIMER_NOT_STARTED);
                control_data.state = CONTROL_STATE_BRAKING;
                SendControlAlert(ALERT_TIMER_FAILED_TO_START);
            }

            // Start low speed motor PWM
            DRV_TMR0_Start();
            DRV_OC0_Start();

            // Set low speed to off
            SetLowSpeedOn(false);

            // Set brakes to off
            SetBrakesOn(false);
            
            SetPropulsionOn(false);
            SetDrainValveOn(false);
            control_data.state = CONTROL_STATE_WAIT_STARTUP;
            SendAlertMessage(ALERT_STARTUP_INIT);
            break;
        }
        case CONTROL_STATE_WAIT_STARTUP:
        {
            DEBUG_MESSAGE(ERROR_CONTROL_STATE_WAIT_STARTUP);
            if (xQueueReceive(control_data.receiveQ, &control_data.receive_message, portMAX_DELAY)) {
                // Set thread status
                switch (control_data.receive_message.type) {
                    case AHRS_START_MESSAGE:
                    {
                        // Unpack the ahrs offset values
                        ReceiveAhrsOffset(&control_data.receive_message.data[0]);
                        control_data.ahrs_thread_status = THREAD_RUNNING;
                        SendAlertMessage(ALERT_AHRS_READY);
                        break;
                    }
                    case I2C_START_MESSAGE:
                    {
                        control_data.i2c_thread_status = THREAD_RUNNING;
                        SendAlertMessage(ALERT_I2C_READY);

                        break;
                    }
                    case COMMUNICATION_START_MESSAGE:
                    {
                        control_data.communication_thread_status = THREAD_RUNNING;
                        SendAlertMessage(ALERT_COMMUNICATION_READY);
                        break;
                    }
                }

                // If all states are running the pod is ready
                if (control_data.ahrs_thread_status == THREAD_RUNNING && control_data.i2c_thread_status == THREAD_RUNNING && control_data.communication_thread_status == THREAD_RUNNING) {
                    // Start the message timer
                    if (xTimerStart(control_data.message_timer, 0) == pdPASS) {
                        control_data.state = CONTROL_STATE_IDLE;
                        SendAlertMessage(ALERT_CONTROL_READY);
                    } else {
                        DEBUG_MESSAGE(ERROR_CONTROL_MESSAGE_TIMER_NOT_STARTED);
                        control_data.state = CONTROL_STATE_ERROR;
                        SendControlAlert(ALERT_TIMER_FAILED_TO_START);
                    }
                }
            }
            break;
        }
        case CONTROL_STATE_IDLE:
        {
            DEBUG_MESSAGE(ERROR_CONTROL_STATE_IDLE);
            if (xQueueReceive(control_data.receiveQ, &control_data.receive_message, portMAX_DELAY)) {
                switch (control_data.receive_message.type) {
                    case AHRS_DATA_MESSAGE:
                    {
                        ReceiveAhrsData(&control_data.receive_message.data[0]);
                        control_data.received_ahrs_data = true;
                        break;
                    }
                    case I2C_DATA_MESSAGE:
                    {
                        ReceiveI2cData(&control_data.receive_message.data[0]);
                        control_data.received_i2c_data = true;
                        break;
                    }
                    case PHOTOELECTRIC_STATE_MESSAGE:
                    {
                        ReceivePhotoelectric(control_data.receive_message.data[0]);
                        break;
                    }
                    case PUSHER_STATE_MESSAGE:
                    {
                        ChangePusherState();
                        break;
                    }
                    case COMMUNICATION_READ:
                    {
                        switch (control_data.receive_message.data[0]) {
                            case COMMAND_SET_BRAKES_OFF:
                            {
                                SetBrakesOn(false);
                                break;
                            }
                            case COMMAND_SET_BRAKES_ON:
                            {
                                SetBrakesOn(true);
                                break;
                            }
                            case COMMAND_SET_LOW_SPEED_OFF:
                            {
                                SetLowSpeedOn(false);
                                break;
                            }
                            case COMMAND_SET_LOW_SPEED_ON:
                            {
                                SetLowSpeedOn(true);
                                break;
                            }
                            case COMMAND_SET_PROPULSION_ON:
                            {
                                SetPropulsionOn(true);
                            }
                            case COMMAND_SET_PROPULSION_OFF:
                            {
                                SetPropulsionOn(false);
                            }
                            case COMMAND_START_TEST:
                            {
                                if (CheckBoundConditionsAnalog() == true && control_data.telemetry.ahrs_x_acc < ACCELERATION_START_THRESHOLD && control_data.telemetry.pusher_status == PUSHER_STATUS_ATTACHED && control_data.telemetry.low_speed_status == LOW_SPEED_OFF && control_data.telemetry.brake_status == BRAKE_STATUS_OFF) {
                                    control_data.state = CONTROL_STATE_CONFIRM_START;
                                    SendAlertMessage(ALERT_START_REQUEST_CONFIRM);
                                } else {
                                    SendAlertMessage(ALERT_START_FAILED_OUT_OF_BOUNDS);
                                }
                                break;
                            }
                            case COMMAND_MESSAGE_ACK:
                            {
                                control_data.received_message_ack = true;
                                break;
                            }
                        }
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
            break;
        }
        case CONTROL_STATE_CONFIRM_START:
        {
            DEBUG_MESSAGE(ERROR_CONTROL_STATE_CONFIRM_START);
            if (xQueueReceive(control_data.receiveQ, &control_data.receive_message, portMAX_DELAY)) {
                switch (control_data.receive_message.type) {
                    case AHRS_DATA_MESSAGE:
                    {
                        ReceiveAhrsData(&control_data.receive_message.data[0]);
                        control_data.received_ahrs_data = true;
                        break;
                    }
                    case I2C_DATA_MESSAGE:
                    {
                        ReceiveI2cData(&control_data.receive_message.data[0]);
                        control_data.received_i2c_data = true;
                        break;
                    }
                    case PHOTOELECTRIC_STATE_MESSAGE:
                    {
                        ReceivePhotoelectric(control_data.receive_message.data[0]);
                        break;
                    }
                    case PUSHER_STATE_MESSAGE:
                    {
                        ChangePusherState();
                        break;
                    }
                    case COMMUNICATION_READ:
                    {
                        switch (control_data.receive_message.data[0]) {
                            case COMMAND_CONFIRM_START_TEST:
                            {
                                // Set brakes to off
                                SetBrakesOn(false);

                                // Set low speed off
                                SetLowSpeedOn(false);

                                if (CheckBoundConditionsAnalog() == true && control_data.telemetry.pusher_status == PUSHER_STATUS_ATTACHED && control_data.telemetry.brake_status == BRAKE_STATUS_OFF && control_data.telemetry.low_speed_status == LOW_SPEED_OFF) {
                                    ResetTelemetry();
                                    control_data.state = CONTROL_STATE_START_IDLE;
                                    control_data.acceleration_start_count = 0;
                                    SendAlertMessage(ALERT_POD_READY_TO_PUSH);
                                } else {
                                    control_data.state = CONTROL_STATE_IDLE;
                                    SendAlertMessage(ALERT_START_FAILED_OUT_OF_BOUNDS);
                                }
                                break;
                            }
                            case COMMAND_DECLINE_START_TEST:
                            {
                                control_data.state = CONTROL_STATE_IDLE;
                                break;
                            }
                            case COMMAND_MESSAGE_ACK:
                            {
                                control_data.received_message_ack = true;
                                break;
                            }
                        }
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
            break;
        }
        case CONTROL_STATE_START_IDLE:
        {
            DEBUG_MESSAGE(ERROR_CONTROL_STATE_START_IDLE);
            if (xQueueReceive(control_data.receiveQ, &control_data.receive_message, portMAX_DELAY)) {
                switch (control_data.receive_message.type) {
                    case AHRS_DATA_MESSAGE:
                    {
                        ReceiveAhrsData(&control_data.receive_message.data[0]);
                        control_data.received_ahrs_data = true;
                        if (control_data.telemetry.ahrs_x_acc > ACCELERATION_START_THRESHOLD) {
                            control_data.acceleration_start_count++;
                            if (control_data.acceleration_start_count > ACCELERATION_START_COUNT) {
                                control_data.telemetry.test_run_time = 0;
                                control_data.test_running = true;

                                if (xTimerStart(control_data.braking_start_timer, 0) == pdPASS) {
                                    control_data.state = CONTROL_STATE_ACCELERATION;
                                    SendAlertMessage(ALERT_POD_ACCELERATING);
                                } else {
                                    SendControlAlert(ALERT_TIMER_FAILED_TO_START);
                                }
                            }
                        } else {
                            control_data.acceleration_start_count = 0;
                        }
                        break;
                    }
                    case I2C_DATA_MESSAGE:
                    {
                        ReceiveI2cData(&control_data.receive_message.data[0]);
                        control_data.received_i2c_data = true;
                        if (!CheckBoundConditionsAnalog()) {
                            SendControlAlert(ALERT_I2C_OUT_OF_BOUNDS);
                        }
                        break;
                    }
                    case PHOTOELECTRIC_STATE_MESSAGE:
                    {
                        ReceivePhotoelectric(control_data.receive_message.data[0]);
                        if (control_data.telemetry.strip_count > 0) {
                            SendControlAlert(ALERT_PHOTOELECTRIC_ERROR);
                        }
                        break;
                    }
                    case PUSHER_STATE_MESSAGE:
                    {
                        ChangePusherState();
                        if (control_data.telemetry.pusher_status == PUSHER_STATUS_DETACHED) {;
                            SendControlAlert(ALERT_PUSHER_ERROR);
                        }
                        break;
                    }
                    case COMMUNICATION_READ:
                    {
                        switch (control_data.receive_message.data[0]) {
                            case COMMAND_ABORT_START:
                            {
                                control_data.state = CONTROL_STATE_IDLE;
                                break;
                            }
                            case COMMAND_MESSAGE_ACK:
                            {
                                control_data.received_message_ack = true;
                                break;
                            }
                        }
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
            break;
        }
        case CONTROL_STATE_ACCELERATION:
        {
            DEBUG_MESSAGE(ERROR_CONTROL_STATE_ACCELERATION);
            if (xQueueReceive(control_data.receiveQ, &control_data.receive_message, portMAX_DELAY)) {
                switch (control_data.receive_message.type) {
                    case AHRS_DATA_MESSAGE:
                    {
                        ReceiveAhrsData(&control_data.receive_message.data[0]);
                        control_data.received_ahrs_data = true;
                        break;
                    }
                    case I2C_DATA_MESSAGE:
                    {
                        ReceiveI2cData(&control_data.receive_message.data[0]);
                        control_data.received_i2c_data = true;
                        if (!CheckBoundConditionsAnalog()) {
                            control_data.state = CONTROL_STATE_BRAKING;
                            SendControlAlert(ALERT_I2C_OUT_OF_BOUNDS);
                        }
                        break;
                    }
                    case PHOTOELECTRIC_STATE_MESSAGE:
                    {
                        ReceivePhotoelectric(control_data.receive_message.data[0]);
                        if (control_data.telemetry.strip_count >= MAX_BRAKING_DISTANCE) {
                            control_data.state = CONTROL_STATE_BRAKING;
                            SendControlAlert(ALERT_MAX_BRAKING_DISTANCE_PASSED);
                        }
                        break;
                    }
                    case PUSHER_STATE_MESSAGE:
                    {
                        ChangePusherState();
                        if (control_data.telemetry.pusher_status == PUSHER_STATUS_DETACHED) {
                            control_data.state = CONTROL_STATE_BRAKING;
                            SendControlAlert(ALERT_PUSH_STATE_CHANGE);
                        }
                        break;
                    }
                    case COMMUNICATION_READ:
                    {
                        switch (control_data.receive_message.data[0]) {
                            case COMMAND_EMERGENCY_STOP:
                            {
                                control_data.state = CONTROL_STATE_BRAKING;
                                SendControlAlert(ALERT_EMERGENCY_STOP_COMMAND_RECEIVED);
                                break;
                            }
                            case COMMAND_MESSAGE_ACK:
                            {
                                control_data.received_message_ack = true;
                                break;
                            }
                        }
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
            break;
        }
        case CONTROL_STATE_PROPULSION:
        {
            DEBUG_MESSAGE(ERROR_CONTROL_STATE_PROPULSION);
            if (control_data.telemetry.test_run_time >= MIN_PROPULSION_TIME && control_data.telemetry.strip_count >= MIN_PROPULSION_DISTANCE && control_data.telemetry.pusher_status == PUSHER_STATUS_DETACHED) {
                SetPropulsionOn(true);
            }
            break;
        }
        case CONTROL_STATE_BRAKING:
        {
            DEBUG_MESSAGE(ERROR_CONTROL_STATE_BRAKING);
            if (control_data.telemetry.test_run_time >= MIN_BRAKING_TIME && control_data.telemetry.strip_count >= MIN_BRAKING_DISTANCE && control_data.telemetry.pusher_status == PUSHER_STATUS_DETACHED) {
                SetBrakesOn(true);
            }
            if (xQueueReceive(control_data.receiveQ, &control_data.receive_message, portMAX_DELAY)) {
                switch (control_data.receive_message.type) {
                    case AHRS_DATA_MESSAGE:
                    {
                        ReceiveAhrsData(&control_data.receive_message.data[0]);
                        control_data.received_ahrs_data = true;
                        if (control_data.telemetry.ahrs_x_acc < BRAKING_COMPLETE_THRESHOLD) {
                            control_data.braking_stop_count++;
                            if (control_data.braking_stop_count >= BRAKING_COMPLETE_COUNT) {
                                control_data.state = CONTROL_STATE_POST_IDLE;
                            }
                        } else {
                            control_data.braking_stop_count = 0;
                        }
                        break;
                    }
                    case I2C_DATA_MESSAGE:
                    {
                        ReceiveI2cData(&control_data.receive_message.data[0]);
                        control_data.received_i2c_data = true;
                        if (!CheckBoundConditionsAnalog()) {
                            SendControlAlert(ALERT_I2C_OUT_OF_BOUNDS);
                        }
                        break;
                    }
                    case PHOTOELECTRIC_STATE_MESSAGE:
                    {
                        ReceivePhotoelectric(control_data.receive_message.data[0]);
                        if (control_data.telemetry.strip_count >= MAX_BRAKING_DISTANCE) {
                            SendControlAlert(ALERT_MAX_BRAKING_DISTANCE_PASSED);
                        }
                        break;
                    }                    
                    case PUSHER_STATE_MESSAGE:
                    {
                        ChangePusherState();
                        if (control_data.telemetry.pusher_status == PUSHER_STATUS_DETACHED) {
                            control_data.state = CONTROL_STATE_BRAKING;
                            SendControlAlert(ALERT_PUSH_STATE_CHANGE);
                        }
                        break;
                    }
                    case COMMUNICATION_READ:
                    {
                        switch (control_data.receive_message.data[0]) {
                            case COMMAND_MESSAGE_ACK:
                            {
                                control_data.received_message_ack = true;
                                break;
                            }
                        }
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
            break;
        }
        case CONTROL_STATE_POST_IDLE:
        {
            DEBUG_MESSAGE(ERROR_CONTROL_STATE_POST_IDLE);
            if (xQueueReceive(control_data.receiveQ, &control_data.receive_message, portMAX_DELAY)) {
                switch (control_data.receive_message.type) {
                    case AHRS_DATA_MESSAGE:
                    {
                        ReceiveAhrsData(&control_data.receive_message.data[0]);
                        control_data.received_ahrs_data = true;
                        break;
                    }
                    case I2C_DATA_MESSAGE:
                    {
                        ReceiveI2cData(&control_data.receive_message.data[0]);
                        control_data.received_i2c_data = true;
                        break;
                    }
                    case PHOTOELECTRIC_STATE_MESSAGE:
                    {
                        ReceivePhotoelectric(control_data.receive_message.data[0]);
                        break;
                    }
                    case PUSHER_STATE_MESSAGE:
                    {
                        ChangePusherState();
                        break;
                    }
                    case COMMUNICATION_READ:
                    {
                        switch (control_data.receive_message.data[0]) {
                            case COMMAND_SET_STATE_IDLE:
                            {
                                control_data.state = CONTROL_STATE_IDLE;
                                break;
                            }
                            case COMMAND_MESSAGE_ACK:
                            {
                                control_data.received_message_ack = true;
                                break;
                            }
                        }
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
            break;
        }
        case CONTROL_STATE_ERROR:
        {
            // Do nothing
            break;
        }
        default:
        {
            break;
        }
    }
}

void SendToControl(QUEUE_MESSAGE* message) {
    DEBUG_MESSAGE(ERROR_CONTROL_RECEIVE_MESSAGE);
    if (xQueueSendToBack(control_data.receiveQ, message, 1) != pdPASS) {
        DEBUG_MESSAGE(ERROR_CONTROL_FAILED_TO_QUEUE);
        if (control_data.state == CONTROL_STATE_ACCELERATION) {
            control_data.state = CONTROL_STATE_BRAKING;
        }
        SendControlAlert(ALERT_CONTROL_FAILED_TO_QUEUE);
    }
}

void SendToControlIsr(QUEUE_MESSAGE* message) {
    DEBUG_MESSAGE(ERROR_CONTROL_RECEIVE_MESSAGE);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendToBackFromISR(control_data.receiveQ, message, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        taskYIELD();
    }
}

void ReceiveAhrsOffset(uint8_t* data) {
    control_data.ahrs_x_offset = (int16_t) (data[0] | ((int8_t) data[1]) << 8);
    control_data.ahrs_y_offset = (int16_t) (data[2] | ((int8_t) data[3]) << 8);
    control_data.ahrs_z_offset = (int16_t) (data[4] | ((int8_t) data[5]) << 8);
}

void ReceiveAhrsData(uint8_t* data) {
    control_data.telemetry.ahrs_pitch = ((int16_t) (data[6] | ((int8_t) data[7]) << 8));
    control_data.telemetry.ahrs_roll = ((int16_t) (data[8] | ((int8_t) data[9]) << 8));

    control_data.ahrs_x_acc = (((int16_t) (data[0] | ((int8_t) data[1]) << 8)) - control_data.ahrs_x_offset) / ACCELERATION_PRECISION;
    control_data.ahrs_y_acc = (((int16_t) (data[2] | ((int8_t) data[3]) << 8)) - control_data.ahrs_y_offset) / ACCELERATION_PRECISION;
    control_data.ahrs_z_acc = (((int16_t) (data[4] | ((int8_t) data[5]) << 8)) - control_data.ahrs_z_offset) / ACCELERATION_PRECISION;

    /*control_data.ahrs_x_acc *= 386.088583;
    control_data.ahrs_y_acc *= 386.088583;
    control_data.ahrs_z_acc *= 386.088583;*/

    control_data.ahrs_x_vel += control_data.ahrs_x_acc / AHRS_DELTA_T;
    control_data.ahrs_y_vel += control_data.ahrs_y_acc / AHRS_DELTA_T;
    control_data.ahrs_z_vel += control_data.ahrs_z_acc / AHRS_DELTA_T;

    control_data.ahrs_x_pos += control_data.ahrs_x_vel / AHRS_DELTA_T;
    control_data.ahrs_y_pos += control_data.ahrs_y_vel / AHRS_DELTA_T;
    control_data.ahrs_z_pos += control_data.ahrs_z_vel / AHRS_DELTA_T;

    control_data.telemetry.ahrs_x_acc = (int16_t) control_data.ahrs_x_acc;
    control_data.telemetry.ahrs_y_acc = (int16_t) control_data.ahrs_y_acc;
    control_data.telemetry.ahrs_z_acc = (int16_t) control_data.ahrs_z_acc;
    control_data.telemetry.ahrs_x_vel = (int16_t) control_data.ahrs_x_vel;
    control_data.telemetry.ahrs_y_vel = (int16_t) control_data.ahrs_y_vel;
    control_data.telemetry.ahrs_z_vel = (int16_t) control_data.ahrs_z_vel;
    control_data.telemetry.ahrs_x_pos = (int16_t) control_data.ahrs_x_pos;
    control_data.telemetry.ahrs_y_pos = (int16_t) control_data.ahrs_y_pos;
    control_data.telemetry.ahrs_z_pos = (int16_t) control_data.ahrs_z_pos;
}

void ReceiveI2cData(uint8_t* data) {
    control_data.telemetry.analog_sensor_0 = ((uint16_t) (data[0] << 8 | ((uint16_t) data[1]))) >> 4;
    control_data.telemetry.analog_sensor_1 = ((uint16_t) (data[2] << 8 | ((uint16_t) data[3]))) >> 4;
    control_data.telemetry.front_right_distance = (((uint16_t) (data[4] << 8 | ((uint16_t) data[5]))) >> 4) * LEVITATION_CONVERSION_MULTIPLIER + LEVITATION_CONVERSION_OFFSET;
    control_data.telemetry.front_lateral_distance = (((uint16_t) (data[6] << 8 | ((uint16_t) data[7]))) >> 4) * LATERAL_CONVERSION_MULTIPLIER + LATERAL_CONVERSTION_OFFSET;
    control_data.telemetry.front_left_distance = (((uint16_t) (data[8] << 8 | ((uint16_t) data[9]))) >> 4) * LEVITATION_CONVERSION_MULTIPLIER + LEVITATION_CONVERSION_OFFSET;
    control_data.telemetry.back_right_distance = (((uint16_t) (data[10] << 8 | ((uint16_t) data[11]))) >> 4) * LEVITATION_CONVERSION_MULTIPLIER + LEVITATION_CONVERSION_OFFSET;
    control_data.telemetry.back_lateral_distance = (((uint16_t) (data[12] << 8 | ((uint16_t) data[13]))) >> 4) * LATERAL_CONVERSION_MULTIPLIER + LATERAL_CONVERSTION_OFFSET;
    control_data.telemetry.back_left_distance = (((uint16_t) (data[14] << 8 | ((uint16_t) data[15]))) >> 4) * LEVITATION_CONVERSION_MULTIPLIER + LEVITATION_CONVERSION_OFFSET;
    control_data.telemetry.high_side_pressure = ((float) (((uint16_t) (data[16] << 8 | ((uint16_t) data[17]))) >> 4)) * PRESSURE_HIGH_SIDE_CONVERSION_MULTIPLIER + PRESSURE_HIGH_SIDE_CONVERSION_OFFSET;
    control_data.telemetry.low_side_pressure = ((float) (((uint16_t) (data[18] << 8 | ((uint16_t) data[19]))) >> 4)) * PRESSURE_LOW_SIDE_CONVERSION_MULTIPLIER + PRESSURE_LOW_SIDE_CONVERSION_OFFSET;
    control_data.telemetry.analog_sensor_10 = ((uint16_t) (data[20] << 8 | ((uint16_t) data[21]))) >> 4;
    control_data.telemetry.analog_sensor_11 = GetTemp(((uint16_t) (data[22] << 8 | ((uint16_t) data[23]))) >> 4);
    control_data.telemetry.analog_sensor_12 = GetTemp(((uint16_t) (data[24] << 8 | ((uint16_t) data[25]))) >> 4);
    control_data.telemetry.analog_sensor_13 = GetTemp(((uint16_t) (data[26] << 8 | ((uint16_t) data[27]))) >> 4);
    control_data.telemetry.analog_sensor_14 = GetTemp(((uint16_t) (data[28] << 8 | ((uint16_t) data[29]))) >> 4);
    control_data.telemetry.analog_sensor_15 = GetTemp(((uint16_t) (data[30] << 8 | ((uint16_t) data[31]))) >> 4);
    control_data.telemetry.analog_sensor_16 = ((uint16_t) (data[32] << 8 | ((uint16_t) data[33]))) >> 4;
    control_data.telemetry.analog_sensor_17 = ((uint16_t) (data[34] << 8 | ((uint16_t) data[35]))) >> 4;
    control_data.telemetry.analog_sensor_18 = ((uint16_t) (data[36] << 8 | ((uint16_t) data[37]))) >> 4;
    control_data.telemetry.analog_sensor_19 = ((uint16_t) (data[38] << 8 | ((uint16_t) data[39]))) >> 4;
    control_data.telemetry.analog_sensor_20 = ((uint16_t) (data[40] << 8 | ((uint16_t) data[41]))) >> 4;
    control_data.telemetry.analog_sensor_21 = ((uint16_t) (data[42] << 8 | ((uint16_t) data[43]))) >> 4;
    control_data.telemetry.analog_sensor_22 = ((uint16_t) (data[44] << 8 | ((uint16_t) data[45]))) >> 4;
    control_data.telemetry.analog_sensor_23 = (((uint16_t) (data[46] << 8 | ((uint16_t) data[47]))) >> 4) * 1425;
    control_data.telemetry.analog_sensor_24 = ((uint16_t) (data[48] << 8 | ((uint16_t) data[49]))) >> 4; 
    control_data.telemetry.current_sensor = ((uint16_t) (data[50] << 8 | ((uint16_t) data[51]))) >> 4;//((((float) (((uint16_t) (data[50] << 8 | ((uint16_t) data[51]))) >> 4)) / 4095.0 * 5.12 * 1000.0) - 2500.0) / 28.0;
    control_data.telemetry.analog_sensor_26 = (((uint16_t) (data[52] << 8 | ((uint16_t) data[53]))) >> 4); 
    control_data.telemetry.analog_sensor_27 = GetTemp(((uint16_t) (data[54] << 8 | ((uint16_t) data[55]))) >> 4);
    control_data.telemetry.analog_sensor_28 = GetTemp(((uint16_t) (data[56] << 8 | ((uint16_t) data[57]))) >> 4);
    control_data.telemetry.analog_sensor_29 = GetTemp(((uint16_t) (data[58] << 8 | ((uint16_t) data[59]))) >> 4);
    control_data.telemetry.analog_sensor_30 = GetTemp(((uint16_t) (data[60] << 8 | ((uint16_t) data[61]))) >> 4);
    control_data.telemetry.analog_sensor_31 = GetTemp(((uint16_t) (data[62] << 8 | ((uint16_t) data[63]))) >> 4);
}

void SendControlData(TimerHandle_t handle) {
    if (control_data.state != CONTROL_STATE_IDLE || (control_data.received_ahrs_data == true && control_data.received_i2c_data == true)) {
        control_data.missed_sensor_update = 0;
        if (control_data.received_message_ack == true) {
            control_data.missed_message_ack = 0;
        } else {
            control_data.missed_message_ack++;
            if (control_data.missed_message_ack >= MAX_MISSED_MESSAGE_ACK) {
                DEBUG_MESSAGE(ERROR_CONTROL_MISSED_MESSAGE_ACK);
                if (control_data.state == CONTROL_STATE_ACCELERATION) {
                    control_data.state = CONTROL_STATE_BRAKING;
                }
                SendControlAlert(ALERT_MESSAGE_ACK_NOT_RECEIVED);
            }
        }
    } else {
        control_data.missed_sensor_update++;
        if (control_data.missed_sensor_update >= 3) {
            DEBUG_MESSAGE(ERROR_CONTROL_DATA_MISSED_UPDATE);
            if (control_data.state == CONTROL_STATE_ACCELERATION) {
                control_data.state = CONTROL_STATE_BRAKING;
            }
            SendControlAlert(ALERT_SENSORS_NOT_UPDATED);
        }
    }

    control_data.received_ahrs_data = false;
    control_data.received_i2c_data = false;
    control_data.received_message_ack = false;

    control_data.telemetry.pod_run_time = control_data.current_time_ms;
    control_data.telemetry.pod_state = control_data.state;

    control_data.data_message.type = CONTROL_DATA_MESSAGE;
    control_data.data_message.size = sizeof (control_data.telemetry);
    control_data.data_message.data = &control_data.telemetry;

    SendToCommunication(&control_data.data_message);
}

void SendControlAlert(ALERT_MESSAGE alert) {
    control_data.alert[0] = alert;
    QUEUE_MESSAGE message;
    message.type = CONTROL_ALERT_MESSAGE;
    message.size = sizeof (control_data.alert);
    message.data = &control_data.alert[0];
    //SendToCommunication(&message);
}

void ResetTelemetry(void) {
    control_data.telemetry.ahrs_x_acc = 0;
    control_data.telemetry.ahrs_y_acc = 0;
    control_data.telemetry.ahrs_z_acc = 0;
    control_data.telemetry.ahrs_x_vel = 0;
    control_data.telemetry.ahrs_y_vel = 0;
    control_data.telemetry.ahrs_z_vel = 0;
    control_data.telemetry.ahrs_x_pos = 0;
    control_data.telemetry.ahrs_y_pos = 0;
    control_data.telemetry.ahrs_z_pos = 0;
    control_data.telemetry.strip_count = 0;
    control_data.telemetry.test_run_time = 0;
    control_data.telemetry.last_strip_time = 0;
}

bool CheckBoundConditionsAnalog(void) {
    if (control_data.telemetry.analog_sensor_0 < MIN_ANALOG_SENSOR_0 || control_data.telemetry.analog_sensor_0 > MAX_ANALOG_SENSOR_0) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_1 < MIN_ANALOG_SENSOR_1 || control_data.telemetry.analog_sensor_1 > MAX_ANALOG_SENSOR_1) {
        return false;
    }
    if (control_data.telemetry.front_right_distance < MIN_FRONT_RIGHT_DISTANCE || control_data.telemetry.front_right_distance > MAX_FRONT_RIGHT_DISTANCE) {
        return false;
    }
    if (control_data.telemetry.front_lateral_distance < MIN_FRONT_LATERAL_DISTANCE || control_data.telemetry.front_lateral_distance > MAX_FRONT_LATERAL_DISTANCE) {
        return false;
    }
    if (control_data.telemetry.front_left_distance < MIN_FRONT_LEFT_DISTANCE || control_data.telemetry.front_left_distance > MAX_FRONT_LEFT_DISTANCE) {
        return false;
    }
    if (control_data.telemetry.back_right_distance < MIN_BACK_RIGHT_DISTANCE || control_data.telemetry.back_right_distance > MAX_BACK_RIGHT_DISTANCE) {
        return false;
    }
    if (control_data.telemetry.back_lateral_distance < MIN_BACK_LATERAL_DISTANCE || control_data.telemetry.back_lateral_distance > MAX_BACK_LATERAL_DISTANCE) {
        return false;
    }
    if (control_data.telemetry.back_left_distance < MIN_BACK_LEFT_DISTANCE || control_data.telemetry.back_left_distance > MAX_BACK_LEFT_DISTANCE) {
        return false;
    }
    if (control_data.telemetry.high_side_pressure < MIN_HIGH_SIDE_PRESSURE || control_data.telemetry.high_side_pressure > MAX_HIGH_SIDE_PRESSURE) {
        return false;
    }
    if (control_data.telemetry.low_side_pressure < MIN_LOW_SIDE_PRESSURE || control_data.telemetry.low_side_pressure > MAX_LOW_SIDE_PRESSURE) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_10 < MIN_ANALOG_SENSOR_10 || control_data.telemetry.analog_sensor_10 > MAX_ANALOG_SENSOR_10) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_11 < MIN_ANALOG_SENSOR_11 || control_data.telemetry.analog_sensor_11 > MAX_ANALOG_SENSOR_11) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_12 < MIN_ANALOG_SENSOR_12 || control_data.telemetry.analog_sensor_12 > MAX_ANALOG_SENSOR_12) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_13 < MIN_ANALOG_SENSOR_13 || control_data.telemetry.analog_sensor_13 > MAX_ANALOG_SENSOR_13) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_14 < MIN_ANALOG_SENSOR_14 || control_data.telemetry.analog_sensor_14 > MAX_ANALOG_SENSOR_14) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_15 < MIN_ANALOG_SENSOR_15 || control_data.telemetry.analog_sensor_15 > MAX_ANALOG_SENSOR_15) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_16 < MIN_ANALOG_SENSOR_16 || control_data.telemetry.analog_sensor_16 > MAX_ANALOG_SENSOR_16) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_17 < MIN_ANALOG_SENSOR_17 || control_data.telemetry.analog_sensor_17 > MAX_ANALOG_SENSOR_17) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_18 < MIN_ANALOG_SENSOR_18 || control_data.telemetry.analog_sensor_18 > MAX_ANALOG_SENSOR_18) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_19 < MIN_ANALOG_SENSOR_19 || control_data.telemetry.analog_sensor_19 > MAX_ANALOG_SENSOR_19) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_20 < MIN_ANALOG_SENSOR_20 || control_data.telemetry.analog_sensor_20 > MAX_ANALOG_SENSOR_20) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_21 < MIN_ANALOG_SENSOR_21 || control_data.telemetry.analog_sensor_21 > MAX_ANALOG_SENSOR_21) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_22 < MIN_ANALOG_SENSOR_22 || control_data.telemetry.analog_sensor_22 > MAX_ANALOG_SENSOR_22) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_23 < MIN_ANALOG_SENSOR_23 || control_data.telemetry.analog_sensor_23 > MAX_ANALOG_SENSOR_23) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_24 < MIN_CURRENT || control_data.telemetry.analog_sensor_24 > MAX_CURRENT) {
        return false;
    }
    if (control_data.telemetry.current_sensor < MIN_ANALOG_SENSOR_25 || control_data.telemetry.current_sensor > MAX_ANALOG_SENSOR_25) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_26 < MIN_ANALOG_SENSOR_26 || control_data.telemetry.analog_sensor_26 > MAX_ANALOG_SENSOR_26) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_27 < MIN_ANALOG_SENSOR_27 || control_data.telemetry.analog_sensor_27 > MAX_ANALOG_SENSOR_27) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_28 < MIN_ANALOG_SENSOR_28 || control_data.telemetry.analog_sensor_28 > MAX_ANALOG_SENSOR_28) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_29 < MIN_ANALOG_SENSOR_29 || control_data.telemetry.analog_sensor_29 > MAX_ANALOG_SENSOR_29) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_30 < MIN_ANALOG_SENSOR_30 || control_data.telemetry.analog_sensor_30 > MAX_ANALOG_SENSOR_30) {
        return false;
    }
    if (control_data.telemetry.analog_sensor_31 < MIN_ANALOG_SENSOR_31 || control_data.telemetry.analog_sensor_31 > MAX_ANALOG_SENSOR_31) {
        return false;
    }
    return true;
}

void SendAlertMessage(ALERT_MESSAGE data) {
    control_data.status_message[0] = data;

    control_data.debug_message.type = CONTROL_ALERT_MESSAGE;
    control_data.debug_message.size = 1;
    control_data.debug_message.data = &control_data.status_message[0];

    //SendToCommunication(&control_data.debug_message);
}

void ReceivePhotoelectric(uint8_t data) {
    switch (data) {
        case 0x00:
        {
            if (control_data.photoelectric_0 == PHOTOELECTRIC_STATUS_ON) {
                control_data.photoelectric_0 = PHOTOELECTRIC_STATUS_OFF;
                SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE0, INT_EDGE_TRIGGER_RISING);
            } else {
                control_data.photoelectric_0 = PHOTOELECTRIC_STATUS_ON;
                SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE0, INT_EDGE_TRIGGER_FALLING);
            }
            break;
        }
        case 0x01:
        {
            if (control_data.photoelectric_1 == PHOTOELECTRIC_STATUS_ON) {
                control_data.photoelectric_1 = PHOTOELECTRIC_STATUS_OFF;
                SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE1, INT_EDGE_TRIGGER_RISING);
            } else {
                control_data.photoelectric_1 = PHOTOELECTRIC_STATUS_ON;
                SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE1, INT_EDGE_TRIGGER_FALLING);
            }
            break;
        }
        case 0x02:
        {
            if (control_data.photoelectric_2 == PHOTOELECTRIC_STATUS_ON) {
                control_data.photoelectric_2 = PHOTOELECTRIC_STATUS_OFF;
                SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE2, INT_EDGE_TRIGGER_RISING);
            } else {
                control_data.photoelectric_2 = PHOTOELECTRIC_STATUS_ON;
                SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE2, INT_EDGE_TRIGGER_FALLING);
            }
            break;
        }
    }
    if ((control_data.photoelectric_0 == PHOTOELECTRIC_STATUS_ON && control_data.photoelectric_1 == PHOTOELECTRIC_STATUS_ON) || (control_data.photoelectric_1 == PHOTOELECTRIC_STATUS_ON && control_data.photoelectric_2 == PHOTOELECTRIC_STATUS_ON) || (control_data.photoelectric_0 == PHOTOELECTRIC_STATUS_ON && control_data.photoelectric_2 == PHOTOELECTRIC_STATUS_ON)) {
        if (control_data.current_time_ms - control_data.telemetry.last_strip_time > MIN_TIME_BETWEEN_PHOTOELECTRIC) {
            control_data.telemetry.strip_count += 1;
            control_data.telemetry.last_strip_time = control_data.current_time_ms;
        }
    }
}

void ChangePusherState() {
    if (control_data.telemetry.pusher_status == PUSHER_STATUS_ATTACHED) {
        control_data.telemetry.pusher_status = PUSHER_STATUS_DETACHED;
        SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE3, INT_EDGE_TRIGGER_RISING);
    } else {
        control_data.telemetry.pusher_status = PUSHER_STATUS_ATTACHED;
        SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE3, INT_EDGE_TRIGGER_FALLING);
    }
}

void SetPropulsionOn(bool state) {
    if (state) {
        SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_A, 1 << 9, 1 << 9);
        control_data.telemetry.propulsion_status = PROPULSION_STATUS_ON;
    } else {
        control_data.telemetry.propulsion_status = PROPULSION_STATUS_OFF;
        SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_A, 1 << 9);
    }
}
    
void SetBrakesOn(bool state) {
    if (state) {
        SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_D, 1 << 9, 1 << 9);
        control_data.telemetry.brake_status = BRAKE_STATUS_ON;
    } else {        
        SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_D, 1 << 9);
        control_data.telemetry.brake_status = BRAKE_STATUS_OFF;
    }
}

void SetLowSpeedOn(bool state) {
    if (state) {
        PLIB_OC_PulseWidth16BitSet(LOW_SPEED_OC, LOW_SPEED_OC_ON);
        control_data.telemetry.low_speed_status = LOW_SPEED_ON;
    } else {
        PLIB_OC_PulseWidth16BitSet(LOW_SPEED_OC, LOW_SPEED_OC_OFF);
        control_data.telemetry.low_speed_status = LOW_SPEED_OFF;
    }
}

void BrakingTimerCallback(TimerHandle_t handle) {
    SetBrakesOn(true);
    control_data.state = CONTROL_STATE_BRAKING;
    SendControlAlert(ALERT_MAX_BRAKING_TIME_PASSED);
}

void MilliTimerCallback(TimerHandle_t handle) {
    control_data.current_time_ms++;
    if (control_data.test_running) {
        control_data.telemetry.test_run_time++;
    }
}

uint16_t GetTemp(uint16_t voltage) {
    float temp = (3435 / (logf((float) voltage / 4095 * 2.56) + 12.794) - 273.15) * 10;
    return (uint16_t) temp;
}