#ifndef _CONTROL_H
#define _CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "queue.h"
#include "queue_message.h"
#include "debug.h"
#include "communication.h"
#include "ahrs.h"
#include "timers.h"
#include "math.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

#define MESSAGE_PERIOD 100 // Message rate in milliseconds
#define AHRS_MESSAGE_RATE 1000 / MESSAGE_PERIOD // AHRS Update rate in Hz
#define I2C_MESSAGE_PERIOD MESSAGE_PERIOD // Analog sensors update rate in milliseconds
#define STARTUP_DELAY 10000 // Startup delay in ms to allow sensors to start
#define LOW_SPEED_OC 3 // ID for the low speed for the OC
#define LOW_SPEED_OC_OFF 10000 // OC pulse width for off
#define LOW_SPEED_OC_ON 12000 // OC pulse width for on
    
// ADC Conversion Values
#define LEVITATION_CONVERSION_MULTIPLIER 1.020249084
#define LEVITATION_CONVERSION_OFFSET 14000
#define LATERAL_CONVERSION_MULTIPLIER 0.765186813
#define LATERAL_CONVERSTION_OFFSET 12000
#define PRESSURE_HIGH_SIDE_CONVERSION_MULTIPLIER 0.937728938
#define PRESSURE_HIGH_SIDE_CONVERSION_OFFSET -750
#define PRESSURE_LOW_SIDE_CONVERSION_MULTIPLIER 0.062515263
#define PRESSURE_LOW_SIDE_CONVERSION_OFFSET -25
    
// AHRS bound conditions
#define ACCELERATION_PRECISION 10 // Precision of the AHRS in 1/10000 of a g
#define AHRS_DELTA_T 10 // Delta T for integration from acceleration to velocity and distance
    
// Analog sensor boundary conditions
#define MIN_ANALOG_SENSOR_0 0x0000
#define MAX_ANALOG_SENSOR_0 0xFFFF
#define MIN_ANALOG_SENSOR_1 0x0000
#define MAX_ANALOG_SENSOR_1 0xFFFF
#define MIN_FRONT_RIGHT_DISTANCE 0x0000
#define MAX_FRONT_RIGHT_DISTANCE 0xFFFF
#define MIN_FRONT_LATERAL_DISTANCE 0x0000
#define MAX_FRONT_LATERAL_DISTANCE 0xFFFF
#define MIN_FRONT_LEFT_DISTANCE 0x0000
#define MAX_FRONT_LEFT_DISTANCE 0xFFFF
#define MIN_BACK_RIGHT_DISTANCE 0x0000
#define MAX_BACK_RIGHT_DISTANCE 0xFFFF
#define MIN_BACK_LATERAL_DISTANCE 0x0000
#define MAX_BACK_LATERAL_DISTANCE 0xFFFF
#define MIN_BACK_LEFT_DISTANCE 0x0000
#define MAX_BACK_LEFT_DISTANCE 0xFFFF
#define MIN_HIGH_SIDE_PRESSURE 0x0000
#define MAX_HIGH_SIDE_PRESSURE 0xFFFF
#define MIN_LOW_SIDE_PRESSURE 0x0000
#define MAX_LOW_SIDE_PRESSURE 0xFFFF
#define MIN_ANALOG_SENSOR_10 0x0000
#define MAX_ANALOG_SENSOR_10 0xFFFF
#define MIN_ANALOG_SENSOR_11 0x0000
#define MAX_ANALOG_SENSOR_11 0xFFFF
#define MIN_ANALOG_SENSOR_12 0x0000
#define MAX_ANALOG_SENSOR_12 0xFFFF
#define MIN_ANALOG_SENSOR_13 0x0000
#define MAX_ANALOG_SENSOR_13 0xFFFF
#define MIN_ANALOG_SENSOR_14 0x0000
#define MAX_ANALOG_SENSOR_14 0xFFFF
#define MIN_ANALOG_SENSOR_15 0x0000
#define MAX_ANALOG_SENSOR_15 0xFFFF
#define MIN_ANALOG_SENSOR_16 0x0000
#define MAX_ANALOG_SENSOR_16 0xFFFF
#define MIN_ANALOG_SENSOR_17 0x0000
#define MAX_ANALOG_SENSOR_17 0xFFFF
#define MIN_ANALOG_SENSOR_18 0x0000
#define MAX_ANALOG_SENSOR_18 0xFFFF
#define MIN_ANALOG_SENSOR_19 0x0000
#define MAX_ANALOG_SENSOR_19 0xFFFF
#define MIN_ANALOG_SENSOR_20 0x0000
#define MAX_ANALOG_SENSOR_20 0xFFFF
#define MIN_ANALOG_SENSOR_21 0x0000
#define MAX_ANALOG_SENSOR_21 0xFFFF
#define MIN_ANALOG_SENSOR_22 0x0000
#define MAX_ANALOG_SENSOR_22 0xFFFF
#define MIN_ANALOG_SENSOR_23 0x0000
#define MAX_ANALOG_SENSOR_23 0xFFFF
#define MIN_CURRENT 0x0000
#define MAX_CURRENT 0xFFFF
#define MIN_ANALOG_SENSOR_25 0x0000
#define MAX_ANALOG_SENSOR_25 0xFFFF
#define MIN_ANALOG_SENSOR_26 0x0000
#define MAX_ANALOG_SENSOR_26 0xFFFF
#define MIN_ANALOG_SENSOR_27 0x0000
#define MAX_ANALOG_SENSOR_27 0xFFFF
#define MIN_ANALOG_SENSOR_28 0x0000
#define MAX_ANALOG_SENSOR_28 0xFFFF
#define MIN_ANALOG_SENSOR_29 0x0000
#define MAX_ANALOG_SENSOR_29 0xFFFF
#define MIN_ANALOG_SENSOR_30 0x0000
#define MAX_ANALOG_SENSOR_30 0xFFFF
#define MIN_ANALOG_SENSOR_31 0x0000
#define MAX_ANALOG_SENSOR_31 0xFFFF
    
// Test run conditions
#define MIN_BRAKING_TIME 10000 // min time that must pass before braking can occur in ms
#define MAX_BRAKING_TIME 30000 // max time that can pass before braking occurs in ms
#define MIN_BRAKING_DISTANCE 0 // min distance before braking can occur in 100 ft
#define NOMINAL_BRAKING_DISTANCE 52 // nominal braking distance in 100 ft
#define MAX_BRAKING_DISTANCE 52 // max distance before braking occurs in 100 ft

#define MIN_PROPULSION_DISTANCE 0 //min distance for propulsion
#define MIN_PROPULSION_TIME 5000 //

#define ACCELERATION_START_THRESHOLD 50 // acceleration indicating that the pusher is active
#define ACCELERATION_START_COUNT 1 // number of positive acceleration readings to indicate the pusher is active
#define BRAKING_COMPLETE_THRESHOLD  10 // acceleration indicating that the pod has stopped
#define BRAKING_COMPLETE_COUNT 5 // number of accelerations below threshold to indicate pod is stopped

#define MAX_MISSED_MESSAGE_ACK 3 // max number of missed message acks before stopping

#define MIN_TIME_BETWEEN_PHOTOELECTRIC 100 // min time between tape strips in ms
#define MAX_DISTANCE_DIFFERENCE 100 // max distance difference between the ahrs position and the optical markings in ft
#define OPTICAL_MARKING_DISTANCE 100 // distance between optical markings in ft
    
typedef enum
{
	CONTROL_STATE_INIT = 0x00,
    CONTROL_STATE_WAIT_STARTUP,
    CONTROL_STATE_IDLE,
    CONTROL_STATE_CONFIRM_START,
    CONTROL_STATE_START_IDLE,
    CONTROL_STATE_ACCELERATION,
    CONTROL_STATE_PROPULSION,
    CONTROL_STATE_BRAKING,
    CONTROL_STATE_POST_IDLE,
    CONTROL_STATE_ERROR
} CONTROL_STATES;

typedef enum
{
    BRAKE_STATUS_OFF = 0x00,
    BRAKE_STATUS_ON = 0x01
} BRAKE_STATUS;

typedef enum
{
    PROPULSION_STATUS_OFF = 0x00,
    PROPULSION_STATUS_ON = 0x01    
} PROPULSION_STATUS;

typedef enum
{
    PUSHER_STATUS_DETACHED = 0x00,
    PUSHER_STATUS_ATTACHED = 0x01
} PUSHER_STATUS;

typedef enum
{
    PHOTOELECTRIC_STATUS_OFF = 0x00,
    PHOTOELECTRIC_STATUS_ON = 0x01
} PHOTOELECTRIC_STATUS;

typedef enum
{
    THREAD_NOT_RUNNING = 0x00,
    THREAD_RUNNING = 0x01
} THREAD_STATUS;

typedef enum
{
    LOW_SPEED_OFF = 0x00,
    LOW_SPEED_ON = 0x01
} LOW_SPEED_STATUS;

typedef enum
{
    COMMAND_SET_BRAKES_OFF = 0x10,
    COMMAND_SET_BRAKES_ON = 0x11,
    COMMAND_SET_LOW_SPEED_OFF = 0x20,
    COMMAND_SET_LOW_SPEED_ON = 0x21,
    COMMAND_START_TEST = 0x30,
    COMMAND_CONFIRM_START_TEST = 0x30,
    COMMAND_DECLINE_START_TEST = 0x31,
    COMMAND_ABORT_START = 0x31,
    COMMAND_SET_STATE_IDLE = 0x31,
    COMMAND_MESSAGE_ACK = 0x60,
    COMMAND_EMERGENCY_STOP = 0x31,
    COMMAND_SET_PROPULSION_OFF = 0x40,
    COMMAND_SET_PROPULSION_ON = 0x41
} CONTROL_COMMANDS;

typedef struct __attribute__((__packed__))
{    
    uint8_t pod_state;
    uint8_t pusher_status;
    uint8_t brake_status;
    uint8_t low_speed_status; 
    int16_t ahrs_x_acc;
    int16_t ahrs_y_acc;
    int16_t ahrs_z_acc;
    int16_t ahrs_x_vel;
    int16_t ahrs_y_vel;
    int16_t ahrs_z_vel;
    int16_t ahrs_x_pos;
    int16_t ahrs_y_pos;
    int16_t ahrs_z_pos;
    int16_t ahrs_pitch;
    int16_t ahrs_roll;    
    uint16_t strip_count;
    uint16_t analog_sensor_0;
    uint16_t analog_sensor_1;
    uint16_t front_right_distance;
    uint16_t front_lateral_distance;
    uint16_t front_left_distance;
    uint16_t back_right_distance;
    uint16_t back_lateral_distance;
    uint16_t back_left_distance;
    uint16_t high_side_pressure;
    uint16_t low_side_pressure;
    uint16_t analog_sensor_10;
    uint16_t analog_sensor_11;
    uint16_t analog_sensor_12;
    uint16_t analog_sensor_13;
    uint16_t analog_sensor_14;
    uint16_t analog_sensor_15;
    uint16_t analog_sensor_16;
    uint16_t analog_sensor_17;
    uint16_t analog_sensor_18;
    uint16_t analog_sensor_19;
    uint16_t analog_sensor_20;
    uint16_t analog_sensor_21;
    uint16_t analog_sensor_22;
    uint16_t analog_sensor_23;
    uint16_t analog_sensor_24;
    uint16_t current_sensor;
    uint16_t analog_sensor_26;
    uint16_t analog_sensor_27;
    uint16_t analog_sensor_28;
    uint16_t analog_sensor_29;
    uint16_t analog_sensor_30;
    uint16_t analog_sensor_31;
    uint32_t pod_run_time; 
    uint32_t test_run_time;
    uint32_t last_strip_time;
} POD_TELEMETRY;

typedef struct
{
    CONTROL_STATES state;
    QueueHandle_t receiveQ;
    TimerHandle_t message_timer;
    TimerHandle_t ms_timer;
    TimerHandle_t braking_start_timer;
    QUEUE_MESSAGE receive_message;
    POD_TELEMETRY telemetry;
    uint8_t alert[1];    
    uint8_t acceleration_start_count;
    uint8_t braking_stop_count;
    uint8_t stop_count;
    
    QUEUE_MESSAGE data_message;
    QUEUE_MESSAGE debug_message;
    uint8_t status_message[1];
    
    uint32_t current_time_ms;
    
    int16_t ahrs_x_offset;
    int16_t ahrs_y_offset;
    int16_t ahrs_z_offset;
    
    THREAD_STATUS ahrs_thread_status;
    THREAD_STATUS i2c_thread_status;
    THREAD_STATUS communication_thread_status;
    
    bool received_ahrs_data;
    bool received_i2c_data;
    bool received_message_ack;
    uint8_t missed_sensor_update;
    uint8_t missed_message_ack;
    
    bool photoelectric_0;
    bool photoelectric_1;
    bool photoelectric_2;
    
    bool test_running; 
    
    float ahrs_x_acc;
    float ahrs_y_acc;
    float ahrs_z_acc;
    float ahrs_x_vel;
    float ahrs_y_vel;
    float ahrs_z_vel;
    float ahrs_x_pos;
    float ahrs_y_pos;
    float ahrs_z_pos;
} CONTROL_DATA;

void CONTROL_Initialize (void);
void CONTROL_Tasks(void);
void SendToControl(QUEUE_MESSAGE*);
void SendToControlIsr(QUEUE_MESSAGE*);
void ReceiveAhrsOffset(uint8_t*);
void ReceiveAhrsData(uint8_t*);
void ReceiveI2cData(uint8_t*);
void SendControlData(TimerHandle_t);
void SendAlertMessage(ALERT_MESSAGE);
void ResetTelemetry(void);
bool CheckBoundConditionsAnalog(void);
void ReceivePhotoelectric(uint8_t);
void ChangePusherState();
void SetPropulsionOn(bool);
void SetDrainValveOn(bool);
void SetBrakesOn(bool);
void SetLowSpeedOn(bool);
void BrakingTimerCallback(TimerHandle_t);
void MilliTimerCallback(TimerHandle_t);
uint16_t GetTemp(uint16_t);


#endif /* _CONTROL_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END