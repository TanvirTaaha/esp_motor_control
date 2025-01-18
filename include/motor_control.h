#pragma once
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__
#include "Arduino.h"
#include "driver/pcnt.h"
#include "driver/timer.h"

#define MOTOR_CONTROL_DEBUG

// Serial params
#define MSG_LEN 16

// Define encoder pins
#define LEFT_ENC_A 34
#define LEFT_ENC_B 35
#define RIGHT_ENC_A 5
#define RIGHT_ENC_B 18

// Timer configuration
#define TIMER_DIVIDER 80                             // Hardware timer clock divider (80MHz/80 = 1MHz)
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // Convert counter value to seconds
#define VELOCITY_CALC_FREQ 50.0                      // Hz. Calculate velocity every 50ms
#define COUNTS_PER_REV 1385                          // uint: ticks

// Delays
const TickType_t SERIAL_COMM_VTASK_DELAY = pdMS_TO_TICKS(50); // unit:ms
#define AUTO_STOP_INTERVAL 2000                               // unit:ms
#define TARGET_VELOCITY_UPDATE_INTERVAL 100                   // unit:ms

// Macro for avoiding overflow/underflow jumps
// ((diff + L + L / 2) % L) - L / 2 : Where L is the limit
#define OVERFLOW_LIMIT 32768
#define STEPS_DIFF(diff) (((diff + ((OVERFLOW_LIMIT) | ((OVERFLOW_LIMIT) >> 1))) % (OVERFLOW_LIMIT)) - ((OVERFLOW_LIMIT) >> 1))

#define LEFT 0
#define RIGHT 1
#define LEFT_ENC_PCNT_UNIT PCNT_UNIT_0
#define RIGHT_ENC_PCNT_UNIT PCNT_UNIT_1
/* Maximum PWM signal */
#define MAX_PWM 1023

// Motor PIN definitions
#define MOTOR_LEFT_FWD 16  // Left motor forward
#define MOTOR_LEFT_REV 17  // Left motor reverse
#define MOTOR_RIGHT_FWD 18 // Right motor forward
#define MOTOR_RIGHT_REV 19 // Right motor reverse

// PWM configurations
#define MOTOR_FREQ 40000 // 20KHz
#define MOTOR_RES 10     // 10-bit resolution (0-1023)

// PWM channel assignments
#define PWM_LEFT_FWD 0
#define PWM_LEFT_REV 1
#define PWM_RIGHT_FWD 2
#define PWM_RIGHT_REV 3

extern char sending_data_buffer[MSG_LEN];
extern char receiving_data_buffer[MSG_LEN];
extern double ros_cmd_positions[2]; // [left, right]
extern unsigned long last_motor_command;

/* PID setpoint info For a Motor */
typedef struct
{
    double target_ticks_per_second; // target speed in ticks per frame
    double sensor_ticks_per_second; // encoder count

    /*
     * Using previous input (prev_sensor_ticks_per_second) instead of PrevError to avoid derivative kick,
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
     */
    double prev_sensor_ticks_per_second; // last input
    // int PrevErr;                   // last error

    /*
     * Using integrated term (ITerm) instead of integrated error (Ierror),
     * to allow tuning changes,
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
     */
    // int Ierror;
    double ITerm; // integrated term

    double output; // last motor setting
} SetPointInfo;

// pid.cpp
/* PID Parameters */
extern int8_t kP;
extern int8_t kD;
extern int8_t kI;
extern int8_t kO;
extern SetPointInfo leftPID, rightPID;
extern uint8_t moving;
void resetPID();
void doPID(SetPointInfo *p);
void updatePID();
// inline void set_targets(double left_target, double right_target)
// {
//     leftPID.target_ticks_per_second = left_target;
//     rightPID.target_ticks_per_second = right_target;
// }

// encoder_driver.cpp
extern volatile double left_ticks_per_sec;
extern volatile double right_ticks_per_sec;
void setup_timer();
void setup_encoder(pcnt_unit_t unit, int pinA, int pinB);
int16_t get_encoder_count(uint8_t i);
inline void reset_encoders();

// motor_driver.cpp
void initMotorDriver();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

// Serial command functions
void backgroundTask(void *pvParameters);
void send_data_to_serial();
void read_data_from_serial();
void setup_serial();

// ############################### Utility Functions ###################################
// from: https://stackoverflow.com/a/46963317/8928251
template <typename T>
inline T clamp(T val, T lo, T hi)
{
    return max(lo, min(hi, val));
}
#endif