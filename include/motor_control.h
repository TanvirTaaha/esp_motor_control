#pragma once
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__
#include "Arduino.h"
#include "driver/pcnt.h"
#include "driver/timer.h"

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
#define VELOCITY_CALC_FREQ 20.0                      // Hz. Calculate velocity every 50ms

// Macro for avoiding overflow/underflow jumps
// ((diff + L + L / 2) % L) - L / 2 : Where L is the limit
#define OVERFLOW_LIMIT 32768
#define STEPS_DIFF(diff) (((diff + ((OVERFLOW_LIMIT) | ((OVERFLOW_LIMIT) >> 1))) % (OVERFLOW_LIMIT)) - ((OVERFLOW_LIMIT) >> 1))

#define LEFT 0
#define RIGHT 1
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
extern double ros_cmd_positions[2];    // [left, right]
extern double velocity_from_sensor[2]; // [left, right]
extern unsigned long last_heard;

/* PID setpoint info For a Motor */
typedef struct
{
    double TargetTicksPerFrame; // target speed in ticks per frame
    long Encoder;               // encoder count
    long PrevEnc;               // last encoder count

    /*
     * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
     */
    int PrevInput; // last input
    // int PrevErr;                   // last error

    /*
     * Using integrated term (ITerm) instead of integrated error (Ierror),
     * to allow tuning changes,
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
     */
    // int Ierror;
    int ITerm; // integrated term

    long output; // last motor setting
} SetPointInfo;

void resetPID();
void doPID(SetPointInfo *p);
void updatePID();

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

// Serial command functions
void backgroundTask(void *pvParameters);
void send_data_to_serial(double *state_positions);
void read_data_from_serial();

template <typename T>
inline T clamp(T val, T lo, T hi);
#endif