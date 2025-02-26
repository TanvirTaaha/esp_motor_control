#pragma once
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__
#include "Arduino.h"
#include "driver/pcnt.h"
#include "driver/timer.h"

// #define ACTIVATE_LOGGING 1  // have to be before including debug.h maybe
#include "debug.h"

// Serial params
#define MSG_LEN 40  // e_00000.0_00000.0_00000.0_00000.0__\n

#define LEFT 0
#define RIGHT 1
#define LEFT_ENC_PCNT_UNIT PCNT_UNIT_0
#define RIGHT_ENC_PCNT_UNIT PCNT_UNIT_1
// Define encoder pins
#define LEFT_ENC_A 13
#define LEFT_ENC_B 14
#define RIGHT_ENC_A 26
#define RIGHT_ENC_B 27

// Timer configuration
#define TIMER_DIVIDER 80                              // Hardware timer clock divider (80MHz/80 = 1MHz)
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)  // Convert counter value to seconds
#define VELOCITY_CALC_FREQ 50.0                       // Hz. Calculate velocity every 50ms
#define COUNTS_PER_REV 1385                           // uint: ticks

// Delays
const TickType_t SERIAL_COMM_VTASK_DELAY = pdMS_TO_TICKS(30);  // unit:ms
#define AUTO_STOP_INTERVAL 2000                                // unit:ms
#define TARGET_VELOCITY_UPDATE_INTERVAL 100                    // unit:ms

// Macro for avoiding overflow/underflow jumps
// ((diff + L + L / 2) % L) - L / 2 : Where L is the limit
#define OVERFLOW_LIMIT 32768
#define STEPS_DIFF(diff) (((diff + ((OVERFLOW_LIMIT) | ((OVERFLOW_LIMIT) >> 1))) % (OVERFLOW_LIMIT)) - ((OVERFLOW_LIMIT) >> 1))

/* Maximum PWM signal */
#define MAX_PWM 1023

// Motor PIN definitions
#define MOTOR_PIN_LEFT_FWD 18   // Left motor forward
#define MOTOR_PIN_LEFT_REV 19   // Left motor reverse
#define MOTOR_PIN_RIGHT_FWD 23  // Right motor forward
#define MOTOR_PIN_RIGHT_REV 22  // Right motor reverse

// PWM configurations
#define MOTOR_FREQ 20000  // 40KHz
#define MOTOR_RES 10      // 10-bit resolution (0-1023)

// PWM channel assignments
#define PWM_CHANNEL_LEFT_FWD 0
#define PWM_CHANNEL_LEFT_REV 1
#define PWM_CHANNEL_RIGHT_FWD 2
#define PWM_CHANNEL_RIGHT_REV 3

extern char sending_data_buffer[MSG_LEN + 1];
extern char receiving_data_buffer[MSG_LEN + 1];
extern float ros_cmd_velocities[2];  // [left, right]
extern unsigned long last_motor_command;

/* PID setpoint info For a Motor */
typedef struct
{
  float target_ticks_per_second;  // target speed in ticks per frame
  float sensor_ticks_per_second;  // encoder count

  /*
   * Using previous input (prev_sensor_ticks_per_second) instead of PrevError to avoid derivative kick,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
   */
  float prev_sensor_ticks_per_second;  // last input
  // int PrevErr;                   // last error

  /*
   * Using integrated term (ITerm) instead of integrated error (Ierror),
   * to allow tuning changes,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   */
  // int Ierror;
  float ITerm;  // integrated term

  float output;  // last motor setting
} SetPointInfo;

// pid.cpp
/* PID Parameters */
extern int16_t kP;
extern int16_t kD;
extern int16_t kI;
extern int16_t kO;
extern volatile SetPointInfo leftPID, rightPID;
extern bool should_move;
void PID_init();
void resetPID();
void doPID(volatile SetPointInfo *p);
void updatePID();
// inline void set_targets(float left_target, float right_target)
// {
//     leftPID.target_ticks_per_second = left_target;
//     rightPID.target_ticks_per_second = right_target;
// }

// encoder_driver.cpp
extern volatile float left_ticks_per_sec;
extern volatile float right_ticks_per_sec;
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

// persistence.cpp
void eeprom_setup();
void eeprom_write_pid_params();
void eeprom_read_pid_params();

// ############################### Utility Functions ###################################
// from: https://stackoverflow.com/a/46963317/8928251
template <typename T>
inline T clamp(T val, T lo, T hi) {
  return max(lo, min(hi, val));
}
#endif