#include "motor_control.h"
void initMotorController()
{
    // Configure PWM channels
    ledcSetup(PWM_LEFT_FWD, MOTOR_FREQ, MOTOR_RES);
    ledcSetup(PWM_LEFT_REV, MOTOR_FREQ, MOTOR_RES);
    ledcSetup(PWM_RIGHT_FWD, MOTOR_FREQ, MOTOR_RES);
    ledcSetup(PWM_RIGHT_REV, MOTOR_FREQ, MOTOR_RES);

    // Attach pins to channels
    ledcAttachPin(MOTOR_LEFT_FWD, PWM_LEFT_FWD);
    ledcAttachPin(MOTOR_LEFT_REV, PWM_LEFT_REV);
    ledcAttachPin(MOTOR_RIGHT_FWD, PWM_RIGHT_FWD);
    ledcAttachPin(MOTOR_RIGHT_REV, PWM_RIGHT_REV);
}

void setMotorSpeed(int idx, int speed)
{
    speed = clamp(speed, -MAX_PWM, MAX_PWM);
    if (idx == LEFT)
    {
        ledcWrite(PWM_LEFT_FWD, abs(max(0, speed)));
        ledcWrite(PWM_LEFT_REV, abs(min(0, speed)));
    }
    else
    {
        ledcWrite(PWM_RIGHT_FWD, abs(max(0, speed)));
        ledcWrite(PWM_RIGHT_REV, abs(min(0, speed)));
    }
}
void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
}

// ############################### Utility Functions ###################################
// from: https://stackoverflow.com/a/46963317/8928251
template <typename T>
inline T clamp(T val, T lo, T hi)
{
    return max(lo, min(hi, val));
}