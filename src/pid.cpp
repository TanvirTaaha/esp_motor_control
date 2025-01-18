#include "motor_control.h"

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

uint8_t moving = 0; // is the base in motion?

/*
 * Initialize PID variables to zero to prevent startup spikes
 * when turning PID on to start moving
 * In particular, assign both Encoder and PrevEnc the current encoder value
 * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
 * Note that the assumption here is that PID is only turned on
 * when going from stop to moving, that's why we can init everything on zero.
 */
void resetPID()
{
    leftPID.target_ticks_per_second = 0.0;
    leftPID.sensor_ticks_per_second = left_ticks_per_second();
    leftPID.prev_sensor_ticks_per_second = leftPID.sensor_ticks_per_second;
    leftPID.output = 0;
    leftPID.ITerm = 0;

    rightPID.target_ticks_per_second = 0.0;
    rightPID.sensor_ticks_per_second = right_ticks_per_second();
    rightPID.prev_sensor_ticks_per_second = 0;
    rightPID.output = 0;
    rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo *p)
{
    double Perror;
    double output;

    // Perror = p->target_ticks_per_second - (p->Encoder - p->PrevEnc);
    Perror = p->target_ticks_per_second - p->sensor_ticks_per_second;

    /*
     * Avoid derivative kick and allow tuning changes,
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
     */
    // D-term is being subtracted cause it is difference of inputs not the difference of errors
    output = (Kp * Perror - Kd * (p->sensor_ticks_per_second - p->prev_sensor_ticks_per_second) + p->ITerm) / Ko;
    // output is accumulated. reducing load on kI. reduces overshooting
    output += p->output;
    // Accumulate Integral error *or* Limit output.
    // Stop accumulating when output saturates
    if (output >= MAX_PWM)
        output = MAX_PWM;
    else if (output <= -MAX_PWM)
        output = -MAX_PWM;
    else
        /*
         * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
         */
        p->ITerm += Ki * Perror;

    p->output = output;
    p->prev_sensor_ticks_per_second = p->sensor_ticks_per_second;
}

/* Read the encoder values and call the PID routine */
void updatePID()
{
    /* Read the encoders */
    leftPID.sensor_ticks_per_second = left_ticks_per_second();
    rightPID.sensor_ticks_per_second = right_ticks_per_second();

    /* If we're not moving there is nothing more to do */
    if (!moving)
    {
        /*
         * Reset PIDs once, to prevent startup spikes,
         * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
         * prev_sensor_ticks_per_second is considered a good proxy to detect
         * whether reset has already happened
         */
        if (leftPID.prev_sensor_ticks_per_second != 0 || rightPID.prev_sensor_ticks_per_second != 0)
            resetPID();
        return;
    }

    /* Compute PID update for each motor */
    doPID(&rightPID);
    doPID(&leftPID);

    /* Set the motor speeds accordingly */
    setMotorSpeeds(leftPID.output, rightPID.output);
}

inline void set_targets(double left_target, double right_target)
{
    leftPID.target_ticks_per_second = left_target;
    rightPID.target_ticks_per_second = right_target;
}