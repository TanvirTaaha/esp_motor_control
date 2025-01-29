#include "motor_control.h"

/* PID Parameters */
int8_t kP = 20;
int8_t kD = 12;
int8_t kI = 0;
int8_t kO = 50;

SetPointInfo leftPID, rightPID;
bool should_move = false; // is the base in motion?

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
    leftPID.sensor_ticks_per_second = left_ticks_per_sec;
    leftPID.prev_sensor_ticks_per_second = leftPID.sensor_ticks_per_second;
    leftPID.output = 0;
    leftPID.ITerm = 0;

    rightPID.target_ticks_per_second = 0.0;
    rightPID.sensor_ticks_per_second = right_ticks_per_sec;
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
    // LOG_DEBUG("Perror:%lf", Perror);
    /*
     * Avoid derivative kick and allow tuning changes,
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
     */
    // D-term is being subtracted cause it is difference of inputs not the difference of errors
    output = (kP * Perror - kD * (p->sensor_ticks_per_second - p->prev_sensor_ticks_per_second) + p->ITerm) / kO;
    // output is accumulated. reducing load on kI. reduces overshooting
    output += p->output;

    // Accumulate Integral error *or* Limit output.
    // Stop accumulating when output saturates
    if (output >= MAX_PWM)
    {
        output = MAX_PWM;
    }
    else if (output <= -MAX_PWM)
    {
        output = -MAX_PWM;
    }
    else
    {
        // allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
        p->ITerm += kI * Perror;
    }

    p->output = output;
    // LOG_DEBUG("output:%lf, p->output:%lf, ITerm:%lf", output, p->output, p->ITerm);
    p->prev_sensor_ticks_per_second = p->sensor_ticks_per_second;
}

/* Read the encoder values and call the PID routine */
void updatePID()
{
    /* Read the encoders */
    leftPID.sensor_ticks_per_second = left_ticks_per_sec;
    rightPID.sensor_ticks_per_second = right_ticks_per_sec;
    /* If we're not moving there is nothing more to do */
    if (!should_move)
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
    // LOG_DEBUG("leftpid output:%d, right pid output:%d", (int)round(leftPID.output), (int)round(rightPID.output));
    setMotorSpeeds(round(leftPID.output), round(rightPID.output));
}
