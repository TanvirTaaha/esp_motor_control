#include "motor_control.h"
/* PID Parameters. Will be initialized from eeprom */
int16_t kP;
int16_t kD;
int16_t kI;
int16_t kO;

volatile SetPointInfo leftPID, rightPID;
bool should_move = false;  // is the base in motion?

void PID_init() {
  eeprom_read_pid_params();
  LOG_DEBUG("retrieved from eeprom: kP:%d, kD:%d, kI:%d, kO:%d", kP, kD, kI, kO);
  // Set default values for PID parameters if invalid
  if (isnan(kP) || isnan(kD) || isnan(kI) || isnan(kO) ||
      isinf(kP) || isinf(kD) || isinf(kI) || isinf(kO) ||
      kP < 0 || kD < 0 || kI < 0 || kO < 0 ||
      kP > (1 << 15) || kD > (1 << 15) || kI > (1 << 15) || kO > (1 << 15) ||
      kP + kD + kI + kO == 0) {
    kP = 10;
    kD = 5;
    kI = 0;
    kO = 50;
    eeprom_write_pid_params();
  }
  LOG_DEBUG("After checking : kP:%d, kD:%d, kI:%d, kO:%d", kP, kD, kI, kO);
  resetPID();
}

/*
 * Initialize PID variables to zero to prevent startup spikes
 * when turning PID on to start moving
 * In particular, assign both Encoder and PrevEnc the current encoder value
 * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
 * Note that the assumption here is that PID is only turned on
 * when going from stop to moving, that's why we can init everything on zero.
 */
void resetPID() {
  leftPID.target_ticks_per_second = 0.0;
  leftPID.sensor_ticks_per_second = left_ticks_per_sec;
  leftPID.prev_sensor_ticks_per_second = leftPID.sensor_ticks_per_second;
  leftPID.output = 0;
  leftPID.ITerm = 0;

  rightPID.target_ticks_per_second = 0.0;
  rightPID.sensor_ticks_per_second = right_ticks_per_sec;
  rightPID.prev_sensor_ticks_per_second = rightPID.sensor_ticks_per_second;
  rightPID.output = 0;
  rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(volatile SetPointInfo *p) {
  float Perror = 0.0;
  float output = 0.0;

  // Perror = p->target_ticks_per_second - (p->Encoder - p->PrevEnc);
  Perror = p->target_ticks_per_second - p->sensor_ticks_per_second;
  // LOG_DEBUG("Perror:%f", Perror);
  /*
   * Avoid derivative kick and allow tuning changes,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   */
  // D-term is being subtracted cause it is difference of inputs not the difference of errors
  LOG_DEBUG("taget:%f, sensor:%f, output:%f, p->output:%f, ITerm:%f, Perror:%f", p->target_ticks_per_second, p->sensor_ticks_per_second, output, p->output, p->ITerm, Perror);
  output = (kP * Perror - kD * (p->sensor_ticks_per_second - p->prev_sensor_ticks_per_second) + p->ITerm) / kO;
  // output is accumulated. reducing load on kI. reduces overshooting
  output += p->output;
  LOG_DEBUG("taget:%f, sensor:%f, output:%f, p->output:%f, ITerm:%f, Perror:%f", p->target_ticks_per_second, p->sensor_ticks_per_second, output, p->output, p->ITerm, Perror);
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM) {
    output = MAX_PWM;
  } else if (output <= -MAX_PWM) {
    output = -MAX_PWM;
  } else {
    // LOG_DEBUG("Iterm is updating");
    // allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    p->ITerm += kI * Perror;
  }

  p->output = output;
  // LOG_DEBUG("output:%f, p->output:%f, ITerm:%f", output, p->output, p->ITerm);
  p->prev_sensor_ticks_per_second = p->sensor_ticks_per_second;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.sensor_ticks_per_second = left_ticks_per_sec;
  rightPID.sensor_ticks_per_second = right_ticks_per_sec;
  /* If we're not moving there is nothing more to do */
  if (!should_move) {
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
  int leftSpd = round(leftPID.output), rightSpd = round(rightPID.output);
  LOG_DEBUG("leftpid output:%d, right pid output:%d", leftSpd, rightSpd);
  setMotorSpeeds(leftSpd, rightSpd);
}
