/**
 * @file persistence.cpp
 * @author Tanvir Hossain Taaha (tanvir.taaha@gmail.com)
 * @brief Implementation of EEPROM functions for storing and retrieving PID tuning parameters
 * @version 0.1
 * @date 2025-02-22
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <EEPROM.h>

#include "motor_control.h"
const int EEPROM_SIZE = 64;
struct PIDParams {
  int16_t kp;
  int16_t ki;
  int16_t kd;
  int16_t ko;
} pid_params = {0, 0, 0, 0};

void eeprom_setup() {
  if (!EEPROM.begin(EEPROM_SIZE)) {
    LOG_ERROR("Failed to init EEPROM");
  }
}

void eeprom_write_pid_params() {
  pid_params.kp = kP;
  pid_params.ki = kI;
  pid_params.kd = kD;
  pid_params.ko = kO;
  EEPROM.writeBytes(0, &pid_params, sizeof(pid_params));
  EEPROM.commit();
  LOG_DEBUG("PID params written successfully to EEPROM: kp=%d, ki=%d, kd=%d, ko=%d", pid_params.kp, pid_params.ki, pid_params.kd, pid_params.ko);
}

void eeprom_read_pid_params() {
  EEPROM.readBytes(0, &pid_params, sizeof(pid_params));
  kP = pid_params.kp;
  kI = pid_params.ki;
  kD = pid_params.kd;
  kO = pid_params.ko;
  LOG_DEBUG("PID params read successfully from EEPROM: kp=%d, ki=%d, kd=%d, ko=%d", pid_params.kp, pid_params.ki, pid_params.kd, pid_params.ko);
}
