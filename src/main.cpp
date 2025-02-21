#include "motor_control.h"

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Setup encoders
  setup_encoder(LEFT_ENC_PCNT_UNIT, LEFT_ENC_A, LEFT_ENC_B);
  setup_encoder(RIGHT_ENC_PCNT_UNIT, RIGHT_ENC_A, RIGHT_ENC_B);
  // Setup timer for velocity calculation
  setup_timer();

  // setup motor)driver
  initMotorDriver();

  // Setup PID
  resetPID();

  // Setup Serial
  setup_serial();

  Serial.println("Started");
}

void loop() {
  auto _millis = millis();
  if (should_move && (_millis - last_motor_command) > AUTO_STOP_INTERVAL) {
    LOG_DEBUG("last_targets: %f, %f", leftPID.target_ticks_per_second, rightPID.target_ticks_per_second);
    LOG_INFO("setting motor to zero");
    setMotorSpeeds(0, 0);
    should_move = false;
  }
  // if (_millis % TARGET_VELOCITY_UPDATE_INTERVAL == 0)
  // {
  //     set_targets(ros_cmd_velocities[0], ros_cmd_velocities[1]);
  //     Serial.println("Setting target");
  // }
}