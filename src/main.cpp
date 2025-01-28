#include "motor_control.h"

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

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

void loop()
{
    auto _millis = millis();
    if (!zeroed && (_millis - last_motor_command) > AUTO_STOP_INTERVAL)
    {
#ifdef MOTOR_CONTROL_DEBUG
        Serial.printf("last_targets: %lf, %lf\n", leftPID.target_ticks_per_second, rightPID.target_ticks_per_second);
        Serial.println("setting motor to zero");
#endif
        setMotorSpeeds(0, 0);
        moving = 0;
        zeroed = true;
    }
    // if (_millis % TARGET_VELOCITY_UPDATE_INTERVAL == 0)
    // {
    //     set_targets(ros_cmd_positions[0], ros_cmd_positions[1]);
    //     Serial.println("Setting target");
    // }
}