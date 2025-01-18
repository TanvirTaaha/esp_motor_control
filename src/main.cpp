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

    // Setup PID
    resetPID();

    // Setup Serial
    setup_serial();

    Serial.println("Started");
}

void loop()
{
    unsigned long _millis = millis();
    if ((_millis % 1000) == 0)
    {
        Serial.println("Loop");
    }
    if (_millis - last_motor_command > AUTO_STOP_INTERVAL)
    {
        setMotorSpeeds(0, 0);
        moving = 0;
    }
    if (_millis % TARGET_VELOCITY_UPDATE_INTERVAL == 0)
    {
        set_targets(ros_cmd_positions[0], ros_cmd_positions[1]);
    }
}