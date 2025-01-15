#include "motor_control.h"


void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Started");
}

void loop()
{
    Serial.println("Loop");
    delay(1000);
}