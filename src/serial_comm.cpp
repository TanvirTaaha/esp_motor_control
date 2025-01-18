#include "motor_control.h"
// MSG Format a_00000_00000__

char sending_data_buffer[MSG_LEN];
char receiving_data_buffer[MSG_LEN];
int bytes_read;
double ros_cmd_positions[2]; // [left, right]
unsigned long last_motor_command;

// Shared resource with mutex
SemaphoreHandle_t mutex;
int sharedCounter = 0;

void backgroundTask(void *pvParameters)
{
    for (;;)
    {
        // Safely modify shared resource
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
        {
            if (Serial.available() > 0)
            {
                read_data_from_serial();
            }
            send_data_to_serial();
            // Serial.println("from background");
            xSemaphoreGive(mutex);

            // Serial.printf("Background task incremented counter to %d on core %d\n",
            //               sharedCounter, xPortGetCoreID());
        }
        vTaskDelay(SERIAL_COMM_VTASK_DELAY);
    }
}

void send_data_to_serial()
{
    String s = String("e_") + String(left_ticks_per_sec) + String("_") + String(right_ticks_per_sec);
    // String s = String("e_") + String(get_encoder_count(LEFT)) + String("_") + String(get_encoder_count(RIGHT));

    for (int i = 0; i < (MSG_LEN - 1); i++)
    { // MSG_LEN - 1 because there is a trailing newline char
        if (i < s.length())
        { // copy the entire string s
            sending_data_buffer[i] = s.charAt(i);
        }
        else
        { // fillup the rest with '_'
            sending_data_buffer[i] = '_';
        }
    }
    Serial.flush();
    // Serial.print(sending_data_buffer);
    Serial.printf("%s\n", sending_data_buffer);
}

void read_data_from_serial()
{
    bytes_read = Serial.readBytesUntil('\n', receiving_data_buffer, MSG_LEN);
    // Make the rest of the bytes null character
    int i = bytes_read;
    while (i < MSG_LEN)
    {
        receiving_data_buffer[i++] = '\0';
    }
    // Serial.printf("bytes_read:%d. bytes:'%s'\n", bytes_read, receiving_data_buffer);
    if (bytes_read > 0 && bytes_read <= MSG_LEN)
    {
        switch (receiving_data_buffer[0])
        {
        case 'a': // a: (A)utomatic mode
            last_motor_command = millis();
            // Directly update the target of PID
            sscanf(receiving_data_buffer + 2, "%lf_%lf", &leftPID.target_ticks_per_second, &rightPID.target_ticks_per_second);
            break;
        case 'g': // g: (G)et count
            Serial.printf("count left%d right:%d\n", get_encoder_count(LEFT), get_encoder_count(RIGHT));
            break;
        case 't': // t: get (T)uning parameter
            Serial.printf("kP:%d,kI:%d,kD:%d,kO:%d", kP, kI, kD, kO);
            break;
        case 'p': // p: set (P)roportional gain
            sscanf(receiving_data_buffer, "_%d", &kP);
            break;
        case 'i': // i: set (I)ntregal gain
            sscanf(receiving_data_buffer, "_%d", &kI);
            break;
        case 'd': // d: set (D)ifferential gain
            sscanf(receiving_data_buffer, "_%d", &kD);
            break;
        case 'o': // o: set (O)utput gain
            sscanf(receiving_data_buffer, "_%d", &kO);
            break;
        default:
            break;
        }
    }
}

void setup_serial()
{
    // Create a mutex
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL)
    {
        Serial.println("Failed to create mutex!");
        while (1)
            ; // Halt execution
    }
    else
    {
        Serial.println("Successfully created mutex!");
    }

    // Create the background task and pin it to core 1
    BaseType_t result = xTaskCreatePinnedToCore(
        backgroundTask,   // Function to implement the task
        "SerialCommTask", // Name of the task
        2048,             // Stack size in words
        NULL,             // Task input parameter
        2,                // Priority of the task
        NULL,             // Task handle
        0                 // Core to pin the task to (1 for the other core)
    );

    if (result != pdPASS)
    {
        Serial.println("Failed to create task!");
    }
    else
    {
        Serial.println("Successfully created task!");
    }
}