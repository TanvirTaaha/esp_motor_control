#include "motor_control.h"
// MSG Format a_00000_00000__

char sending_data_buffer[MSG_LEN];
char receiving_data_buffer[MSG_LEN];
int bytes_read;
double ros_cmd_positions[2];    // [left, right]
double velocity_from_sensor[2]; // [left, right]
unsigned long last_heard;

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
            read_data_from_serial();
            velocity_from_sensor[0] = readEncoder(LEFT);
            velocity_from_sensor[0] = readEncoder(RIGHT);
            send_data_to_serial(velocity_from_sensor);
            xSemaphoreGive(mutex);
            
            // Serial.printf("Background task incremented counter to %d on core %d\n",
            //               sharedCounter, xPortGetCoreID());
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void send_data_to_serial(double *state_positions)
{
    String s = String("e_") + String(state_positions[0]) + String("_") + state_positions[1];

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
        if (receiving_data_buffer[0] == 'a')
        {
            last_heard = millis();
            sscanf(receiving_data_buffer + 2, "%lf_%lf", &ros_cmd_positions[0], &ros_cmd_positions[1]);
        }
    }
}
