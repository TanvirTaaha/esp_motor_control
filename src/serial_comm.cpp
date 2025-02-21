#include "motor_control.h"
// MSG Format a_00000_00000__

char sending_data_buffer[MSG_LEN];
char receiving_data_buffer[MSG_LEN];
int bytes_read;
float ros_cmd_velocities[2];  // [left, right]
unsigned long last_motor_command;

int32_t pos, last_pos;
// Shared resource with mutex
SemaphoreHandle_t mutex;
int sharedCounter = 0;

void backgroundTask(void *pvParameters) {
  for (;;) {
    // Safely modify shared resource
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
      if (Serial.available() > 0) {
        yield();
        read_data_from_serial();
      }
      yield();
      send_data_to_serial();

      // Serial.println("from background");
      xSemaphoreGive(mutex);

      // Serial.printf("Background task incremented counter to %d on core %d\n",
      //               sharedCounter, xPortGetCoreID());
    }
    vTaskDelay(SERIAL_COMM_VTASK_DELAY);
  }
}

void send_data_to_serial() {
  sprintf(sending_data_buffer, "e_%.1lf_%.1lf_%d_%d", left_ticks_per_sec, right_ticks_per_sec, get_encoder_count(LEFT), get_encoder_count(RIGHT));
  // pos = (pos + 50) % 100000;
  // float vel = (pos * 1.0 - last_pos * 1.0) / (1.0 * SERIAL_COMM_VTASK_DELAY);

  // sprintf(sending_data_buffer, "e_%.1lf_%.1lf_%d_%d\0", vel, -vel, pos, -pos);
  // last_pos = pos;

  // MSG_LEN - 1 because there is a trailing newline char
  for (int i = strlen(sending_data_buffer); i < (MSG_LEN - 1); i++) {
    sending_data_buffer[i] = '_';
  }
  yield();
  Serial.flush();
  // Serial.print(sending_data_buffer);
  Serial.printf("%s\n", sending_data_buffer);
}

void read_data_from_serial() {
  bytes_read = Serial.readBytesUntil('\n', receiving_data_buffer, MSG_LEN);
  yield();
  // Make the rest of the bytes null character
  int i = bytes_read;
  while (i < MSG_LEN) {
    receiving_data_buffer[i++] = '\0';
  }
  if (bytes_read > 0 && bytes_read <= MSG_LEN) {
    switch (receiving_data_buffer[0]) {
      case 'a':  // a: (A)utomatic mode
        last_motor_command = millis();
        should_move = true;
        // Directly update the target of PID does not work because the PID loop is running in the background
        float left_target, right_target;
        if (sscanf(receiving_data_buffer, "a_%f_%f", &left_target, &right_target) == 2) {
          leftPID.target_ticks_per_second = left_target;
          rightPID.target_ticks_per_second = right_target;
        }
        break;
      case 'g':  // g: (G)et count
        Serial.printf("count left%d right:%d\n", get_encoder_count(LEFT), get_encoder_count(RIGHT));
        break;
      case 't':  // t: get (T)uning parameter
        Serial.printf("kP:%d,kI:%d,kD:%d,kO:%d", kP, kI, kD, kO);
        break;
      case 'p':  // p: set (P)roportional gain
        int val;
        sscanf(receiving_data_buffer, "p_%d", &val);
        kP = val;
        break;
      case 'i':  // i: set (I)ntregal gain
        int val;
        sscanf(receiving_data_buffer, "i_%d", &val);
        kI = val;
        break;
      case 'd':  // d: set (D)ifferential gain
        int val;
        sscanf(receiving_data_buffer, "d_%d", &val);
        kD = val;
        break;
      case 'o':  // o: set (O)utput gain
        int val;
        sscanf(receiving_data_buffer, "o_%d", &val);
        kO = val;
        break;
      default:
        break;
    }
  }
}

void setup_serial() {
  // Create a mutex
  mutex = xSemaphoreCreateMutex();
  if (mutex == NULL) {
    Serial.println("Failed to create mutex!");
    while (1);  // Halt execution
  } else {
    Serial.println("Successfully created mutex!");
  }

  // Create the background task and pin it to core 1
  BaseType_t result = xTaskCreatePinnedToCore(
      backgroundTask,    // Function to implement the task
      "SerialCommTask",  // Name of the task
      2048,              // Stack size in words
      NULL,              // Task input parameter
      2,                 // Priority of the task
      NULL,              // Task handle
      0                  // Core to pin the task to (1 for the other core)
  );

  if (result != pdPASS) {
    Serial.println("Failed to create task!");
  } else {
    Serial.println("Successfully created task!");
  }
}