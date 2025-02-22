#include "motor_control.h"

// Previous counts for velocity calculation
static int16_t prev_left_count = 0;
static int16_t prev_right_count = 0;
static int16_t current_count;

// Velocities in steps per second
volatile float left_ticks_per_sec = 0;
volatile float right_ticks_per_sec = 0;

// Timer interrupt handler
void calc_vel_and_pid_and_write(void *args) {
  // 1. Get current counts
  // 2. Calculate velocities (steps per second). Multiply by VELOCITY_CALC_FREQ to convert to per second
  // 3. Store current counts for next calculation
  pcnt_get_counter_value(PCNT_UNIT_0, &current_count);
  left_ticks_per_sec = STEPS_DIFF(current_count - prev_left_count) * VELOCITY_CALC_FREQ;
  // left_ticks_per_sec = (current_count - prev_left_count) * VELOCITY_CALC_FREQ;
  prev_left_count = current_count;

  pcnt_get_counter_value(PCNT_UNIT_1, &current_count);
  right_ticks_per_sec = STEPS_DIFF(current_count - prev_right_count) * VELOCITY_CALC_FREQ;
  // right_ticks_per_sec = (current_count - prev_right_count) * VELOCITY_CALC_FREQ;
  prev_right_count = current_count;

  updatePID();
}

void setup_timer() {
  esp_timer_create_args_t timer_args = {
      .callback = calc_vel_and_pid_and_write,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "calc_vel_and_pid_and_write",
  };
  esp_timer_handle_t timer;
  esp_timer_create(&timer_args, &timer);
  esp_timer_start_periodic(timer, TIMER_SCALE / VELOCITY_CALC_FREQ);
}

void setup_encoder(pcnt_unit_t unit, int pinA, int pinB) {
  // Your existing encoder setup code...
  pcnt_config_t config = {
      .pulse_gpio_num = pinA,
      .ctrl_gpio_num = pinB,
      .lctrl_mode = PCNT_MODE_REVERSE,
      .hctrl_mode = PCNT_MODE_KEEP,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DEC,
      .counter_h_lim = 32767,
      .counter_l_lim = -32768,
      .unit = unit,
      .channel = PCNT_CHANNEL_0,
  };

  pcnt_unit_config(&config);
  pcnt_set_filter_value(unit, 100);
  pcnt_filter_enable(unit);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

// void setup()
// {
//     Serial.begin(115200);

//     // Setup encoders
//     setup_encoder(PCNT_UNIT_0, LEFT_ENC_A, LEFT_ENC_B);
//     setup_encoder(PCNT_UNIT_1, RIGHT_ENC_A, RIGHT_ENC_B);

//     // Setup timer for velocity calculation
//     setup_timer();
// }

// Function to get encoder count
int16_t get_encoder_count(uint8_t i) {
  int16_t count;
  if (i == LEFT) {
    pcnt_get_counter_value(LEFT_ENC_PCNT_UNIT, &count);
  } else if (i == RIGHT) {
    pcnt_get_counter_value(RIGHT_ENC_PCNT_UNIT, &count);
  }
  return count;
}

// void loop()
// {
//     // Print velocities
//     Serial.printf("Velocities (steps/s) - Left: %.2f, Right: %.2f ", left_ticks_per_sec, right_ticks_per_sec);
//     // Read both encoder counts
//     int16_t left_count = get_encoder_count(PCNT_UNIT_0);
//     int16_t right_count = get_encoder_count(PCNT_UNIT_1);
//     Serial.printf("Counts (steps) - Left: %d, Right: %d\n", left_count, right_count);

//     delay(100); // Update display every 100ms
// }

void reset_encoders() {
  left_ticks_per_sec = 0.0f;
  right_ticks_per_sec = 0.0f;
}