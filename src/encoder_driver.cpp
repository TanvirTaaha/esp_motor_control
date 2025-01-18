#include "motor_control.h"

// Previous counts for velocity calculation
static int16_t prev_left_count = 0;
static int16_t prev_right_count = 0;
static int16_t current_count;

// Velocities in steps per second
volatile double left_ticks_per_sec = 0;
volatile double right_ticks_per_sec = 0;

// Timer interrupt handler
bool IRAM_ATTR timer_group_isr_callback(void *args)
{

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

    return false; // Keep the watchdog happy
}

void setup_timer()
{
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,         // Enable alarm
        .counter_en = TIMER_START,          // Start immediately
        .intr_type = TIMER_INTR_LEVEL,      // Interrupt mode
        .counter_dir = TIMER_COUNT_UP,      // Count up
        .auto_reload = TIMER_AUTORELOAD_EN, // Auto-reload on alarm
        .divider = TIMER_DIVIDER            // Counter clock divider
    };

    // Initialize timer with config
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Set alarm value (20ms)
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_SCALE / VELOCITY_CALC_FREQ);

    // Enable timer interrupt
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    // Register interrupt callback
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, NULL, 0);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void setup_encoder(pcnt_unit_t unit, int pinA, int pinB)
{
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
int16_t get_encoder_count(uint8_t i)
{
    int16_t count;
    if (i == LEFT)
    {
        pcnt_get_counter_value(LEFT_ENC_PCNT_UNIT, &count);
    }
    else if (i == RIGHT)
    {
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

void reset_encoders()
{
    left_ticks_per_sec = 0.0f;
    right_ticks_per_sec = 0.0f;
}