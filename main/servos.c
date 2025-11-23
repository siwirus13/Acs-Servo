#include "../include/servos.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#define SERVO_PIN_1 13
#define SERVO_PIN_2 12
#define SERVO_PIN_3 14
#define SERVO_PIN_4 27

#define SERVO_CH_1   LEDC_CHANNEL_0
#define SERVO_CH_2   LEDC_CHANNEL_1
#define SERVO_CH_3   LEDC_CHANNEL_2
#define SERVO_CH_4   LEDC_CHANNEL_3

#define LEDC_SPEED_MODE   LEDC_LOW_SPEED_MODE
#define LEDC_FREQ         50       // 50 Hz servo PWM
#define LEDC_RES          LEDC_TIMER_10_BIT  // official enum, not raw 10

#define MIN_PULSE 1000
#define MAX_PULSE 2000
#define SERVO_RANGE 180

#define MIN_ANGLE (-(SERVO_RANGE/2))
#define MAX_ANGLE ( SERVO_RANGE/2 )

const uint8_t servo_pins[4] = {
    SERVO_PIN_1, SERVO_PIN_2, SERVO_PIN_3, SERVO_PIN_4
};

const ledc_channel_t servo_channels[4] = {
    SERVO_CH_1, SERVO_CH_2, SERVO_CH_3, SERVO_CH_4
};

uint32_t angle_to_duty(float angle)
{
    if (angle > MAX_ANGLE) angle = MAX_ANGLE;
    if (angle < MIN_ANGLE) angle = MIN_ANGLE;

    float pulse = (((angle - MIN_ANGLE) / SERVO_RANGE) *
                   (MAX_PULSE - MIN_PULSE)) + MIN_PULSE;

    uint32_t max_duty = (1 << 10) - 1;  // 10-bit resolution = 1023
    uint32_t duty = (pulse * max_duty) / (1000000 / LEDC_FREQ);

    return duty;
}

bool init_servos()
{
    bool error = false;

    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_SPEED_MODE,
        .duty_resolution  = LEDC_RES,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = LEDC_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };

    error |= ESP_ERROR_CHECK_WITHOUT_ABORT( ledc_timer_config(&timer_cfg) ) != ESP_OK;

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {
            .gpio_num       = servo_pins[i],
            .speed_mode     = LEDC_SPEED_MODE,
            .channel        = servo_channels[i],
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .duty           = angle_to_duty(0),
            .hpoint         = 0
        };

        error |= ESP_ERROR_CHECK_WITHOUT_ABORT( ledc_channel_config(&ch) ) != ESP_OK;

        error |= ESP_ERROR_CHECK_WITHOUT_ABORT(
            ledc_update_duty(LEDC_SPEED_MODE, servo_channels[i])
        ) != ESP_OK;
    }

    return !error;
}

void update_canards(const canards_t *c)
{
    ledc_set_duty(LEDC_SPEED_MODE, SERVO_CH_1, angle_to_duty(c->canard_angle1));
    ledc_set_duty(LEDC_SPEED_MODE, SERVO_CH_2, angle_to_duty(c->canard_angle2));
    ledc_set_duty(LEDC_SPEED_MODE, SERVO_CH_3, angle_to_duty(c->canard_angle3));
    ledc_set_duty(LEDC_SPEED_MODE, SERVO_CH_4, angle_to_duty(c->canard_angle4));

    ledc_update_duty(LEDC_SPEED_MODE, SERVO_CH_1);
    ledc_update_duty(LEDC_SPEED_MODE, SERVO_CH_2);
    ledc_update_duty(LEDC_SPEED_MODE, SERVO_CH_3);
    ledc_update_duty(LEDC_SPEED_MODE, SERVO_CH_4);
}
