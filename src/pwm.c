#include "../include/pwm.h"

/**
 * @brief clamp given value to a range between 0 and 1
 * 
 * @param percentage 
 * 
 * @return clamped and converted percentage
 */
static float percentage_clamp(float percentage);


struct pwm_model pwmSetup(int pin_num) {

    struct pwm_model pwm;
    // Send pin_num into PWM mode
    gpio_set_function(pin_num, GPIO_FUNC_PWM);
    
    // Get PWM slice & channel for pin_num
    pwm.slice = pwm_gpio_to_slice_num(pin_num);
    pwm.channel = pwm_gpio_to_channel(pin_num);

    // Configure PWM channel and set it running
    pwm_set_wrap(pwm.slice, WRAP_VALUE);
    pwm_set_enabled(pwm.slice, true);

    return pwm;
}

void pwmUpdateDutyCycle(uint slice_num, uint8_t channel, float duty_cycle_percent) {

    uint16_t duty = (uint16_t) ((WRAP_VALUE + 1) * percentage_clamp(duty_cycle_percent));
    pwm_set_chan_level(slice_num, channel, duty);
}

void pwmUpdateServoPercent(uint slice_num, float percent) {
    // Calculate the value by clamping the percent from 0 to 100
    // to the SERVO_MIN and SERVO_MAX
    uint32_t value = (uint32_t) (percentage_clamp(percent) * SERVO_RANGE) + SERVO_MIN;
    pwm_set_duty_cycle(slice_num, value);
}


static float percentage_clamp(float percentage) {
    if (percentage < 0) {
        percentage = 0;
    }
    if (percentage > 100) {
        percentage = 100;
    }
    if (percentage > 1) {
        percentage /= 100;
    }

    return percentage;
}













