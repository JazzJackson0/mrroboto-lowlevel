#include "../include/pwm.h"


uint pwm_setup(int pin_num) {

    // Send pin_num into PWM mode
    gpio_set_function(pin_num, GPIO_FUNC_PWM);
    
    // Get PWM channel for pin_num
    uint slice_num = pwm_gpio_to_slice_num(pin_num);

    // Configure PWM channel and set it running
    pwm_set_wrap(slice_num, WRAP_VALUE);
    pwm_set_enabled(slice_num, true);

    return slice_num;
}

void pwm_update_duty_cycle(uint slice_num, float duty_cycle_percent) {

    uint16_t duty = (uint16_t) ((WRAP_VALUE + 1) * duty_cycle_percent);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty);
}

void pwm_update_servo_percent(uint slice_num, float percent) {
    // Calculate the value by clamping the percent from 0 to 100
    // to the SERVO_MIN and SERVO_MAX
    uint32_t value = (uint32_t) (percent * SERVO_RANGE) + SERVO_MIN;
    pwm_set_duty_cycle(slice_num, value);
}













