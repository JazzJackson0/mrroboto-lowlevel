#include "../include/motor.h"

void initMotor(struct motor_data * motor, uint8_t pwm_pin, uint8_t dir_pin_1, uint8_t dir_pin_2) {

    // PWM Motor Output Setup
    motor->direction_pin_1 = dir_pin_1;
    motor->direction_pin_2 = dir_pin_2;
    motor->direction = 0;
    motor->slice_num = pwmSetup(pwm_pin);
}

void motorSpeedOut(struct motor_data * left_motor, struct motor_data * right_motor) {
    
    pwmUpdateDutyCycle(left_motor->slice_num, PWM_CHAN_A, left_motor->duty_cycle_percent);
    pwmUpdateDutyCycle(right_motor->slice_num, PWM_CHAN_B, right_motor->duty_cycle_percent);
}

void motorDirectionOut(struct motor_data * left_motor, struct motor_data * right_motor) {
    uint8_t mask = 1;
    gpio_put(left_motor->direction_pin_1, (left_motor->direction >> 3) & mask);
    gpio_put(left_motor->direction_pin_2, (left_motor->direction >> 2) & mask);
    gpio_put(right_motor->direction_pin_1, (right_motor->direction >> 1) & mask);
    gpio_put(right_motor->direction_pin_2, right_motor->direction & mask);
}

