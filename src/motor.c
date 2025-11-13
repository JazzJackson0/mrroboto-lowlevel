#include "../include/motor.h"

/**
 * @brief Provide a safety buffer (for the motor driver) between motor direction changes
 * @param motor
 */
static void safety_buffer(struct motor_data * motor);

/**
 * @brief Safely update the direction of the motor rotation
 * @param motor
 */
static void safe_motor_direction_change(struct motor_data * motor);


void initMotor(struct motor_data * motor, uint8_t pwm_pin, uint8_t dir_pin_1, uint8_t dir_pin_2) {

    // PWM Motor Output Setup
    motor->direction_pin_1 = dir_pin_1;
    motor->direction_pin_2 = dir_pin_2;
    motor->direction = 0;
    motor->pwm = pwmSetup(pwm_pin);
}

void motorSpeedOut(struct motor_data * left_motor, struct motor_data * right_motor) {
    
    pwmUpdateDutyCycle(left_motor->pwm.slice, left_motor->pwm.channel, left_motor->duty_cycle_percent);
    pwmUpdateDutyCycle(right_motor->pwm.slice, right_motor->pwm.channel, right_motor->duty_cycle_percent);
}

void motorDirectionOut(struct motor_data * left_motor, struct motor_data * right_motor) {
    safe_motor_direction_change(left_motor);    
    safe_motor_direction_change(right_motor);    
}

static void safety_buffer(struct motor_data * motor) {

    // Brake motor
    gpio_put(motor->direction_pin_1, 0);
    gpio_put(motor->direction_pin_2, 0);

    // Wait 100ms
    busy_wait_ms(101);
}

static void safe_motor_direction_change(struct motor_data * motor) {
    uint8_t mask = 1;
    uint8_t pin_1_dir = (motor->direction >> 1) & mask;
    uint8_t pin_2_dir = motor->direction & mask;

    // Apply safety buffer if reversing direction 
    if (gpio_get(motor->direction_pin_1) != pin_1_dir && gpio_get(motor->direction_pin_2) != pin_2_dir) {
        safety_buffer(motor);
    }
    gpio_put(motor->direction_pin_1, pin_1_dir);
    gpio_put(motor->direction_pin_2, pin_2_dir);
}

