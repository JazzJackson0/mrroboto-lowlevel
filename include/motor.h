#ifndef MOTOR_H
#define MOTOR_H

#include <inttypes.h>
#include "pwm.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"


struct motor_data {
    struct pwm_model pwm;
    float duty_cycle_percent;
    int direction_pin_1;
    int direction_pin_2;
    uint8_t direction; // packet of direction values
};

/**
 * @brief
 * @param motor
 * @param pwm_pin
 * @param dir_pin_1
 * @param dir_pin_2
 */
void initMotor(struct motor_data * motor, uint8_t pwm_pin, uint8_t dir_pin_1, uint8_t dir_pin_2);

/**
 * @brief Update motor PWM speed values
 * @param left_motor
 * @param right_motor
 * @return 
 */
void motorSpeedOut(struct motor_data * left_motor, struct motor_data * right_motor); 

/**
 * @brief Update motor direction pins
 * @param left_motor
 * @param right_motor
 * @return 
 */
void motorDirectionOut(struct motor_data * left_motor, struct motor_data * right_motor);


#endif