#ifndef PWM_H
#define PWM_H
#include "boards/pico_w.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include "pico/divider.h"

#define PWM_FREQ 10000
#define CLOCK_FREQ 125000000
#define WRAP_VALUE (CLOCK_FREQ / PWM_FREQ) - 1 
#define SERVO_MIN 3
#define SERVO_MAX 13
#define SERVO_RANGE (SERVO_MAX - SERVO_MIN)



uint pwm_setup(int pin_num);

/**
 * @brief 
 * 
 * @param slice_num 
 * @param duty_cycle_percent Duty Cycle Percent from 0 - 1
 */
void pwm_update_duty_cycle(uint slice_num, float  duty_cycle_percent);

/**
 * @brief 
 * 
 * @param slice_num 
 * @param percent Percent from 0 - 1
 */
void pwm_update_servo_percent(uint slice_num, float percent);




#endif