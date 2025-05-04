#ifndef TASKS_H
#define TASKS_H
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
// For Pico W
// #include "boards/pico_w.h"
// #include "pico/cyw43_arch.h" // Required for initializing and using the Wi-Fi chip
#include "pwm.h"
#include "bno055.h"
#include "Quaternions.h"
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include "../CMSIS-DSP-main/Include/arm_math.h"
// #include "math_helper.h"

#define DATA_RATE_HZ 100
#define FAST_MODE 400000 // 400 Kbps
#define STANDARD_MODE 100000 // 100 Kbps

#define I2C_SLAVE_ADDR 0x55 // Default Slave Address
#define I2C_SDA_PIN 26 // GP26 (GPIO PIN #31)
#define I2C_SCL_PIN 27 // GP27 (GPIO PIN # 32)
#define I2C_CLK_RATE 100000 // 100 KHz

#define ENCODER_R_INT_PIN 13 // GP13
#define ENCODER_L_INT_PIN 18 // GP18

#define MOTOR_R_PIN 12 // GP12
#define MOTOR_L_PIN  19 // GP19 

#define UART_ID uart1
#define UART_TX_GPIO 8
#define UART_RX_GPIO 9
#define UART_BAUD 9600
#define DATA_BITS 8
#define STOP_BITS 1

#define DIST_BUFFER_SIZE 8
#define VEL_BUFFER_SIZE 16
#define PWM_BUFFER_SIZE 8

#define NUM_STAGES 2 // (4th Order Filter / 2) = 2
#define BLOCK_SIZE 25 // Number of samples processed per function call
#define POST_SHIFT 5 /* Number of bits the output of each biquad filter stage is right-shifted to keep it within the valid Q31 range and avoid overflow.
                        Low Value (0-1): keeps output close to full Q31 resolution. But the internal values may overflow, causing distortion or wraparound
                        High Value (4+): reduces the risk of overflow. But you lose resolution (precision), and output amplitude becomes smaller.
                    */
#define NUM_COEFFS 12
/*
A larger POST_SHIFT reduces output magnitude, preventing overflow.
A smaller POST_SHIFT keeps more precision, but risks overflow.
*/

#define FIXED_PT_CONVERSION 1000

#define MEDIAN_FILTER_SIZE 11
#define DT 0.01

// Complementary Filter Weights
#define GYRO_WEIGHT 0.98
#define ACCEL_WEIGHT 1 - GYRO_WEIGHT

void start_tasks();


#endif