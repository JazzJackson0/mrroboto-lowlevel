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
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

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

void start_tasks();


#endif