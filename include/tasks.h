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
#include "pose_estimation.h"
#include "Quaternions.h"
#include "encoder.h"
#include "motor.h"
#include "imu.h"
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include "../CMSIS-DSP-main/Include/arm_math.h"
// #include "math_helper.h"

#define HIGH 1
#define LOW 0

// I2C Settings
#define DATA_RATE_HZ 100
#define FAST_MODE 400000 // 400 Kbps
#define STANDARD_MODE 100000 // 100 Kbps
#define I2C_SLAVE_ADDR 0x55 // Default Slave Address
#define I2C_SDA_PIN 26 // GP26 (GPIO PIN #31)
#define I2C_SCL_PIN 27 // GP27 (GPIO PIN # 32)
#define I2C_CLK_RATE 100000 // 100 KHz

// UART Settings
#define UART_ID uart1
#define UART_TX_GPIO 8
#define UART_RX_GPIO 9
#define UART_BAUD 9600
#define DATA_BITS 8
#define STOP_BITS 1

// UART Data Packet Settings
// Full Packet Format: [Type (1 Byte), Direction (1 Byte), Speed (4 Bytes), Speed (4 Bytes)]
// Direction: [LEFT | RIGHT]
#define DIRECTION_PACKET 2
#define SPEED_PACKET 9
#define FULL_PACKET 10
#define QUAD_PACKET 17

// Encoder Settings
#define ENCODER_R_INT_PIN_A 13 // GP13
#define ENCODER_R_INT_PIN_B 12 // GP12
#define ENCODER_L_INT_PIN_A 18 // GP18
#define ENCODER_L_INT_PIN_B 19 // GP19
#define DISTANCE_PER_ROTATION 0
#define TICKS_PER_ROTATION 0

// Motor Settings
#define MOTOR_R_PIN 11 // GP11
#define MOTOR_R_DIR_1_PIN 14 // GP14
#define MOTOR_R_DIR_2_PIN 15 // GP15
#define MOTOR_L_PIN  20 // GP20
#define MOTOR_L_DIR_1_PIN 16 // GP16
#define MOTOR_L_DIR_2_PIN 17 // GP17

// ...
#define TRACKWIDTH 5 // TODO: B.S number for now

#define DIST_BUFFER_SIZE 8
#define VEL_BUFFER_SIZE 16
#define MAX_PWM_BUFFER_SIZE 17

// ???
#define DT 0.01 // TODO: B.S number for now

void startTasks();


#endif