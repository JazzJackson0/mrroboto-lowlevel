#ifndef MPL3115A2_H
#define MPL3115A2_H
#include <math.h>
#include <stdio.h>
#include "boards/pico_w.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#define OUT_P_MSB 0x01
#define OUT_P_DELTA_MSB 0x07
#define PT_DATA_CFG 0x13
#define MPL_PRESSURE_TARGET_MSB_REG 0x16
#define MPL_PRESSURE_TARGET_LSB_REG 0x17
#define MPL_CTRL_REG1 0x26
#define MPL_CTRL_REG3 0x28
#define MPL_INTRRUPT_ENABLE_REG 0x29
#define MPL_INTRRUPT_CFG_REG 0x2A
#define ALTIMETER_ADDR 0x60

#define PICO_DATA_RATE_HZ 15
#define INT1_PIN 6 // INT1 PIN on MPL3115A2 connected to GPIO PIN 9 (GP6)


/**
 * @brief Initialize the MPL3115A2 Altimeter settings
 * 
 * @param target_altitude (meters)
 */
void altimeter_init(float target_altitude);


/**
 * @brief Read altitude data from Altimeter (MPL3115A2)
 * 
 * @return float 
 */
float get_altitude();


/**
 * @brief Get change in altitude value from Altimeter (MPL3115A2)
 *          and calculate velocity.
 * 
 * @return float 
 */
float get_velocity();






#endif