#ifndef BN0055_H
#define BNO055_H
#include <stdio.h>
#include "boards/pico_w.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"

#define BNO055_ID 0xA0
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_ADDRESS_A 0x28
#define BNO055_ADDRESS_B 0x29

#define BNO055_SYS_TRIGGER_REG 0X3F
#define BNO055_PWR_MODE_REG 0X3E
#define BNO055_AXIS_MAP_CONFIG_REG 0X41
#define BNO055_AXIS_MAP_SIGN_REG 0X42
#define BNO055_UNIT_SEL_REG 0X3B
#define BNO055_OPR_MODE_REG 0X3D
#define OPERATION_MODE_NDOF 0X0C

#define BNO055_CALIB_STAT_REG 0X35
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_REG 0X28
#define BNO055_QUATERNION_DATA_W_LSB_REG 0X20
#define BNO055_EULER_H_LSB_REG 0X1A
#define BNO055_ACCEL_DATA_X_LSB_REG 0X08


struct _quaternion {
    float w;
    float x;
    float y;
    float z;
};

struct _vector3f {
    float x;
    float y;
    float z;
};

struct _CALIB_STATUS {
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
};

typedef struct _quaternion quaternion;
typedef struct _vector3f vector3f;
typedef struct _CALIB_STATUS CALIB_STATUS;


/**
 * @brief 
 * 
 */
void imu_init();

/**
 * @brief 
 * 
 * @return CALIB_STATUS 
 */
CALIB_STATUS read_calib_status();

/**
 * @brief 
 * 
 * @return vector3f 
 */
vector3f read_accel();

/**
 * @brief 
 * 
 * @return vector3f 
 */
vector3f read_lin_accel();

/**
 * @brief 
 * 
 * @return quaternion 
 */
quaternion read_abs_quaternion();

/**
 * @brief 
 * 
 * @return vector3f 
 */
vector3f read_euler_angles();




#endif