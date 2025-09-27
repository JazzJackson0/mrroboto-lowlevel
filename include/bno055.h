#ifndef BN0055_H
#define BNO055_H
#include <stdio.h>
#include <math.h>
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
#define BNO055_GYR_DATA_X_LSB_REG 0X14

#define LSB_ACCEL 100.f // 100 LSB = 1 m/s²
#define LSB_ROT 16.f // 16 LSB = 1 °/s


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
void initBno055();

/**
 * @brief 
 * 
 * @return CALIB_STATUS 
 */
CALIB_STATUS readCalibStatus();

/**
 * @brief Includes the effects of gravity as well as any motion-induced accelerations.
 *      If the sensor is at rest on a horizontal surface, it will measure an acceleration of approximately 9.8 m/s^2 due to gravity
 *      Includes both the robot's acceleration and the effect of gravity.
 * 
 * @return vector3f 
 */
vector3f readAccel();

/**
 * @brief Acceleration with the gravity component removed. Represents only the acceleration due to the motion of the sensor.
 *      If the sensor is at rest on a flat surface, the linear acceleration will be 0 m/s^2
 *      Represents only the robot's movement, making it easier to analyze its dynamics or use it for motion control.
 * 
 * @return vector3f 
 */
vector3f readLinAccel();

/**
 * @brief Rotational velocity (Radians)
 * 
 * @return vector3f 
 */
vector3f readRotVel(); 

/**
 * @brief 
 * 
 * @return quaternion 
 */
quaternion readAbsQuaternion();

/**
 * @brief 
 * 
 * @return vector3f 
 */
vector3f readEulerAngles();




#endif