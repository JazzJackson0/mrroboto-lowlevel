#include "../include/bno055.h"



void imuInit() {
    sleep_ms(1000); // Wait 650ms for the sensor to reset
    uint8_t id[1];
    uint8_t chip_id_addr = BNO055_CHIP_ID_ADDR;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, &chip_id_addr, 1, false);
    i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, id, 1, false);
    if (!id[0] == BNO055_ID) {
        printf("IMU Not Detected\n");
    }

    // Set to reset all interrupt status bits, and INT output
    uint8_t config[2];
    config[0] = BNO055_SYS_TRIGGER_REG;
    config[1] = 0x40; 
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, config, 2, true);

    // Map the Axes P1=Z, P2=Y, P3=X (Default Mapping)
    config[0] = BNO055_AXIS_MAP_CONFIG_REG;
    config[1] = 0x24; 
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, config, 2, true);

    // Set Axis Sign (Default)
    config[0] = BNO055_AXIS_MAP_SIGN_REG;
    config[1] = 0x00;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, config, 2, true);

    // Set Units to m/s^2 (Default)
    config[0] = BNO055_UNIT_SEL_REG;
    config[1] = 0x00;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, config, 2, true);
    sleep_ms(30);

    // Set Normal Power Mode (Default)
    config[0] = BNO055_PWR_MODE_REG;
    config[1] = 0x00;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, config, 2, true);
    sleep_ms(50); // Wait 50ms for the sensor to switch to normal power mode

    //The default operation mode after power-on is CONFIGMODE
    // Set mode to NDOF
    // Takes 7ms to switch from CONFIG mode; see page 21 on datasheet (3.3)
    config[0] = BNO055_OPR_MODE_REG;
    config[1] = OPERATION_MODE_NDOF;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, config, 2, false);
    sleep_ms(100);
}



CALIB_STATUS readCalibStatus() {
    CALIB_STATUS calib_status;
    uint8_t calib_stat[1];
    uint8_t calib_stat_reg = BNO055_CALIB_STAT_REG;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, &calib_stat_reg, 1, true);
    i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, calib_stat, 1, false);

    calib_status.mag = ((calib_stat[0] & 0b00000011) >> 0);
    calib_status.accel = ((calib_stat[0] & 0b00001100) >> 2);
    calib_status.gyro = ((calib_stat[0] & 0b00110000) >> 4);
    calib_status.sys = ((calib_stat[0] & 0b11000000) >> 6);

    return calib_status;
}


vector3f readAccel() {
    vector3f acceleration;
    uint8_t accel[6];
    uint8_t accel_reg = BNO055_ACCEL_DATA_X_LSB_REG;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, &accel_reg, 1, true);
    i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, accel, 6, false);

    int16_t x, y, z;
    x = y = z = 0;
    x = ((int16_t)accel[0]) | (((int16_t)accel[1]) << 8);
    y = ((int16_t)accel[2]) | (((int16_t)accel[3]) << 8);
    z = ((int16_t)accel[4]) | (((int16_t)accel[5]) << 8);

    acceleration.x = ((float)x) / LSB_ACCEL;
    acceleration.y = ((float)y) / LSB_ACCEL;
    acceleration.z = ((float)z) / LSB_ACCEL;

    return acceleration;
}


vector3f readLinAccel() {
    vector3f linear_acceleration;
    uint8_t accel[6];
    uint8_t lin_accel_reg = BNO055_LINEAR_ACCEL_DATA_X_LSB_REG;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, &lin_accel_reg, 1, true);
    i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, accel, 6, false);

    int16_t x, y, z;
    x = y = z = 0;
    x = ((int16_t)accel[0]) | (((int16_t)accel[1]) << 8);
    y = ((int16_t)accel[2]) | (((int16_t)accel[3]) << 8);
    z = ((int16_t)accel[4]) | (((int16_t)accel[5]) << 8);

    // reg_val / LSB = m/s^2
    linear_acceleration.x = ((float)x) / LSB_ACCEL;
    linear_acceleration.y = ((float)y) / LSB_ACCEL;
    linear_acceleration.z = ((float)z) / LSB_ACCEL;

    return linear_acceleration;
}

vector3f readRotVel() {
    vector3f rotational_vel; // radians
    uint8_t rot_vel[6];
    uint8_t rot_vel_reg = BNO055_GYR_DATA_X_LSB_REG;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, &rot_vel_reg, 1, true);
    i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, rot_vel, 6, false);

    int16_t x, y, z;
    x = y = z = 0;
    x = ((int16_t)rot_vel[0]) | (((int16_t)rot_vel[1]) << 8);
    y = ((int16_t)rot_vel[2]) | (((int16_t)rot_vel[3]) << 8);
    z = ((int16_t)rot_vel[4]) | (((int16_t)rot_vel[5]) << 8);

    // reg_val / LSB = DPS
    rotational_vel.x = ((float)x) / LSB_ROT;
    rotational_vel.y = ((float)y) / LSB_ROT;
    rotational_vel.z = ((float)z) / LSB_ROT;

    // rad/s
    // rotational_vel.x = ((float)x) * (M_PI / 180.f);
    // rotational_vel.y = ((float)y) * (M_PI / 180.f);
    // rotational_vel.z = ((float)z) * (M_PI / 180.f);

    return rotational_vel;
}

quaternion readAbsQuaternion() {
    quaternion abs_quaternion;
    uint8_t quat[8];
    uint8_t quat_reg = BNO055_QUATERNION_DATA_W_LSB_REG;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, &quat_reg, 1, true);
    i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, quat, 8, false);

    int16_t w, x, y, z;
    w = x = y = z = 0;
    w = ((int16_t)quat[0]) | (((int16_t)quat[1]) << 8);
    x = ((int16_t)quat[2]) | (((int16_t)quat[3]) << 8);
    y = ((int16_t)quat[4]) | (((int16_t)quat[5]) << 8);
    z = ((int16_t)quat[6]) | (((int16_t)quat[7]) << 8);

    abs_quaternion.w = ((float)w) / 16384.0; // 2^14 LSB
    abs_quaternion.x = ((float)x) / 16384.0;
    abs_quaternion.y = ((float)y) / 16384.0;
    abs_quaternion.z = ((float)z) / 16384.0;

    return abs_quaternion;
}

vector3f readEulerAngles() {
    vector3f euler_angles;
    uint8_t euler[6];
    uint8_t euler_reg = BNO055_EULER_H_LSB_REG;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, &euler_reg, 1, true);
    i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, euler, 6, false);

    /// @note heading = yaw
    int16_t heading, roll, pitch;
    heading = roll = pitch = 0;
    heading = ((int16_t)euler[0]) | (((int16_t)euler[1]) << 8);
    roll = ((int16_t)euler[2]) | (((int16_t)euler[3]) << 8);
    pitch = ((int16_t)euler[4]) | (((int16_t)euler[5]) << 8);

    euler_angles.x = ((float)roll) / 16.0;
    euler_angles.y = ((float)pitch) / 16.0;
    euler_angles.z = ((float)heading) / 16.0;

    return euler_angles;
}


