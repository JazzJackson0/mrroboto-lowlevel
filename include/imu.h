#ifndef IMU_H
#define IMU_H
#include "bno055.h"

struct imu_data {
    vector3f linear_accelerations;
    vector3f rotational_velocities;
};

/**
 * @brief
 * @param imu
 */
void initImu(struct imu_data * imu);

/**
 * @brief
 * @param imu
 */
void updateImu(struct imu_data * imu);


#endif