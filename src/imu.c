#include "../include/imu.h"



initImu(struct imu_data * imu) {

    initBno055();
    imu->linear_accelerations.x = 0;
    imu->linear_accelerations.y = 0;
    imu->linear_accelerations.z = 0;

    imu->rotational_velocities.x = 0;
    imu->rotational_velocities.y = 0;
    imu->rotational_velocities.z = 0;
}


updateImu(struct imu_data * imu) {
    imu->rotational_velocities = readRotVel();
    imu->linear_accelerations = readLinAccel();
}