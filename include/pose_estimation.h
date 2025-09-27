#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H

#include <inttypes.h>
#include "Quaternions.h"
#include "filters.h"
#include "utils.h"
#include "../CMSIS-DSP-main/Include/arm_math.h"
#include "imu.h"
#include "encoder.h"
#include "Odometry2Wheel.h"

// IIR Filter Settings
#define NUM_STAGES 2 // (4th Order Filter / 2) = 2
#define BLOCK_SIZE 25 
#define POST_SHIFT 5 
#define NUM_COEFFS 12

#define GLOBAL_QUATERNION_INIT (Quaternion) { \
    .q0 = 1, \
    .q1 = 0, \
    .q2 = 0, \
    .q3 = 0, \
}


struct velocities {
    vector3f translational;
    vector3f rotational;
};


/**
 * @brief 
 * @param timestep
 */
void initPoseEstimation(int timestep);

/**
 * @brief
 * @param encoder_left
 * @param encoder_right
 * @param odometry
 */
void encoder_GetRobotVelocities(struct encoder_data * encoder_left, struct encoder_data * encoder_right, RobotOdom * odometry);


/**
 * @brief
 * @param encoder_left
 * @param encoder_right
 * @param odometry
 */
void encoder_getRobotPosition(struct encoder_data * encoder_left, struct encoder_data * encoder_right, RobotOdom * odometry);

/**
 * @brief
 * @param imu
 * @param vels 
 * @return 
 */
void imu_GetRawVelocities(struct imu_data * imu, struct velocities * vels);

/**
 * @brief 
 * @param orientation
 * @param rotational_velocity
 * @return 
 */
void updateOrientation(Quaternion *orientation, vector3f rotational_velocity);


#endif