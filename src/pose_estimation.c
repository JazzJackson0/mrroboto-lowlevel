#include "../include/pose_estimation.h"

volatile float32_t accels_x[BLOCK_SIZE] = {0};
volatile float32_t accels_y[BLOCK_SIZE] = {0};
volatile float32_t accels_x_filtered[BLOCK_SIZE] = {0};
volatile float32_t accels_y_filtered[BLOCK_SIZE] = {0};
struct filter * accel_filter_x;
struct filter * accel_filter_y;

Quaternion global_orientation;
int dt;

// IIR Filter Parameters
float32_t raw_coeffs[NUM_COEFFS] = {7.699098914706358e-09, 1.5398197571172107e-08, 7.699099026177254e-09, 
                                    1.0, -1.9654195032783257, 0.9657687174143698, 1.0, 2.000000033541675,
                                    0.9999999855215732, 1.0, -1.985324587198886, 0.9856773380490558};



void initPoseEstimation(int timestep) {
    dt = timestep;
    global_orientation = GLOBAL_QUATERNION_INIT;
    accel_filter_x = buildIirFilter(POST_SHIFT, NUM_STAGES, BLOCK_SIZE, NUM_COEFFS, raw_coeffs);
    accel_filter_y = buildIirFilter(POST_SHIFT, NUM_STAGES, BLOCK_SIZE, NUM_COEFFS, raw_coeffs);
}

void encoder_GetRobotVelocities(struct encoder_data * encoder_left, struct encoder_data * encoder_right, 
    RobotOdom * odometry) {

    float left_distance = getDistance(encoder_left);
    float right_distance = getDistance(encoder_right);
    float phi = left_distance - right_distance / odometry->trackwidth;
    odometry->rotational_vel = odometry->r_center/dt;
    odometry->translational_vel = phi/dt;
}

void encoder_getRobotPosition(struct encoder_data * encoder_left, struct encoder_data * encoder_right, 
    RobotOdom * odometry) {

    float left_distance = getDistance(encoder_left);
    float right_distance = getDistance(encoder_right);
   
    getOdometryPose(odometry, left_distance, right_distance);
}

void imu_GetRawVelocities(struct imu_data * imu, struct velocities * vels) {

    static volatile float raw_integral_x = 0.f;
    static volatile float raw_integral_y = 0.f;
    static volatile int accels_count = 0;

    if (accels_count < BLOCK_SIZE)
        accels_count++;

    // 1. Get Rotational Velocities
    vels->rotational = imu->rotational_velocities;

    // 2. Convert Linear Acceleration into Translational Velocities
        // 2-a. Rotate Accelerations into Global Space
    updateOrientation(&global_orientation, vels->rotational);
    float global_acceleration[4] = {1, imu->linear_accelerations.x, imu->linear_accelerations.y, imu->linear_accelerations.z};
    rotateVector(global_acceleration, global_orientation);

        // 2-b. Filter the Accelerations
    enqueue(accels_x, BLOCK_SIZE, imu->linear_accelerations.x, accels_count);
    enqueue(accels_y, BLOCK_SIZE, imu->linear_accelerations.y, accels_count);

    if (accels_count >= BLOCK_SIZE) {
        runIirFilter(accel_filter_x, accels_x, accels_x_filtered);
        runIirFilter(accel_filter_y, accels_y, accels_y_filtered);
        
        raw_integral_x += accels_x_filtered[BLOCK_SIZE - 1] * dt;
        raw_integral_y += accels_y_filtered[BLOCK_SIZE - 1] * dt;
        vels->translational.x = raw_integral_x;
        vels->translational.y = raw_integral_y;
    }
}

void updateOrientation(Quaternion * orientation, vector3f rotational_velocity) {
    
    // Convert Gyro Angles to Orientation Quaternion
    static Quaternion gyro_angles = {1, 0, 0, 0};

    float magnitude = sqrt((rotational_velocity.x * rotational_velocity.x) + 
        (rotational_velocity.y * rotational_velocity.y) + (rotational_velocity.z * rotational_velocity.z));    
    float magnitude_inv = 1 / magnitude;

    vector3f axis_of_rot = { ((rotational_velocity.x * dt) * magnitude_inv), 
        ((rotational_velocity.y * dt) * magnitude_inv), ((rotational_velocity.z * dt) * magnitude_inv)};

    gyro_angles.q0 = cos(magnitude * 0.5);
    gyro_angles.q1 = axis_of_rot.x * sin(magnitude * 0.5);
    gyro_angles.q2 = axis_of_rot.y * sin(magnitude * 0.5);
    gyro_angles.q3 = axis_of_rot.z * sin(magnitude * 0.5);

    // Update Global Orientation
    *orientation = multiplyQuaternions(gyro_angles, *orientation);
}