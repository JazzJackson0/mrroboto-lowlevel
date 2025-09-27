#include "../include/Odometry2Wheel.h"

void initOdometry(RobotOdom * r_odom, float trackwidth) {
    r_odom->trackwidth = trackwidth;
    r_odom->x = 0.0;
    r_odom->y = 0.0;
    r_odom->orientation = 0.0;
    r_odom->r_center = r_odom->trackwidth / 2;
}

void getOdometryPose(RobotOdom* r_odom, float left_dist, float right_dist) {
    
    float shiftAngle = (left_dist - right_dist) / r_odom->trackwidth; // How much robot shifts
    float orientation = r_odom->orientation * (3.14159265359 / 180); // Convert to radians

    // Get point of rotation
    float P_x = r_odom->x - (r_odom->r_center * cos(orientation));
    float P_y = r_odom->y - (r_odom->r_center * sin(orientation));

    // Get new position & orientation
    r_odom->x = P_x + (r_odom->r_center * cos(orientation + shiftAngle));
    r_odom->y = P_y + (r_odom->r_center * sin(orientation + shiftAngle));
    r_odom->orientation = (orientation + shiftAngle) * (180 / 3.14159265359); // Convert to degrees
}

/*
 * 			TO-DO
 * 			-----
 *  - Test Code
 *
 *  - 
 *  
 *  - 
 *  */