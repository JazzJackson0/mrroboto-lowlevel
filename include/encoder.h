#ifndef ENCODER_H
#define ENCODER_H

#include <inttypes.h>

#define RIGHT 1
#define LEFT 2
#define WHEEL_FWD 1
#define WHEEL_BKWD -1

struct encoder_data {
    int ticks_per_rotation;
    int distance_per_rotation;

    int wheel;

    volatile int fwd_tick_count_r;
    volatile int bkwd_tick_count_r;
    
    volatile int fwd_tick_count_l;
    volatile int bkwd_tick_count_l;
};

/**
 * @brief Initialize encoder for given wheel
 * @param encoder
 * @param _distance_per_tick
 * @param _ticks_per_rotation
 * @param wheel
 * @return 
 */
void initEncoder(struct encoder_data * encoder, float _ticks_per_rotation, float _distance_per_rotation, int wheel);

/**
 * @brief Update encoder measurements from given wheel
 * @param encoder
 * @param wheel_direction
 * @return 
 */
void updateEncoder(struct encoder_data * encoder, uint8_t wheel_direction);

/**
 * @brief Calculate distance traveled by given wheel
 * @param encoder
 * @return 
 */
float getDistance(struct encoder_data * encoder); 


#endif