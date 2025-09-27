#include "../include/encoder.h"


void initEncoder(struct encoder_data * encoder, float _ticks_per_rotation, float _distance_per_rotation, int wheel) {
    encoder->ticks_per_rotation = _ticks_per_rotation;
    encoder->distance_per_rotation = _distance_per_rotation;

    encoder->wheel = wheel;
    encoder->fwd_tick_count_l = 0;
    encoder->bkwd_tick_count_l = 0;
    encoder->fwd_tick_count_r = 0;
    encoder->bkwd_tick_count_r = 0;
}

void updateEncoder(struct encoder_data * encoder, uint8_t wheel_direction) {
    
    if (encoder->wheel == RIGHT && wheel_direction == WHEEL_FWD) {
        encoder->fwd_tick_count_r++;
    }
    else if (encoder->wheel == RIGHT && wheel_direction == WHEEL_BKWD) {
        encoder->bkwd_tick_count_r++;
    }
    else if (encoder->wheel == LEFT && wheel_direction == WHEEL_FWD) {
        encoder->fwd_tick_count_l++;
    }
    else if (encoder->wheel == LEFT && wheel_direction == WHEEL_BKWD) {
        encoder->bkwd_tick_count_l++;
    }
}

float getDistance(struct encoder_data * encoder) {
    
    if (encoder->wheel == RIGHT) {
        return encoder->distance_per_rotation 
            * ((encoder->fwd_tick_count_r - encoder->bkwd_tick_count_r) / encoder->ticks_per_rotation);
    }
    else if (encoder->wheel == LEFT) {
        return encoder->distance_per_rotation
            * ((encoder->fwd_tick_count_l - encoder->bkwd_tick_count_l) / encoder->ticks_per_rotation);
    }
}