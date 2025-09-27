#ifndef FILTERS_H
#define FILTERS_H

#include <stdbool.h>
#include "../CMSIS-DSP-main/Include/arm_math.h"
#include "utils.h"

// Median Filter Settings
#define MEDIAN_FILTER_SIZE 11

// Complementary Filter Settings
#define GYRO_WEIGHT 0.98
#define ACCEL_WEIGHT 1 - GYRO_WEIGHT

struct filter {
    q31_t * coeffs;
    int num_stages;
    int post_shift; /* Number of bits the output of each biquad filter stage is right-shifted to keep it within the valid Q31 range and avoid overflow.
                        Low Value (0-1): keeps output close to full Q31 resolution. But the internal values may overflow, causing distortion or wraparound
                        High Value (4+): reduces the risk of overflow. But you lose resolution (precision), and output amplitude becomes smaller.
                        - A larger POST_SHIFT reduces output magnitude, preventing overflow.
                        - A smaller POST_SHIFT keeps more precision, but risks overflow.
                    */
    int block_size; // Number of samples processed per function call
    int num_coeffs;
    arm_biquad_casd_df1_inst_q31 * engine;

    q31_t * intermediate_state;
    q31_t * intermediate_data_1;
    q31_t * intermediate_data_2;
};


/**
 * @brief 
 * @param post_shift
 * @param num_stages
 * @param block_size
 * @param num_coeffs
 * @param raw_coeffs
 * 
 */
struct filter * buildIirFilter(int post_shift, int num_stages, int block_size, int num_coeffs, float32_t * raw_coeffs);

/**
 * @brief 
 * @param flt
 */
void destroyIirFilter(struct filter * flt);

/**
 * @brief 
 * @param flt
 * @param data_in
 * @param data_out
 */
void runIirFilter(struct filter * flt, float32_t * data_in, float32_t * data_out);


/**
 * @brief
 * @param vals
 * @param val
 * @return 
 */
float medianFilter(float32_t vals[], float32_t val);


/**
 * @brief Run complementary filter
 * @param gyro_old
 * @param gyro_new
 * @param accel
 * @param dt
 * @return 
 */
float complementaryFilter(float gyro_old, float gyro_new, float accel, float dt);

#endif