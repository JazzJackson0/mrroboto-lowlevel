#include "../include/filters.h"


float filtered_accels_x[MEDIAN_FILTER_SIZE] = {0};
float filtered_accels_y[MEDIAN_FILTER_SIZE] = {0};


struct filter * buildIirFilter(int post_shift, int num_stages, int block_size, int num_coeffs, float32_t * raw_coeffs) {
    struct filter *flt = (struct filter *) malloc(sizeof(struct filter));
    flt->coeffs = (q31_t *) calloc(num_coeffs, sizeof(q31_t));
    flt->intermediate_state = (q31_t *) calloc(2 * num_stages, sizeof(q31_t));
    flt->intermediate_data_1 = (q31_t *) calloc(block_size, sizeof(q31_t));
    flt->intermediate_data_2 = (q31_t *) calloc(block_size, sizeof(q31_t));
    flt->post_shift = post_shift;
    flt->num_stages = num_stages;
    flt->block_size = block_size;
    flt->num_coeffs = num_coeffs;

    // Setup IIR Filter (q31 is optimized for M0+ processors)
    arm_float_to_q31(raw_coeffs, flt->coeffs, flt->num_coeffs);
    arm_biquad_cascade_df1_init_q31(flt->engine, flt->num_stages, flt->coeffs, flt->intermediate_state, flt->post_shift);

    return flt;
}

void destroyIirFilter(struct filter * flt) {
    free(flt->coeffs);
    free(flt->intermediate_state);
    free(flt->intermediate_data_1);
    free(flt->intermediate_data_2);
    free(flt);
}

void runIirFilter(struct filter * flt, float32_t * data_in, float32_t * data_out) {
    arm_float_to_q31(data_in, flt->intermediate_data_1, flt->block_size);
    arm_biquad_cascade_df1_q31(flt->engine, flt->intermediate_data_1, flt->intermediate_data_2, flt->block_size);
    arm_q31_to_float(flt->intermediate_data_2, data_out, flt->block_size);
}


float complementaryFilter(float gyro_old, float gyro_new, float accel, float dt) {
    return (GYRO_WEIGHT * (gyro_old + (gyro_new * dt))) + (ACCEL_WEIGHT * accel);
}


float medianFilter(float32_t vals[], float32_t val) {
    // float32_t vals[BLOCK_SIZE];
    // memcpy(vals, arr, BLOCK_SIZE * sizeof(float32_t));
    // // Insertion Sort
    // for (int i = 0; i < BLOCK_SIZE; i++) {    
    //     for (int j = i; j > 0; j--) {
    //         if (vals[j] < vals[j - 1]) {
    //             float temp = vals[j - 1];
    //             vals[j - 1] = vals[j];
    //             vals[j] = temp;
    //         }
    //     }
    // }
    
    // return vals[BLOCK_SIZE / 2];

    static int index = 0;

    enqueue(vals, MEDIAN_FILTER_SIZE, val, index);

    if (index == MEDIAN_FILTER_SIZE)
        index = 0;

    // Maintain sorted order
    for (int j = index; j > 0; j--) {
        if (vals[j] < vals[j - 1]) {
            float temp = vals[j - 1];
            vals[j - 1] = vals[j];
            vals[j] = temp;
        }
    }
    

    index++;

    return index == MEDIAN_FILTER_SIZE? (vals[MEDIAN_FILTER_SIZE / 2]) : vals[0];
}



