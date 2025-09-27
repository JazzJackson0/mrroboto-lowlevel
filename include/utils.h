#ifndef UTILS_H
#define UTILS_H

#include <inttypes.h>
#include "../CMSIS-DSP-main/Include/arm_math.h"




/**
 * @brief
 * @param queue
 * @param queue_size
 * @param val
 * @param idx
 * @return 
 */
void enqueue(float32_t queue[], int queue_size, float32_t val, int idx);





#endif