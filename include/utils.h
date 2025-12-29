#ifndef UTILS_H
#define UTILS_H

#include <inttypes.h>
#include <stdlib.h>
#include "../CMSIS-DSP-main/Include/arm_math.h"


struct ring_buffer {
    uint8_t * buffer;
    size_t size;
    uint8_t producer_idx;
    uint8_t consumer_idx;
};


/**
 * @brief
 * @param queue
 * @param queue_size
 * @param val
 * @param idx
 * @return 
 */
void enqueue(float32_t queue[], int queue_size, float32_t val, int idx);

/**
 * @brief
 * @param buffer
 * @param raw_buffer
 * @param buffer_size
 */
void ringBufferInit(struct ring_buffer * buffer, uint8_t * raw_buffer, size_t buffer_size);

/**
 * @brief
 * @param buffer
 * @param value
 */
void ringBufferProduceUint8(struct ring_buffer * buffer, uint8_t value);

/**
 * @brief
 * @param buffer
 * @return data popped from the ring buffer 
 */
uint8_t ringBufferConsumeUint8(struct ring_buffer * buffer);


/**
 * @brief
 * @param buffer
 * @return size of available bytes in the ring buffer
 */
uint8_t ringBufferGetSizeAvailable(struct ring_buffer * buffer);

/**
 * @brief
 * @param buffer
 * @param dump
 * @return Number of bytes dumped 
 */
uint8_t ringBufferDump(struct ring_buffer * buffer, uint8_t * dump);


#endif