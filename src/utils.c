#include "../include/utils.h"


void enqueue(float32_t queue[], int queue_size, float32_t val, int idx) {

    if (idx > (queue_size - 1))
        return;

    // Dequeue the oldest value
    if (idx == (queue_size - 1)) {
        for (int i = 1; i < queue_size; i++)
            queue[i - 1] = queue[i];
    }

    // Enqueue new value
    queue[idx] = val;
}

void ringBufferInit(struct ring_buffer * buffer, uint8_t * raw_buffer, size_t buffer_size) {

    buffer->buffer = raw_buffer; 
    buffer->size = buffer_size;

    for (int i = 0; i < buffer_size; i++) {
        buffer->buffer[i] = 0;
    }
    buffer->producer_idx = 0;
    buffer->consumer_idx = 0;
}


void ringBufferProduceUint8(struct ring_buffer * buffer, uint8_t value) {

    buffer->buffer[buffer->producer_idx++] = value;

    // Wrap around if needed
    if (buffer->producer_idx >= buffer->size) {
        buffer->producer_idx = 0;
    }
}


uint8_t ringBufferConsumeUint8(struct ring_buffer * buffer) {

    // If consumer has caught up with producer
    if (buffer->consumer_idx == buffer->producer_idx) {
        // Return last valid data
        return buffer->buffer[buffer->consumer_idx - 1];
    }


    uint8_t value = buffer->buffer[buffer->consumer_idx++];
    
    // Wrap around if needed
    if (buffer->consumer_idx >= buffer->size) {
        buffer->consumer_idx = 0;
    }

    return value;
}