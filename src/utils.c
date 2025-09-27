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