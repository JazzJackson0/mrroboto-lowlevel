#ifndef TASKS_H
#define TASKS_H
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "boards/pico_w.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "pwm.h"
#include "bno055.h"
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

void start_tasks();


#endif