#include "../include/tasks.h"

static void init_encoder_r(float _distance_per_tick, float _ticks_per_rotation);
static void init_encoder_l(float _distance_per_tick, float _ticks_per_rotation);
static void encoder_update_r();
static void encoder_update_l();
static float get_distance_r();
static float get_distance_l();
static void actuator_out(float left_duty_cycle_percent, float right_duty_cycle_percent);
static void update_imu_velocities();
// void encoder_update_isr(uint pin_no, uint32_t event_flags);
static float complementary_filter(float gyro_old, float gyro_new, float accel);
static void update_global_orientation(vector3f rotational_velocity, vector3f linear_accel);
static void update_array(float32_t arr[], float32_t val, int array_size, int idx);
static float median_filter(float32_t vals[], float32_t val);

void distance_request_isr();
void pwm_receive_isr();
void timer_callback(TimerHandle_t xTimer);
void linear_accel_timer_callback(TimerHandle_t xLinearAccelTimer);

// void vEncoderUpdateTask(void *pvParameters);
// void vSendEncoderDistanceDataTask(void *pvParameters);
void vUpdateIMUDistanceDataTask(void *pvParameters); 
void vSendIMUDistanceDataTask(void *pvParameters); 
void vReceivePWMDataTask(void *pvParameters);
void vPWMOutTask(void *pvParameters);
// void vUpdateIMUDataTask(void *pvParameters);
// void vUpdateLinearAccelTask(void *pvParameters);

TaskHandle_t xTaskHandle = NULL;
TaskHandle_t encoder_dist_read_task_handle = NULL;
TaskHandle_t imu_dist_read_task_handle = NULL;
TaskHandle_t imu_dist_send_task_handle = NULL;
TaskHandle_t pwm_read_task_handle = NULL;
TaskHandle_t imu_timer_task_handle = NULL;
TaskHandle_t pwm_out_handle = NULL;
TaskHandle_t linear_accel_task_handle = NULL;

static volatile int global_total_tick_count_r;
static volatile int total_tick_count_r;
static volatile int tick_count_r;
static volatile int ticks_per_rotation_r;
static volatile int rotation_count_r;
static volatile int distance_per_tick_r;
static volatile int global_total_tick_count_l;
static volatile int total_tick_count_l;
static volatile int tick_count_l;
static volatile int ticks_per_rotation_l;
static volatile int rotation_count_l;
static volatile int distance_per_tick_l;

volatile float left_duty_percent = 0;
volatile float right_duty_percent = 0;

volatile float32_t integral_xs[BLOCK_SIZE] = {0};
volatile float32_t integral_ys[BLOCK_SIZE] = {0};
volatile int integral_count = 0;

volatile float prev_pos[2];
volatile float t_vels[2];
volatile float r_vels[2];


float filtered_accels_x[MEDIAN_FILTER_SIZE] = {0};
float filtered_accels_y[MEDIAN_FILTER_SIZE] = {0};

uint r_slice_num;
uint l_slice_num;
uint pin_triggered;

// IIR Filter Parameters
arm_biquad_casd_df1_inst_q31 filter_x;
arm_biquad_casd_df1_inst_q31 filter_y;
float32_t raw_coeffs[NUM_COEFFS] = {7.699098914706358e-09, 1.5398197571172107e-08, 7.699099026177254e-09, 
                      1.0, -1.9654195032783257, 0.9657687174143698, 1.0, 2.000000033541675,
                      0.9999999855215732, 1.0, -1.985324587198886, 0.9856773380490558};
q31_t coeffs[NUM_COEFFS] = {0};
q31_t state_x[2 * NUM_STAGES] = {0}; // Stores intermediate filter states during filtering (Where recursive iir filter stores past values)
q31_t state_y[2 * NUM_STAGES] = {0}; // Stores intermediate filter states during filtering (Where recursive iir filter stores past values)
bool filter = false;

Quaternion global_orientation;


// void vEncoderUpdateTask(void *pvParameters) {

//     for (;;) {
//         // uint32_t ulNotificationValue;
//         // ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//         if (pin_triggered == ENCODER_R_INT_PIN) {
//             encoder_update_r();
//         }
//         else {
//             encoder_update_l();
//         }
//     }
// }

// void vSendEncoderDistanceDataTask(void *pvParameters) {

//     for (;;) {
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//         // Master wants distance data
//         if (i2c_get_hw(i2c1)->status & I2C_IC_STATUS_TFNF_BITS) {
//             uint8_t dist_per_wheel_buffer[DIST_BUFFER_SIZE];
//             dist_per_wheel_buffer[0] = (uint32_t) get_distance_l();
//             dist_per_wheel_buffer[4] = (uint32_t) get_distance_r();
//             i2c_write_raw_blocking(i2c1, dist_per_wheel_buffer, DIST_BUFFER_SIZE);
//         }
//     }
// }



void vUpdateIMUDistanceDataTask(void *pvParameters) {

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        update_imu_velocities();
        // printf("Trans Vels: " " x: %f" " y: %f\n", integral_x, integral_y);
    }
}

void vSendIMUDistanceDataTask(void *pvParameters) {

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // NOTE: ALL THESE PRINT STATEMENTS CAUSE DATA TRANSFER ISSUES (MAYBE STACK OVERFLOW OR DEADLOCK OVER TASKS ACCESSING PRINTF SERIAL BUS)
        uint8_t velocities_buffer[VEL_BUFFER_SIZE];
        memcpy(&velocities_buffer[0], r_vels, sizeof(r_vels));
        memcpy(&velocities_buffer[8], t_vels, sizeof(t_vels));

        i2c_write_raw_blocking(i2c1, velocities_buffer, VEL_BUFFER_SIZE);
        // for (size_t i = 0; i < VEL_BUFFER_SIZE; i++) {
        //     while (!(i2c_get_hw(i2c1)->status & I2C_IC_STATUS_TFNF_BITS));  // Wait for FIFO space
        //     i2c_get_hw(i2c1)->data_cmd = velocities_buffer[i];  // Load next byte into FIFO
        //     // printf("Adding 0x%x. Valid Entries in FIFO: %d\n", velocities_buffer[i], i2c_get_hw(i2c1)->txflr);
        // }
    }
}

void vReceivePWMDataTask(void *pvParameters) {

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t pwm_buffer[PWM_BUFFER_SIZE];
        // volatile size_t pwm_buff_idx = 0;

        // while (pwm_buff_idx < PWM_BUFFER_SIZE) {
        //     pwm_buffer[pwm_buff_idx++] = uart_getc(UART_ID);
        // }

        uart_read_blocking(UART_ID, pwm_buffer, PWM_BUFFER_SIZE);

        // Test
        printf("%d, %d, %d, %d, %d, %d, %d, %d\n", pwm_buffer[0], pwm_buffer[1], pwm_buffer[2], 
                    pwm_buffer[3], pwm_buffer[4], pwm_buffer[5], pwm_buffer[6], pwm_buffer[7]);

        // left_duty_percent = (float) (((uint32_t)pwm_buffer[0] << 24) 
        //     | ((uint32_t)pwm_buffer[1] << 16) | ((uint32_t)pwm_buffer[2] << 8) | ((uint32_t)pwm_buffer[3]));
        // right_duty_percent = (float) (((uint32_t)pwm_buffer[4] << 24) 
        //     | ((uint32_t)pwm_buffer[5] << 16) | ((uint32_t)pwm_buffer[6] << 8) | ((uint32_t)pwm_buffer[7]));
        
        // // Test
        // printf("PWM: " " left duty: %f" " right duty: %f\n", left_duty_percent, right_duty_percent);
        
        
        // xTaskNotify(pwm_out_handle, 0, eNoAction);
    }
}

void vPWMOutTask(void *pvParameters) {

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        actuator_out(left_duty_percent, right_duty_percent);
    }
}


// void vUpdateIMUDataTask(void *pvParameters) {

//     for (;;) {
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

//         vector3f linear_acceleration = read_lin_accel();
//         printf("Linear Acceleration: " " x: %f" " y: %f" " z: %f\n",
//         linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);

//         quaternion abs_quaternion = read_abs_quaternion();
//         printf("Absolute Quaternion: " " w: %f" " x: %f" " y: %f" " z: %f\n",
//         abs_quaternion.w, abs_quaternion.x, abs_quaternion.y, abs_quaternion.z);

//         vector3f euler_angles = read_euler_angles();
//         printf("Euler Angles: " " Roll: %f" " Pitch: %f" " Yaw: %f\n",
//         euler_angles.x, euler_angles.y, euler_angles.z);

//         CALIB_STATUS calib_status = read_calib_status();
//         printf("Calibration Status: " " System: %d" " Gyro: %d" " Accel: %d" " Mag: %d\n",
//         calib_status.sys, calib_status.gyro, calib_status.accel, calib_status.mag);
//     }
// }



void startTasks() {

    global_orientation.q0 = 1;
    global_orientation.q1 = 0;
    global_orientation.q2 = 0;
    global_orientation.q3 = 0;

    // Setup IIR Filters (q31 is optimized for M0+ processors)
    arm_float_to_q31(raw_coeffs, coeffs, NUM_COEFFS);
    arm_biquad_cascade_df1_init_q31(&filter_x, NUM_STAGES, coeffs, state_x, POST_SHIFT);
    arm_biquad_cascade_df1_init_q31(&filter_y, NUM_STAGES, coeffs, state_y, POST_SHIFT);

    // PWM Setup
    r_slice_num = pwm_setup(MOTOR_R_PIN);
    l_slice_num = pwm_setup(MOTOR_L_PIN);

    // Setup UART for PWM comms from Microprocessor
    uart_init(UART_ID, UART_BAUD);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, UART_PARITY_NONE);
    gpio_set_function(UART_TX_GPIO, GPIO_FUNC_UART); // UART1 TX
    gpio_set_function(UART_RX_GPIO, GPIO_FUNC_UART); // UART1 RX
    uart_set_fifo_enabled(UART_ID, false); // (Clear FIFO Buffer)
    uart_set_fifo_enabled(UART_ID, true);
    uart_set_irq_enables(UART_ID, true, false); // RX Interrupt
    irq_set_exclusive_handler(UART1_IRQ, pwm_receive_isr);
    irq_set_enabled(UART1_IRQ, true);

    // Encoder Setup
    gpio_init(ENCODER_R_INT_PIN);
    gpio_pull_up(ENCODER_R_INT_PIN);
    init_encoder_r(0.001, 40);
    gpio_init(ENCODER_L_INT_PIN);
    gpio_pull_up(ENCODER_L_INT_PIN);
    init_encoder_l(0.001, 40);
    // gpio_set_irq_enabled_with_callback(ENCODER_R_INT_PIN, GPIO_IRQ_LEVEL_LOW, true, &encoder_update_isr);
    // gpio_set_irq_enabled_with_callback(ENCODER_L_INT_PIN, GPIO_IRQ_LEVEL_LOW, true, &encoder_update_isr);

    // Setup I2C1 Bus for the coms with microprocessor
    i2c_init(i2c1, STANDARD_MODE);
    i2c_set_slave_mode(i2c1, true, I2C_SLAVE_ADDR & 0x7F);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    i2c_get_hw(i2c1)->intr_mask = I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_INTR_MASK_M_TX_ABRT_BITS | I2C_IC_INTR_MASK_M_TX_OVER_BITS;
    irq_set_exclusive_handler(I2C1_IRQ, distance_request_isr);
    irq_set_enabled(I2C1_IRQ, true);


    // Setup I2C0 Bus for the coms with IMU
    i2c_init(i2c_default, FAST_MODE);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    imu_init();

    TimerHandle_t xTimer = xTimerCreate("IMU Timer", pdMS_TO_TICKS(10) /*10ms to 10 ticks*/, pdTRUE, (void *)0, timer_callback);
    
    // Setup Tasks
    // xTaskCreate(vEncoderUpdateTask, "Encoder Update Task", 256, NULL, 5, &xTaskHandle);
    // xTaskCreate(vSendEncoderDistanceDataTask, "Send Encoder Distance Data Task", 256, NULL, 3, &encoder_dist_read_task_handle);
    xTaskCreate(vUpdateIMUDistanceDataTask, "Update IMU Distance Data Task", 256, NULL, 2, &imu_dist_read_task_handle);
    xTaskCreate(vSendIMUDistanceDataTask, "Send IMU Distance Data Task", 256, NULL, 3, &imu_dist_send_task_handle);
    xTaskCreate(vReceivePWMDataTask, "Receive PWM Data Task", 256, NULL, 2, &pwm_read_task_handle);
    xTaskCreate(vPWMOutTask, "PWM Out Task", 256, NULL, 2, &pwm_out_handle);
    // xTaskCreate(vUpdateIMUDataTask, "Update IMU Data Task", 256, NULL, 1, &imu_timer_task_handle);

    if (xTimer != NULL) {
        xTimerStart(xTimer, 0);
    }

    vTaskStartScheduler();
}

// ISRs----------------------------------------------------------------------------------
// void encoder_update_isr(uint pin_no, uint32_t event_flags) {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     pin_triggered = pin_no;
//     vTaskNotifyGiveFromISR(xTaskHandle, &xHigherPriorityTaskWoken);
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// }



void distance_request_isr() {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t status = i2c_get_hw(i2c1)->intr_stat;

    // Master is reading from slave
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        (void)i2c_get_hw(i2c1)->clr_rd_req; // Clear interrupt
        vTaskNotifyGiveFromISR(imu_dist_send_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        printf("!!!!!!!!!!!!!!!ABORT!!!!!!!!!!!!!!!!!!!!!\n");
        (void)i2c_get_hw(i2c1)->clr_tx_abrt;
    }
}

void pwm_receive_isr() {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t status = uart_get_hw(uart1)->mis;

    if (status & UART_UARTMIS_RXMIS_BITS) {
        uart_get_hw(uart1)->icr |= UART_UARTICR_RXIC_BITS;
        vTaskNotifyGiveFromISR(pwm_read_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (status & UART_UARTMIS_RTMIS_BITS) {
        uart_get_hw(uart1)->icr |= UART_UARTICR_RTIC_BITS;
    }
    
}
//------------------------------------------------------------------------------------------------------


// Timers----------------------------------------------------------------------------------
void timer_callback(TimerHandle_t xTimer) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(imu_dist_read_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
//------------------------------------------------------------------------------------------------------


// Encoder Data---------------------------------------------------------------------------------
static void encoder_update_r() {
    
    tick_count_r++;
    total_tick_count_r++;

    if (tick_count_r == ticks_per_rotation_r) {
        rotation_count_r++;
        tick_count_r = 0;
    }
}

static void encoder_update_l() {
    
    tick_count_l++;
    total_tick_count_l++;

    if (tick_count_l == ticks_per_rotation_l) {
        rotation_count_l++;
        tick_count_l = 0;
    }
}

static void init_encoder_r(float _distance_per_tick, float _ticks_per_rotation) {

    global_total_tick_count_r = 0;
    total_tick_count_r = 0;
    tick_count_r = 0; 
    global_total_tick_count_l = 0;
}

static void init_encoder_l(float _distance_per_tick, float _ticks_per_rotation) {

    global_total_tick_count_l = 0;
    total_tick_count_l = 0;
    tick_count_l = 0; 
    rotation_count_l = 0;
    ticks_per_rotation_l = _ticks_per_rotation;
    distance_per_tick_l = _distance_per_tick;
}

static float get_distance_r() {

    float ticks_during_time_period = total_tick_count_r - global_total_tick_count_r;
    global_total_tick_count_r = total_tick_count_r;
    return ticks_during_time_period * distance_per_tick_r;
}

static float get_distance_l() {

    float ticks_during_time_period = total_tick_count_l - global_total_tick_count_l;
    global_total_tick_count_l = total_tick_count_l;
    return ticks_during_time_period * distance_per_tick_l;
}
//------------------------------------------------------------------------------------------------------


// IMU Data----------------------------------------------------------------------------------
static void update_imu_velocities() {

    static volatile float32_t accels_x_raw[BLOCK_SIZE] = {0};
    static volatile q31_t accels_x_in[BLOCK_SIZE] = {0};
    static volatile q31_t accels_x_out[BLOCK_SIZE] = {0};
    static volatile float32_t accels_x_filtered[BLOCK_SIZE] = {0};
    static volatile float32_t accels_y_raw[BLOCK_SIZE] = {0};
    static volatile q31_t accels_y_in[BLOCK_SIZE] = {0};
    static volatile q31_t accels_y_out[BLOCK_SIZE] = {0};
    static volatile float32_t accels_y_filtered[BLOCK_SIZE] = {0};
    static volatile float raw_integral_x = 0.f;
    static volatile float raw_integral_y = 0.f;
    static volatile int accels_count = 0;

    // Update Translational Vels
    vector3f linear_acceleration = readLinAccel();

    if (accels_count < BLOCK_SIZE)
        accels_count++;

    // Avoid accumulating error for near-zero accelerations
    // if (linear_acceleration.x > 0 && linear_acceleration.x < 1) { linear_acceleration.x = floor(linear_acceleration.x); }
    // else if (linear_acceleration.x > -1 && linear_acceleration.x < 0) { linear_acceleration.x = ceil(linear_acceleration.x); }
    // if (linear_acceleration.y > 0 && linear_acceleration.y < 1) { linear_acceleration.y = floor(linear_acceleration.y); }
    // else if (linear_acceleration.y > -1 && linear_acceleration.y < 0) { linear_acceleration.y = ceil(linear_acceleration.y); }

    // Update Rotational Vels
    vector3f rotational_velocity = readRotVel();
    r_vels[0] = rotational_velocity.x;
    r_vels[1] = rotational_velocity.y;

    // Rotate Accelerations into Global Space
    update_global_orientation(rotational_velocity, linear_acceleration);
    float global_acceleration[4] = {1, linear_acceleration.x, linear_acceleration.y, linear_acceleration.z};
    rotateVector(global_acceleration, global_orientation);

    // Filter the Accelerations
    // update_array(accels_x_raw, global_acceleration[1], BLOCK_SIZE, accels_count);
    // update_array(accels_y_raw, global_acceleration[2], BLOCK_SIZE, accels_count);
    update_array(accels_x_raw, linear_acceleration.x, BLOCK_SIZE, accels_count);
    update_array(accels_y_raw, linear_acceleration.y, BLOCK_SIZE, accels_count);
    if (accels_count >= BLOCK_SIZE) {
        arm_float_to_q31(accels_x_raw, accels_x_in, BLOCK_SIZE);
        arm_float_to_q31(accels_y_raw, accels_y_in, BLOCK_SIZE);
        arm_biquad_cascade_df1_q31(&filter_x, accels_x_in, accels_x_out, BLOCK_SIZE);
        arm_biquad_cascade_df1_q31(&filter_y, accels_y_in, accels_y_out, BLOCK_SIZE);
        arm_q31_to_float(accels_x_out, accels_x_filtered, BLOCK_SIZE);
        arm_q31_to_float(accels_y_out, accels_y_filtered, BLOCK_SIZE);
        
        raw_integral_x += accels_x_filtered[BLOCK_SIZE - 1] * DT;
        raw_integral_y += accels_x_filtered[BLOCK_SIZE - 1] * DT;
    }

    // // Test
    // printf("Accels: " " x: %f" " y: %f\n", linear_acceleration.x, linear_acceleration.y);
    // printf("Trans Vels: " " x: %f" " y: %f\n", integral_x, integral_y);
    // printf("Rotational Vels: " " x: %f" " y: %f\n", rotational_velocity.x, rotational_velocity.y);

}
// --------------------------------------------------------------------------------------

static void update_global_orientation(vector3f rotational_velocity, vector3f linear_accel) {
    
    // static float x_rot_prev = 0;
    // static float y_rot_prev = 0;
    // static float z_rot_prev = 0;
    // float x_rot = complementary_filter(x_rot_prev, rotational_velocity.x, linear_accel.x);
    // float y_rot = complementary_filter(y_rot_prev, rotational_velocity.y, linear_accel.y);
    // float z_rot = complementary_filter(z_rot_prev, rotational_velocity.z, linear_accel.z);
    // x_rot_prev = x_rot;
    // y_rot_prev = y_rot;
    // z_rot_prev = z_rot;

    // Convert Gyro Angles to Quaternion
    static Quaternion gyro_angles = {1, 0, 0, 0};
    float magnitude = sqrt((rotational_velocity.x * rotational_velocity.x) + 
        (rotational_velocity.y * rotational_velocity.y) + (rotational_velocity.z * rotational_velocity.z));    
    float magnitude_inv = 1 / magnitude;
    vector3f axis_of_rot = { ((rotational_velocity.x * DT) * magnitude_inv), 
        ((rotational_velocity.y * DT) * magnitude_inv), ((rotational_velocity.z * DT) * magnitude_inv)};

    gyro_angles.q0 = cos(magnitude * 0.5);
    gyro_angles.q1 = axis_of_rot.x * sin(magnitude * 0.5);
    gyro_angles.q2 = axis_of_rot.y * sin(magnitude * 0.5);
    gyro_angles.q3 = axis_of_rot.z * sin(magnitude * 0.5);

    // Update Global Orientation
    global_orientation = multiplyQuaternions(gyro_angles, global_orientation);
}

static float complementary_filter(float gyro_old, float gyro_new, float accel) {
    return (GYRO_WEIGHT * (gyro_old + (gyro_new * DT))) + (ACCEL_WEIGHT * accel);
}

// Actuator----------------------------------------------------------------------------------
static void actuator_out(float left_duty_cycle_percent, float right_duty_cycle_percent) {
    
    pwm_update_duty_cycle(r_slice_num, right_duty_cycle_percent);
    pwm_update_duty_cycle(l_slice_num, left_duty_cycle_percent);
}
// --------------------------------------------------------------------------------------

static void update_array(float32_t arr[], float32_t val, int array_size, int idx) {

    // Remove the oldest value
    if (idx > (array_size - 1)) {
        for (int i = 1; i < array_size; i++)
            arr[i - 1] = arr[i];
    }

    // Insert new value
    arr[idx] = val;
}

static float median_filter(float32_t vals[], float32_t val) {
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

    update_array(vals, val, MEDIAN_FILTER_SIZE, index);

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









