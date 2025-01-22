#include "../include/tasks.h"

void init_encoder_r(float _distance_per_tick, float _ticks_per_rotation);
void init_encoder_l(float _distance_per_tick, float _ticks_per_rotation);
void encoder_update_r();
void encoder_update_l();
float get_distance_r();
float get_distance_l();
void actuator_out(float left_duty_cycle, float right_duty_cycle);
void get_imu_position();
void update_imu_velocities();

// void vEncoderUpdateTask(void *pvParameters);
// void vSendEncoderDistanceDataTask(void *pvParameters);
void vSendIMUDistanceDataTask(void *pvParameters); 
void vReceivePWMDataTask(void *pvParameters);
void vPWMOutTask(void *pvParameters);
// void vUpdateIMUDataTask(void *pvParameters);
// void vUpdateLinearAccelTask(void *pvParameters);

// void encoder_update_isr(uint pin_no, uint32_t event_flags);
// void distance_request_isr();
void pwm_receive_isr();
void timer_callback(TimerHandle_t xTimer);
void linear_accel_timer_callback(TimerHandle_t xLinearAccelTimer);

int global_total_tick_count_r;
int total_tick_count_r;
int tick_count_r;
int ticks_per_rotation_r;
int rotation_count_r;
int distance_per_tick_r;
int global_total_tick_count_l;
int total_tick_count_l;
int tick_count_l;
int ticks_per_rotation_l;
int rotation_count_l;
int distance_per_tick_l;

#define DATA_RATE_HZ 100
#define MAX_SCL 400000
#define I2C_ADDR 0x055 // Default Slave Address
#define ENCODER_R_INT_PIN 15 // GP15
#define ENCODER_L_INT_PIN 16 // GP16
#define MOTOR_R_PIN 14 // GP14 
#define MOTOR_L_PIN  17 // GP17 
#define I2C_SDA_PIN 26 // GP26 (GPIO PIN #31)
#define I2C_SCL_PIN 27 // GP27 (GPIO PIN # 32)
#define UART_ID uart0

#define DIST_BUFFER_SIZE 8
#define VEL_BUFFER_SIZE 16
#define PWM_BUFFER_SIZE 8

volatile float left_duty = 0;
volatile float right_duty = 0;

volatile float integral_x = 0.f;
volatile float double_integral_x = 0.f;
volatile float integral_y = 0.f;
volatile float double_integral_y = 0.f;
volatile float prev_pos[2];
volatile float current_pos[2];
volatile float t_vels[2];
volatile float r_vels[2];

uint r_slice_num;
uint l_slice_num;
uint pin_triggered;

TaskHandle_t xTaskHandle = NULL;
TaskHandle_t encoder_dist_read_task_handle = NULL;
TaskHandle_t imu_dist_read_task_handle = NULL;
TaskHandle_t pwm_read_task_handle = NULL;
TaskHandle_t imu_timer_task_handle = NULL;
TaskHandle_t pwm_out_handle = NULL;
TaskHandle_t linear_accel_task_handle = NULL;

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

void vSendIMUDistanceDataTask(void *pvParameters) {

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Master wants distance data
        if (i2c_get_hw(i2c1)->status & I2C_IC_STATUS_TFNF_BITS) {

            // uint8_t dist_per_axis_buffer[DIST_BUFFER_SIZE];
            // float *pos_buffer = get_imu_position();
            // float x_dist = pos_buffer[0] - prev_pos[0];
            // float y_dist = pos_buffer[1] - prev_pos[1];
            // dist_per_axis_buffer[0] = (uint32_t) x_dist;
            // dist_per_axis_buffer[4] = (uint32_t) y_dist;
            // prev_pos[0] = pos_buffer[0];
            // prev_pos[1] = pos_buffer[1];
            // i2c_write_raw_blocking(i2c1, dist_per_axis_buffer, DIST_BUFFER_SIZE);

            uint8_t velocities_buffer[VEL_BUFFER_SIZE];
            update_imu_velocities();
            i2c_write_raw_blocking(i2c1, velocities_buffer, VEL_BUFFER_SIZE);

            // // Test
            // printf("Sensor Rotations: " " x: %f" " y: %f\n", r_vels[0], r_vels[1]);
            // // Test
            // printf("Sensor Trans: " " x: %f" " y: %f\n", t_vels[0], t_vels[1]);
            // memcpy(&velocities_buffer[0], r_vels, sizeof(r_vels));
            // memcpy(&velocities_buffer[8], t_vels, sizeof(t_vels));

            // // Test
            // printf("IMU Data To Send: " " rot x: %f" " rot y: %f" " trans x: %f" " trans y: %f\n\n",
            //     *(float*)&velocities_buffer[0], *(float*)&velocities_buffer[4], 
            //     *(float*)&velocities_buffer[8], *(float*)&velocities_buffer[12]);
        }
    }
}

void vReceivePWMDataTask(void *pvParameters) {

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t pwm_buffer[PWM_BUFFER_SIZE];
        volatile size_t pwm_buff_idx = 0;

        while (uart_is_readable(UART_ID) && pwm_buff_idx < PWM_BUFFER_SIZE) {
            pwm_buffer[pwm_buff_idx++] = uart_getc(UART_ID);
            

            if (pwm_buff_idx >= PWM_BUFFER_SIZE) {
                pwm_buff_idx = PWM_BUFFER_SIZE - 1;
            }
        }

        left_duty = (float) (((uint32_t)pwm_buffer[0] << 24) 
            | ((uint32_t)pwm_buffer[1] << 16) | ((uint32_t)pwm_buffer[2] << 8) | ((uint32_t)pwm_buffer[3]));
        right_duty = (float) (((uint32_t)pwm_buffer[4] << 24) 
            | ((uint32_t)pwm_buffer[5] << 16) | ((uint32_t)pwm_buffer[6] << 8) | ((uint32_t)pwm_buffer[7]));
        
        // Test
        printf("PWM: " " left duty: %f" " right duty: %f\n", left_duty, right_duty);
        
        xTaskNotify(pwm_out_handle, 0, eNoAction);
    }
}

void vPWMOutTask(void *pvParameters) {

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    actuator_out(left_duty, right_duty);
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

void start_tasks() {

    // PWM Setup
    r_slice_num = pwm_setup(MOTOR_R_PIN);
    l_slice_num = pwm_setup(MOTOR_L_PIN);
    uart_init(UART_ID, 115200);
    gpio_set_function(0, GPIO_FUNC_UART); // UART0 TX
    gpio_set_function(1, GPIO_FUNC_UART); // UART0 RX
    uart_set_irq_enables(UART_ID, true, false);
    irq_set_exclusive_handler(UART0_IRQ, pwm_receive_isr);
    irq_set_enabled(UART0_IRQ, true);


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
    i2c_init(i2c1, MAX_SCL);
    i2c_set_slave_mode(i2c1, true, I2C_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    i2c_get_hw(i2c1)->intr_mask = I2C_IC_INTR_MASK_M_RD_REQ_BITS;
    // irq_set_exclusive_handler(I2C1_IRQ, distance_request_isr);
    // irq_set_enabled(I2C1_IRQ, true);

    // Setup I2C0 Bus for the coms with IMU
    i2c_init(i2c_default, MAX_SCL);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    imu_init();
    TimerHandle_t xTimer = xTimerCreate("IMU Timer", pdMS_TO_TICKS(10) /*10ms to 10 ticks*/, pdTRUE, (void *)0, timer_callback);
    
    // Setup Tasks
    // xTaskCreate(vEncoderUpdateTask, "Encoder Update Task", 256, NULL, 5, &xTaskHandle);
    // xTaskCreate(vSendEncoderDistanceDataTask, "Send Encoder Distance Data Task", 256, NULL, 3, &encoder_dist_read_task_handle);
    xTaskCreate(vSendIMUDistanceDataTask, "Send IMU Distance Data Task", 256, NULL, 3, &imu_dist_read_task_handle);
    xTaskCreate(vReceivePWMDataTask, "Receive PWM Data Task", 256, NULL, 3, &pwm_read_task_handle);
    xTaskCreate(vPWMOutTask, "PWM Out Task", 256, NULL, 3, &pwm_out_handle);
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

// void distance_request_isr() {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     vTaskNotifyGiveFromISR(dist_read_task_handle, &xHigherPriorityTaskWoken);
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// }

void pwm_receive_isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(pwm_read_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
//------------------------------------------------------------------------------------------------------


// Timers----------------------------------------------------------------------------------
void timer_callback(TimerHandle_t xTimer) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // vTaskNotifyGiveFromISR(imu_timer_task_handle, &xHigherPriorityTaskWoken);
    vTaskNotifyGiveFromISR(imu_dist_read_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
//------------------------------------------------------------------------------------------------------


// Encoder Data---------------------------------------------------------------------------------
void encoder_update_r() {
    
    tick_count_r++;
    total_tick_count_r++;

    if (tick_count_r == ticks_per_rotation_r) {
        rotation_count_r++;
        tick_count_r = 0;
    }
}

void encoder_update_l() {
    
    tick_count_l++;
    total_tick_count_l++;

    if (tick_count_l == ticks_per_rotation_l) {
        rotation_count_l++;
        tick_count_l = 0;
    }
}

void init_encoder_r(float _distance_per_tick, float _ticks_per_rotation) {

    global_total_tick_count_r = 0;
    total_tick_count_r = 0;
    tick_count_r = 0; 
    global_total_tick_count_l = 0;
}

void init_encoder_l(float _distance_per_tick, float _ticks_per_rotation) {

    global_total_tick_count_l = 0;
    total_tick_count_l = 0;
    tick_count_l = 0; 
    rotation_count_l = 0;
    ticks_per_rotation_l = _ticks_per_rotation;
    distance_per_tick_l = _distance_per_tick;
}

float get_distance_r() {

    float ticks_during_time_period = total_tick_count_r - global_total_tick_count_r;
    global_total_tick_count_r = total_tick_count_r;
    return ticks_during_time_period * distance_per_tick_r;
}

float get_distance_l() {

    float ticks_during_time_period = total_tick_count_l - global_total_tick_count_l;
    global_total_tick_count_l = total_tick_count_l;
    return ticks_during_time_period * distance_per_tick_l;
}
//------------------------------------------------------------------------------------------------------


// IMU Data----------------------------------------------------------------------------------
void get_imu_position() {

    // TODO: Need to accomplish at a regular timestep (i.e. dt)
    vector3f linear_acceleration = read_lin_accel();
    integral_x += linear_acceleration.x;
    integral_y += linear_acceleration.y;
    double_integral_x += integral_x;
    double_integral_y += integral_y;
    current_pos[0] = double_integral_x;
    current_pos[1] = double_integral_y;
}


void update_imu_velocities() {

    // Update Rotational Vels
    vector3f rotational_velocity = read_rot_vel();
    r_vels[0] = rotational_velocity.x;
    r_vels[1] = rotational_velocity.y;

    // Update Translational Vels
    vector3f linear_acceleration = read_lin_accel();
    integral_x += linear_acceleration.x;
    integral_y += linear_acceleration.y;
    t_vels[0] = integral_x;
    t_vels[1] = integral_y;
    
    // Test
    // printf("Trans Vels: " " x: %f" " y: %f\n", integral_x, integral_y);

}
// --------------------------------------------------------------------------------------


// Actuator----------------------------------------------------------------------------------
void actuator_out(float left_duty_cycle, float right_duty_cycle) {
    
    pwm_update_duty_cycle(r_slice_num, right_duty_cycle);
    pwm_update_duty_cycle(l_slice_num, left_duty_cycle);
}
// --------------------------------------------------------------------------------------










