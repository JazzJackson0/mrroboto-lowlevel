#include "../include/tasks.h"

void encoder_update_isr(uint pin_no, uint32_t event_flags);
void velocities_request_isr();
void pwm_receive_isr();
void timer_callback(TimerHandle_t xTimer);

void vUpdateEncoderTask(void *pvParameters);
void vUpdateIMUVelocitiesTask(void *pvParameters); 
void vSendEncoderVelocitiesTask(void *pvParameters);
void vSendIMUVelocitiesTask(void *pvParameters); 
void vReceivePWMDataTask(void *pvParameters);
void vPWMOutTask(void *pvParameters);

TaskHandle_t encoder_update_task_handle = NULL;
TaskHandle_t imu_update_task_handle = NULL;
TaskHandle_t encoder_vel_send_task_handle = NULL;
TaskHandle_t imu_vel_send_task_handle = NULL;
TaskHandle_t pwm_read_task_handle = NULL;
TaskHandle_t pwm_out_handle = NULL;
TaskHandle_t imu_timer_task_handle = NULL;

struct encoder_data left_encoder;
struct encoder_data right_encoder;
struct motor_data left_motor;
struct motor_data right_motor;
struct imu_data imu;
RobotOdom odometry;

uint pin_triggered; // Encoder Pin
volatile uint8_t motor_packet_type = 0;


/**
 * @brief Task: Updates encoder information
 * @param pvParameters
 * @return 
 */
void vUpdateEncodersTask(void *pvParameters) {

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (pin_triggered == ENCODER_R_INT_PIN_A && gpio_get(ENCODER_R_INT_PIN_B)) {
            updateEncoder(&right_encoder, WHEEL_FWD);
        }
        else if (pin_triggered == ENCODER_R_INT_PIN_A && !gpio_get(ENCODER_R_INT_PIN_B)) {
            updateEncoder(&right_encoder, WHEEL_BKWD);
        }
        else if (pin_triggered == ENCODER_L_INT_PIN_A && gpio_get(ENCODER_L_INT_PIN_B)) {
            updateEncoder(&left_encoder, WHEEL_FWD);
        }
        else if (pin_triggered == ENCODER_L_INT_PIN_B && !gpio_get(ENCODER_L_INT_PIN_B)) {
            updateEncoder(&left_encoder, WHEEL_BKWD);
        }
    }
}

/**
 * @brief Task: Updates IMU Velocity values
 * @param pvParameters
 * @return 
 */
void vUpdateIMUVelocitiesTask(void *pvParameters) {

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        updateImu(&imu);
        // printf("Trans Vels: " " x: %f" " y: %f\n", integral_x, integral_y);
    }
}


/**
 * @brief
 * @param pvParameters
 * @return 
 */
void vSendEncoderVelocitiesTask(void *pvParameters) {

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Master wants distance data
        if (i2c_get_hw(i2c1)->status & I2C_IC_STATUS_TFNF_BITS) {
            uint8_t MASK = 0xFF;
            uint8_t vels_buffer[DIST_BUFFER_SIZE];
            encoder_GetRobotVelocities(&left_encoder, &right_encoder, &odometry);
            uint32_t rotational_vel = (uint32_t) odometry.rotational_vel;
            uint32_t translational_vel = (uint32_t) odometry.translational_vel;
            
            vels_buffer[0] = (rotational_vel >> 24) & MASK;
            vels_buffer[1] = (rotational_vel >> 16) & MASK;
            vels_buffer[2] = (rotational_vel >> 8) & MASK;
            vels_buffer[3] = rotational_vel & MASK;
            vels_buffer[4] = (translational_vel>> 24) & MASK;
            vels_buffer[5] = (translational_vel>> 16) & MASK;
            vels_buffer[6] = (translational_vel>> 8) & MASK;
            vels_buffer[7] = translational_vel & MASK;
            i2c_write_raw_blocking(i2c1, vels_buffer, DIST_BUFFER_SIZE);
        }
    }
}

/**
 * @brief Task: Sends rotational and translational velocities over i2c.
 * @param pvParameters
 * @return 
 */
void vSendIMUVelocitiesTask(void *pvParameters) {

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // NOTE: MULTIPLE PRINT STATEMENTS CAUSE DATA TRANSFER ISSUES (MAYBE STACK OVERFLOW OR DEADLOCK OVER TASKS ACCESSING PRINTF SERIAL BUS)
        struct velocities * vels;
        imu_GetRawVelocities(&imu, vels);
        uint8_t velocities_buffer[VEL_BUFFER_SIZE];
        memcpy(&velocities_buffer[0], (uint32_t) vels->rotational.x, sizeof(uint32_t));
        memcpy(&velocities_buffer[4], (uint32_t) vels->rotational.y, sizeof(uint32_t));
        memcpy(&velocities_buffer[8], (uint32_t) vels->translational.x, sizeof(uint32_t));
        memcpy(&velocities_buffer[12], (uint32_t) vels->translational.y, sizeof(uint32_t));

        i2c_write_raw_blocking(i2c1, velocities_buffer, VEL_BUFFER_SIZE);
        // for (size_t i = 0; i < VEL_BUFFER_SIZE; i++) {
        //     while (!(i2c_get_hw(i2c1)->status & I2C_IC_STATUS_TFNF_BITS));  // Wait for FIFO space
        //     i2c_get_hw(i2c1)->data_cmd = velocities_buffer[i];  // Load next byte into FIFO
        //     // printf("Adding 0x%x. Valid Entries in FIFO: %d\n", velocities_buffer[i], i2c_get_hw(i2c1)->txflr);
        // }
    }
}

/**
 * @brief Receives data packets (containing pwm info) over uart
 * @param pvParameters
 * @return 
 */
void vReceivePWMDataTask(void *pvParameters) {

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t pwm_buffer[PWM_BUFFER_SIZE];
        // volatile size_t pwm_buff_idx = 0;

        // while (pwm_buff_idx < PWM_BUFFER_SIZE) {
        //     pwm_buffer[pwm_buff_idx++] = uart_getc(UART_ID);
        // }

        uart_read_blocking(UART_ID, pwm_buffer, PWM_BUFFER_SIZE);
        motor_packet_type = pwm_buffer[0];
        uint32_t max_period = 10000;

        if (motor_packet_type == DIRECTION_PACKET) {
            left_motor.direction = pwm_buffer[1];
            right_motor.direction = pwm_buffer[1];
        }

        else if (motor_packet_type == SPEED_PACKET) {
            memcpy(&left_motor.duty_cycle_percent, pwm_buffer[1], sizeof(float));
            memcpy(&right_motor.duty_cycle_percent, pwm_buffer[5], sizeof(float));
        }

        else if (motor_packet_type == FULL_PACKET) {
            left_motor.direction = pwm_buffer[1];
            right_motor.direction = pwm_buffer[1];
            memcpy(&left_motor.duty_cycle_percent, pwm_buffer[2], sizeof(float));
            memcpy(&right_motor.duty_cycle_percent, pwm_buffer[6], sizeof(float));
        }

        else if (motor_packet_type == QUAD_PACKET) {
        }

        xTaskNotify(pwm_out_handle, 0, eNoAction);
    }
}

/**
 * @brief Task: Updates motor direction and speeds based on global parameters
 * @param pvParameters
 * @return 
 */
void vPWMOutTask(void *pvParameters) {

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (motor_packet_type == DIRECTION_PACKET) {
            motorDirectionOut(&left_motor, &right_motor);
        }

        else if (motor_packet_type == SPEED_PACKET) {
            motorSpeedOut(&left_motor, &right_motor);
        }
        
        else if (motor_packet_type == FULL_PACKET) { 
            motorDirectionOut(&left_motor, &right_motor);
            motorSpeedOut(&left_motor, &right_motor);
        }

        else if (motor_packet_type == QUAD_PACKET) { 

        }
    }
}

/**
 * @brief
 * @param 
 * @return 
 */
void startTasks() {

    initOdometry(&odometry, TRACKWIDTH);

    // Setup Right Encoder 
    gpio_init(ENCODER_R_INT_PIN_A);
    gpio_init(ENCODER_R_INT_PIN_B);
    gpio_pull_up(ENCODER_R_INT_PIN_A);
    initEncoder(&right_encoder, TICKS_PER_ROTATION, DISTANCE_PER_ROTATION, RIGHT);
    // gpio_set_irq_enabled_with_callback(ENCODER_R_INT_PIN_A, GPIO_IRQ_LEVEL_LOW, true, &encoder_update_isr);

    // Setup Left Encoder
    gpio_init(ENCODER_L_INT_PIN_A);
    gpio_init(ENCODER_L_INT_PIN_B);
    gpio_pull_up(ENCODER_L_INT_PIN_A);
    initEncoder(&left_encoder, TICKS_PER_ROTATION, DISTANCE_PER_ROTATION, LEFT);
    // gpio_set_irq_enabled_with_callback(ENCODER_L_INT_PIN_A, GPIO_IRQ_LEVEL_LOW, true, &encoder_update_isr);

    // Setup Right Motor 
    gpio_init(MOTOR_R_DIR_1_PIN);
    gpio_init(MOTOR_R_DIR_2_PIN);
    gpio_set_dir(MOTOR_R_DIR_1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_R_DIR_2_PIN, GPIO_OUT);
    gpio_put(MOTOR_R_DIR_1_PIN, HIGH);
    gpio_put(MOTOR_R_DIR_2_PIN, LOW);
    initMotor(&right_motor, MOTOR_R_PIN, MOTOR_R_DIR_1_PIN, MOTOR_R_DIR_2_PIN);
    
    // Setup Left Motor
    gpio_init(MOTOR_L_DIR_1_PIN);
    gpio_init(MOTOR_L_DIR_2_PIN);
    gpio_set_dir(MOTOR_L_DIR_1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_L_DIR_2_PIN, GPIO_OUT);
    gpio_put(MOTOR_L_DIR_1_PIN, HIGH);
    gpio_put(MOTOR_L_DIR_2_PIN, LOW);
    initMotor(&left_motor, MOTOR_L_PIN, MOTOR_L_DIR_1_PIN, MOTOR_L_DIR_2_PIN);

    // // // Test Motor Output -------------------------------------------
    // right_motor.duty_cycle_percent = 50;
    // right_motor.direction_pin_1 = 1;
    // right_motor.direction_pin_2 = 0;
    // left_motor.duty_cycle_percent = 50;
    // left_motor.direction_pin_1 = 1;
    // left_motor.direction_pin_2 = 0;
    // motorSpeedOut(&left_motor, &right_motor);
    // // // End Test -----------------------------------------

    // // TEST 2!!!!!!!!!!!!!!!!!!!!------------
    // gpio_init(MOTOR_R_PIN);
    // gpio_init(MOTOR_L_PIN);
    // gpio_set_dir(MOTOR_R_PIN, GPIO_OUT);
    // gpio_set_dir(MOTOR_L_PIN, GPIO_OUT);
    // gpio_put(MOTOR_R_PIN, HIGH);
    // gpio_put(MOTOR_L_PIN, HIGH);
    // // End TEST ------------------------------


    // Setup Microprocessor PWM-Receiver Connection (UART)
    uart_init(UART_ID, UART_BAUD);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, UART_PARITY_NONE);
    gpio_set_function(UART_TX_GPIO, GPIO_FUNC_UART); // UART1 TX
    gpio_set_function(UART_RX_GPIO, GPIO_FUNC_UART); // UART1 RX
    uart_set_fifo_enabled(UART_ID, false); // (Clear FIFO Buffer)
    uart_set_fifo_enabled(UART_ID, true);
    uart_set_irq_enables(UART_ID, true, false); // RX Interrupt
    irq_set_exclusive_handler(UART1_IRQ, pwm_receive_isr);
    irq_set_enabled(UART1_IRQ, true);

    // Setup Microprocessor Odometry-Sender Connection (I2C1 Bus)
    i2c_init(i2c1, STANDARD_MODE);
    i2c_set_slave_mode(i2c1, true, I2C_SLAVE_ADDR & 0x7F);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    i2c_get_hw(i2c1)->intr_mask = I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_INTR_MASK_M_TX_ABRT_BITS | I2C_IC_INTR_MASK_M_TX_OVER_BITS;
    irq_set_exclusive_handler(I2C1_IRQ, velocities_request_isr);
    irq_set_enabled(I2C1_IRQ, true);

    // Setup IMU Sensor Connection (I2C0 Bus)
    i2c_init(i2c_default, FAST_MODE);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    initImu(&imu);

    initPoseEstimation(DT);

    TimerHandle_t xTimer = xTimerCreate("IMU Timer", pdMS_TO_TICKS(10) /*10ms to 10 ticks*/, pdTRUE, (void *)0, timer_callback);
    
    // Setup Tasks-----------------------------------

    // Updater Tasks
    // xTaskCreate(vUpdateEncoderTask, "Update Encoder Task", 256, NULL, 5, &encoder_update_task_handle);
    xTaskCreate(vUpdateIMUVelocitiesTask, "Update IMU Velocities Task", 256, NULL, 2, &imu_update_task_handle);

    // Sender Tasks
    // xTaskCreate(vSendEncoderVelocitiesTask, "Send Encoder Velocities Task", 256, NULL, 3, &encoder_vel_send_task_handle);
    xTaskCreate(vSendIMUVelocitiesTask, "Send IMU Velocities Task", 256, NULL, 3, &imu_vel_send_task_handle);
    
    // PWM Tasks
    xTaskCreate(vReceivePWMDataTask, "Receive PWM Data Task", 256, NULL, 2, &pwm_read_task_handle);
    xTaskCreate(vPWMOutTask, "PWM Out Task", 256, NULL, 2, &pwm_out_handle);

    if (xTimer != NULL) {
        xTimerStart(xTimer, 0);
    }

    vTaskStartScheduler();
}

// ISRs----------------------------------------------------------------------------------
/**
 * @brief ISR for 
 * @param 
 * @return 
 */
void encoder_update_isr(uint pin_no, uint32_t event_flags) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    pin_triggered = pin_no;
    vTaskNotifyGiveFromISR(encoder_update_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/**
 * @brief ISR for 
 * @param 
 * @return 
 */
void velocities_request_isr() {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t status = i2c_get_hw(i2c1)->intr_stat;

    // Master is reading from slave
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        (void)i2c_get_hw(i2c1)->clr_rd_req; // Clear interrupt
        vTaskNotifyGiveFromISR(imu_vel_send_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        // printf("!!!!!!!!!!!!!!!ABORT!!!!!!!!!!!!!!!!!!!!!\n");
        (void)i2c_get_hw(i2c1)->clr_tx_abrt;
    }
}

/**
 * @brief ISR for 
 * @param 
 * @return 
 */
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

/**
 * @brief
 * @param xTimer
 * @return 
 */
void timer_callback(TimerHandle_t xTimer) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(imu_update_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
