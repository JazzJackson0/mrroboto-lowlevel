#include "../include/mpl3115a2.h"


/**
 * @brief Initialize the MPL3115A2 Altimeter settings
 * 
 */
void altimeter_init(float target_altitude) {

    uint8_t config[2] = {0};

    // Set [Active Mode], [Oversample Ratio = 16 i.e. 2^4], [Altimeter Mode]
    config[0] = MPL_CTRL_REG1;
    config[1] = 0xA1;
    i2c_write_blocking(i2c_default, ALTIMETER_ADDR, config, 2, true);

    // Enable Event Flag on New Pressure/Altitude Data
    config[0] = PT_DATA_CFG;
    config[1] = 0x07;
    i2c_write_blocking(i2c_default, ALTIMETER_ADDR, config, 2, true);

    
    // Interrupt Config-------------------------------------------------------
    // Store Target Altitude
    config[0] = MPL_PRESSURE_TARGET_MSB_REG;
    config[1] = (uint8_t) (((int16_t)(target_altitude)) >> 8);
    i2c_write_blocking(i2c_default, ALTIMETER_ADDR, config, 2, true);
    config[0] = MPL_PRESSURE_TARGET_LSB_REG;
    config[1] = (uint8_t) (((int16_t)(target_altitude)));
    i2c_write_blocking(i2c_default, ALTIMETER_ADDR, config, 2, true);

    // Enable Pressure Threshold Interrupt
    config[0] = MPL_INTRRUPT_ENABLE_REG;
    config[1] = 0x08;
    i2c_write_blocking(i2c_default, ALTIMETER_ADDR, config, 2, true);

    // Route Interrupt to INT1    
    config[0] = MPL_INTRRUPT_CFG_REG;
    config[1] = 0x08;
    i2c_write_blocking(i2c_default, ALTIMETER_ADDR, config, 2, true);

    // Set INT1 to Open Drain
    config[0] = MPL_CTRL_REG3;
    config[1] = 0x04;
    i2c_write_blocking(i2c_default, ALTIMETER_ADDR, config, 2, true);
}



/**
 * @brief Read altitude data from Altimeter (MPL3115A2)
 * 
 * @return float 
 */
float get_altitude() {
    uint8_t reg = OUT_P_MSB;
    uint8_t data[3];
    i2c_write_blocking(i2c_default, ALTIMETER_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, ALTIMETER_ADDR, data, 3, false);
    // Source (Datasheet pg. 21)
    return (float) (((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8)) * 0.0625;
}


/**
 * @brief Get change in altitude value from Altimeter (MPL3115A2)
 *          and calculate velocity.
 * 
 * @return float 
 */
float get_velocity() {
    uint8_t reg = OUT_P_DELTA_MSB;
    uint8_t data[3];
    i2c_write_blocking(i2c_default, ALTIMETER_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, ALTIMETER_ADDR, data, 3, false);
    float delta = (float) (((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8)) * 0.0625;
    return delta * PICO_DATA_RATE_HZ; // ??
}