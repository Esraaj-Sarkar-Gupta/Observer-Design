/*
 * I2C Driver for the Raspberry Pi Pico, to be used
 * with the LSM9DS1 IMU.
 * 
 * Author: Esraaj Sarkar Gupta
 * Date: 18th February, 2026
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <stdint.h>

#include "include/i2c_driver.h"

/* ---- I2C Protocol Layout ---- */

#define I2C_PORT i2c0

#define SDA 0
#define SCL 1


/* ---- I2C Functions ---- */

void device_write_byte(
    uint8_t device_addr,// The device address (Gyro / Magnetometer)
    uint8_t registr,    // The register we wish to write to
    uint8_t value      // The value we wish to write
) {
    uint8_t buffer[2] = {registr, value};
    i2c_write_blocking(I2C_PORT,
        device_addr,
        buffer,
        2,
        false
    );
}

uint8_t device_read_byte(
    uint8_t device_addr,
    uint8_t registr
) {
    uint8_t value;

    // Start I2C connection and hold it open
    i2c_write_blocking(
        I2C_PORT,
        device_addr,
        &registr,
        1,
        true
    );
    
    // Read incoming I2C value
    i2c_read_blocking(
        I2C_PORT,
        device_addr,
        &value,
        1,
        false
    );

    return value;
}

void device_read_bytes(
    uint8_t device_addr,
    uint8_t registr,
    uint8_t* data,
    uint16_t length_of_data
) {
    i2c_write_blocking(
        I2C_PORT,
        device_addr,
        &registr,
        1,
        true
    );

    i2c_read_blocking(
        I2C_PORT,
        device_addr,
        data,
        length_of_data,
        false
    );
}

/* ---- I2C Helper Functions ---- */

uint16_t combine_bytes(uint8_t high, uint8_t low) {
    return ((uint16_t)high << 8) | (uint16_t)low;
}

