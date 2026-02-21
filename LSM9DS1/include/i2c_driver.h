/*
 * I2C Driver for the Raspberry Pi Pico, to be used
 * with the LSM9DS1 IMU.
 *
 * Author: Esraaj Sarkar Gupta
 * Date: 18th February, 2026
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>
#include "hardware/i2c.h"   // for i2c_inst_t, i2c0

/* ---- I2C Protocol Layout ---- */

/* Default I2C instance + pins used by this driver */
#define I2C_PORT i2c0
#define SDA 0
#define SCL 1

/* ---- I2C Functions ---- */

void device_write_byte(
    uint8_t device_addr,   // 7-bit I2C device address
    uint8_t registr,       // register address inside device
    uint8_t value          // value to write
);

uint8_t device_read_byte(
    uint8_t device_addr,   // 7-bit I2C device address
    uint8_t registr        // register address inside device
);

void device_read_bytes(
    uint8_t device_addr,   // 7-bit I2C device address
    uint8_t registr,       // start register address
    uint8_t *data,         // output buffer
    uint16_t length_of_data
);

/* ---- I2C Helper Functions ---- */

uint16_t combine_bytes(uint8_t high, uint8_t low);

#endif // I2C_DRIVER_H
