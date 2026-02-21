/*
 * I2C functions to interface with the LSM9DS1
 * IMU.
 * 
 * Author: Esraaj Sarkar Gupta
 * Date: 18th February, 2026
 */

#include "pico/stdlib.h"
#include <stdint.h>
#include <stdio.h>

#include "include/i2c_driver.h"
/* ---- Devices ---- */

#define LSM9DS1_ADDR_AG 0x6A
#define LSM9DS1_ADDR_M  0X1c

/* ---- Special Registers ---- */

/* -- WHO AM I --*/
#define WHO_AM_I 0x0F

#define EXPECTED_WHO_I_AM_AG    0x68
#define EXPECTED_WHO_I_AM_M     0x3D

/* -- Silicon Temperature -- */
#define THIS_TEMP_L 0x15
#define THIS_TEMP_H 0x16

/* ---- Gyro Registers ---- */
#define CTRL_REG1_G 0x10

/* -- Gyroscope Output Registers*/
#define OUT_X_L_G 0x18
#define OUT_X_H_G 0x19

#define OUT_Y_L_G 0x1A
#define OUT_Y_H_G 0x1B

#define OUT_Z_L_G 0x1C
#define OUT_Z_H_G 0x1D

/* -- Gryoscope DPS -- */
#define GYRO_DPS_PER_LSB 0.00875f

/* ---- Accelerometer Registers ---- */
#define CTRL_REG6_XL 0x20

/* -- Accelerometer Output Registers -- */
#define OUT_X_L_XL 0x28
#define OUT_X_H_XL 0x29

#define OUT_Y_L_XL 0x2A
#define OUT_Y_H_XL 0x2B

#define OUT_Z_L_XL 0x2C
#define OUT_Z_H_XL 0x2D

/* -- Accelerometer g -- */
#define ACCELEROMETER_G_PER_LSB 0.000732f
#define GRAVITY                 9.80665f

/* ---- Magnetometer Registers (M device @ LSM9DS1_ADDR_M) ---- */

#define CTRL_REG1_M      0x20
#define CTRL_REG2_M      0x21
#define CTRL_REG3_M      0x22
#define CTRL_REG4_M      0x23
#define CTRL_REG5_M      0x24

#define STATUS_REG_M     0x27

#define OUT_X_L_M        0x28
#define OUT_X_H_M        0x29
#define OUT_Y_L_M        0x2A
#define OUT_Y_H_M        0x2B
#define OUT_Z_L_M        0x2C
#define OUT_Z_H_M        0x2D

#define MAG_TESLA_PER_LSB_FS4  1.4e-8f   // ±4 gauss

/* Magnetometer: for multi-byte reads, datasheet uses sub-address MSB = 1 */
#define M_AUTO_INC(reg)  ((uint8_t)((reg) | 0x80u))

/* ---- Housekeeper Functions ---- */

int who_am_i(uint8_t device_addr, uint8_t registr) {
    if (device_addr != LSM9DS1_ADDR_AG && device_addr != LSM9DS1_ADDR_M) {
        return 2; // Unknown device error
    }

    if (registr != WHO_AM_I) {
        return 3; // Unknown register error
    }

    uint8_t v = device_read_byte(device_addr, registr);

    if (device_addr == LSM9DS1_ADDR_AG && v != EXPECTED_WHO_I_AM_AG) return 1;
    if (device_addr == LSM9DS1_ADDR_M  && v != EXPECTED_WHO_I_AM_M)  return 1;

    return 0;
}

float silicon_temperature() {
    uint8_t temperature_buffer[2];

    device_read_bytes(
        LSM9DS1_ADDR_AG,
        THIS_TEMP_L,
        temperature_buffer,
        2
    );

    int16_t raw_temperature =
            (int16_t)(((uint16_t)temperature_buffer[1] << 8) |
                    (uint16_t)temperature_buffer[0]);

    return 25.0f + ((float) raw_temperature /  16.0f);
}

/* ---- Gyroscope Reads ---- */

int enable_gyro() {
    device_write_byte(
        LSM9DS1_ADDR_AG,
        CTRL_REG1_G,
        0x60
    );

    return 0;
}

void read_gyroscope(float *gyro_data) {
    int8_t x_low = device_read_byte(
        LSM9DS1_ADDR_AG,
        OUT_X_L_G
    );

    int8_t x_high = device_read_byte(
        LSM9DS1_ADDR_AG,
        OUT_X_H_G
    );

    int8_t y_low = device_read_byte(
        LSM9DS1_ADDR_AG,
        OUT_Y_L_G
    );

    int8_t y_high = device_read_byte(
        LSM9DS1_ADDR_AG,
        OUT_Y_H_G
    );

    int8_t z_low = device_read_byte(
        LSM9DS1_ADDR_AG,
        OUT_Z_L_G
    );

    int8_t z_high = device_read_byte(
        LSM9DS1_ADDR_AG,
        OUT_Z_H_G
    );

    int16_t raw_x = (int16_t)combine_bytes(x_high, x_low);
    int16_t raw_y = (int16_t)combine_bytes(y_high, y_low);
    int16_t raw_z = (int16_t)combine_bytes(z_high, z_low);

    /* -- Map to degrees per second -- */

    float gx_dps = (float) raw_x * GYRO_DPS_PER_LSB;
    float gy_dps = (float) raw_y * GYRO_DPS_PER_LSB;
    float gz_dps = (float) raw_z * GYRO_DPS_PER_LSB;

    gyro_data[0] = gx_dps;
    gyro_data[1] = gy_dps;
    gyro_data[2] = gz_dps;

    return;
}

/* ---- Accelerometer Reads ---- */

int enable_accelerometer() {
    device_write_byte(
        LSM9DS1_ADDR_AG,
        CTRL_REG6_XL,
        0x68
    );

    return 0;
}

void read_accelerometer(float *accel_si)
{
    int8_t x_low  = device_read_byte(LSM9DS1_ADDR_AG, OUT_X_L_XL);
    int8_t x_high = device_read_byte(LSM9DS1_ADDR_AG, OUT_X_H_XL);

    int8_t y_low  = device_read_byte(LSM9DS1_ADDR_AG, OUT_Y_L_XL);
    int8_t y_high = device_read_byte(LSM9DS1_ADDR_AG, OUT_Y_H_XL);

    int8_t z_low  = device_read_byte(LSM9DS1_ADDR_AG, OUT_Z_L_XL);
    int8_t z_high = device_read_byte(LSM9DS1_ADDR_AG, OUT_Z_H_XL);

    int16_t raw_x = (int16_t)combine_bytes(x_high, x_low);
    int16_t raw_y = (int16_t)combine_bytes(y_high, y_low);
    int16_t raw_z = (int16_t)combine_bytes(z_high, z_low);

    accel_si[0] = (float)raw_x * ACCELEROMETER_G_PER_LSB * GRAVITY;
    accel_si[1] = (float)raw_y * ACCELEROMETER_G_PER_LSB * GRAVITY;
    accel_si[2] = (float)raw_z * ACCELEROMETER_G_PER_LSB * GRAVITY; 
}

/* ---- Magnetometer Reads ---- */

int enable_magnetometer(void)
{
    device_write_byte(LSM9DS1_ADDR_M, CTRL_REG3_M, 0x00); // Continuous conversion mode
    device_write_byte(LSM9DS1_ADDR_M, CTRL_REG4_M, 0x0C); // Magnetometer z is controlled by a different register (for some reason?)

    device_write_byte(LSM9DS1_ADDR_M, CTRL_REG1_M, 0x0C); // ODR + Temp Compensation

    device_write_byte(LSM9DS1_ADDR_M, CTRL_REG2_M, 0x00); // +- 4 Gauss

    return 0;
}

void read_magnetometer(float *mag_T)
{
    int8_t x_low  = device_read_byte(LSM9DS1_ADDR_M, OUT_X_L_M);
    int8_t x_high = device_read_byte(LSM9DS1_ADDR_M, OUT_X_H_M);

    int8_t y_low  = device_read_byte(LSM9DS1_ADDR_M, OUT_Y_L_M);
    int8_t y_high = device_read_byte(LSM9DS1_ADDR_M, OUT_Y_H_M);

    int8_t z_low  = device_read_byte(LSM9DS1_ADDR_M, OUT_Z_L_M);
    int8_t z_high = device_read_byte(LSM9DS1_ADDR_M, OUT_Z_H_M);

    int16_t raw_x = (int16_t)combine_bytes(x_high, x_low);
    int16_t raw_y = (int16_t)combine_bytes(y_high, y_low);
    int16_t raw_z = (int16_t)combine_bytes(z_high, z_low);

    mag_T[0] = (float)raw_x * MAG_TESLA_PER_LSB_FS4;
    mag_T[1] = (float)raw_y * MAG_TESLA_PER_LSB_FS4;
    mag_T[2] = (float)raw_z * MAG_TESLA_PER_LSB_FS4;

    return;
}
