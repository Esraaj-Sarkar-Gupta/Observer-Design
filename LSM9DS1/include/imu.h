#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>

/* =========================
 * LSM9DS1 I2C addresses (7-bit)
 * ========================= */
#define LSM9DS1_ADDR_AG 0x6A
#define LSM9DS1_ADDR_M  0x1C

/* =========================
 * Registers (A/G device)
 * ========================= */
#define WHO_AM_I        0x0F
#define THIS_TEMP_L     0x15
#define THIS_TEMP_H     0x16

#define CTRL_REG1_G     0x10
#define CTRL_REG6_XL    0x20

#define OUT_X_L_G       0x18
#define OUT_X_H_G       0x19
#define OUT_Y_L_G       0x1A
#define OUT_Y_H_G       0x1B
#define OUT_Z_L_G       0x1C
#define OUT_Z_H_G       0x1D

#define OUT_X_L_XL      0x28
#define OUT_X_H_XL      0x29
#define OUT_Y_L_XL      0x2A
#define OUT_Y_H_XL      0x2B
#define OUT_Z_L_XL      0x2C
#define OUT_Z_H_XL      0x2D

/* =========================
 * Expected IDs
 * ========================= */
#define EXPECTED_WHO_I_AM_AG 0x68
#define EXPECTED_WHO_I_AM_M  0x3D

/* =========================
 * Scale factors (your chosen configs)
 *
 * Gyro: ±245 dps -> 8.75 mdps/LSB = 0.00875 dps/LSB
 * Accel: ±16 g   -> 0.732 mg/LSB = 0.000732 g/LSB
 * ========================= */
#define GYRO_DPS_PER_LSB  0.00875f
#define ACCEL_G_PER_LSB   0.000732f
#define GRAVITY_SI        9.80665f

/* =========================
 * API
 * ========================= */

/*
 * who_am_i
 * Returns:
 *   0 = OK
 *   1 = WHO_AM_I mismatch
 *   2 = unknown device address
 *   3 = unknown register (only WHO_AM_I expected)
 */
int   who_am_i(uint8_t device_addr, uint8_t registr);

/* Reads temperature from AG device and converts to °C. */
float silicon_temperature(void);

/* Enable gyro (writes CTRL_REG1_G). Return 0 on success. */
int   enable_gyro(void);

/* Read gyro and return values in degrees/second (dps). gyro_dps[0..2] = X,Y,Z */
void  read_gyroscope(float *gyro_dps);

/* Enable accelerometer (writes CTRL_REG6_XL). Return 0 on success. */
int   enable_accelerometer(void);

/* Read acceleration and return SI units (m/s^2). accel_si[0..2] = X,Y,Z */
void  read_accelerometer(float *accel_si);

/* Enable magnetometer (writes CTRL_REG1,2,3). Returns 0 on success. */
int enable_magnetometer();

/* Read acceleration and return gauss units. */
void read_magnetometer(float *mag_T);

/* ---- Troubleshooting ---- */
void troubleshoot_magnetometer_status();


#ifdef __cplusplus
}
#endif

#endif /* IMU_H */