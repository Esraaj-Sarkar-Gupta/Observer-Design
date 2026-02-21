#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"

// Your driver / IMU functions
#include "include/i2c_driver.h"
#include "include/imu.h"

// If these live in some imu.c, include its header instead.
// For now, we just declare the function you wrote:
float silicon_temperature(void);

static void core1_blink_task(void) {
    

    while (true) {
        tight_loop_contents();        
    }
}

static void i2c_bus_init(void) {
    i2c_init(I2C_PORT, 400 * 1000); // 400 kHz

    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);

    gpio_pull_up(SDA);
    gpio_pull_up(SCL);

    enable_gyro();
    enable_accelerometer();
    enable_magnetometer();
}

int main(void) {
    stdio_init_all();

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Give USB-serial a moment to come up
    sleep_ms(1500);
    printf("Booting...\n");

    i2c_bus_init();
    multicore_launch_core1(core1_blink_task);

    while (true) {
        gpio_put(LED_PIN, 1);
        float gyroscope_reading[3];
        float accelerometer_reading[3];
        float magnetometer_reading[3];

        read_gyroscope(gyroscope_reading);
        read_accelerometer(accelerometer_reading);
        read_magnetometer(magnetometer_reading);

        printf("%f, %f, %f; %f, %f, %f; %f, %f, %f;\n",
            gyroscope_reading[0],
            gyroscope_reading[1],
            gyroscope_reading[2],

            accelerometer_reading[0],
            accelerometer_reading[1],
            accelerometer_reading[2],

            magnetometer_reading[0],
            magnetometer_reading[1],
            magnetometer_reading[2]
        );

        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }
    return 0;
}
