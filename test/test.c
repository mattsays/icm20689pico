#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "icm20689pico/icm20689pico.h"


// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1

icm20689* _icm20689;

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    _icm20689 = (icm20689*) malloc(sizeof(icm20689));

    printf("starting..\n");
    sleep_ms(5000);

    uint8_t error = 0;
    error = icm20689_init(_icm20689);

    bool toggle = true;
    double gyro[3] = {};
    double acc[3] = {};
    double orientation[2] = {};
    double temp;

    const int led = PICO_DEFAULT_LED_PIN;
    gpio_init(led);
    gpio_set_dir(led, GPIO_OUT);

    while(1) {
        toggle = !toggle;
        gpio_put(led, toggle);

        if(error != 0) {
            printf("Error code: %d \n" , error);
        } else {
            icm20689_read_gyroacc(_icm20689, gyro, acc);
            printf("Gyroscope > X: %lf, Y: %lf, Z: %lf \n", gyro[0], gyro[1], gyro[2]);
            printf("Accelerometer > X: %lf, Y: %lf, Z: %lf \n", acc[0], acc[1], acc[2]);
        }
        
        sleep_ms(100);
    }

    return 0;
}
