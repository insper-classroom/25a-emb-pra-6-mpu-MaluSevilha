#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include "Fusion.h"
#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
QueueHandle_t xQueuePos;

typedef struct {
    int id;
    int dados;
} mpu_t;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    // configuracao do I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;
    float acc_antigo = 0.0f;
    int last_yaw = 0.0f, last_roll = 0.0f;
    mpu_t info;
    float yaw_gyro = 0.0f;

    while(1) {
        // leitura da MPU, sem fusao de dados
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        }; 

        // printf("GYRO X: %0.0f| Y: %0.0f| Z: %0.0f \n", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);

        // printf("ACCEL X: %0.0f| Y: %0.0f| Z: %0.0f \n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
        
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        if (yaw_gyro == 0.0f){
            yaw_gyro -= gyroscope.axis.z * SAMPLE_PERIOD;
            last_yaw = yaw_gyro;
        } else {
            yaw_gyro += gyroscope.axis.z * SAMPLE_PERIOD;
        }

        // printf("YAW: %0.0f \n", euler.angle.yaw);
        
        if (last_yaw != yaw_gyro){
            info.id = 0;
            info.dados = -yaw_gyro;
            last_yaw = yaw_gyro;
            xQueueSend(xQueuePos, &info, 0);
        }

        if (euler.angle.roll != last_roll){
            info.id = 1;
            info.dados = -euler.angle.roll;
            last_roll = euler.angle.roll;
            xQueueSend(xQueuePos, &info, 0);
        }

        double acc = pow(accelerometer.axis.x, 2) + pow(accelerometer.axis.y, 2) + pow(accelerometer.axis.z, 2);
        acc = sqrt(acc);

        if ((acc - acc_antigo) > 0.65 && acc_antigo != 0.0f){
            info.id = 2;
            info.dados = 0;
            xQueueSend(xQueuePos, &info, 150);
        }

        acc_antigo = acc;

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void uart_task(void *p) {
    mpu_t data;

    while (1) {
        if (xQueueReceive(xQueuePos, &data, 100)){
            uint8_t bytes[4];
            bytes[0] = (uint8_t)(data.id);
            bytes[1] = (data.dados >> 8) & 0xFF;
            bytes[2] = data.dados & 0xFF;
            bytes[3] = 0xFF;

            uart_write_blocking(uart_default, bytes, 4);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


int main() {
    stdio_init_all();

    uart_init(uart_default, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_task", 4095, NULL, 1, NULL);
    xQueuePos = xQueueCreate(64, sizeof(mpu_t));

    vTaskStartScheduler();

    while (true)
        ;
}
