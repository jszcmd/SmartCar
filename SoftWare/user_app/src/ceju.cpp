#include "ceju.h"

// vl53l0x测距模块
//+mpu6050陀螺仪模块

static lq_i2c_vl53l0x &vl53l0x_dev()
{
    static lq_i2c_vl53l0x dev;
    return dev;
}

static lq_i2c_mpu6050 &mpu6050_dev()
{
    static lq_i2c_mpu6050 dev;
    return dev;
}

void ceju_loop(void)
{
    printf("VL53L0X distance = %05u\n\n", vl53l0x_dev().get_vl53l0x_dis());
}

void mpu6050_loop(void)
{
    int16_t ax, ay, az, gx, gy, gz;
    const uint8_t init_id = mpu6050_dev().get_mpu6050_id();

    if (init_id == 0)
    {
        printf("MPU6050 init check failed, id=0x00, please check driver/module/i2c wiring.\n");
        return;
    }
    if (!mpu6050_dev().get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz))
    {
        const uint8_t id_now = mpu6050_dev().get_mpu6050_id();
        printf("MPU6050 read failed, id=0x%02x\n\n", id_now);
        usleep(100 * 100);
        return;
    }
    printf("MPU6050 gyro: ax=%6d ay=%6d az=%6d gx=%6d gy=%6d gz=%6d\n\n", ax, ay, az, gx, gy, gz);
}