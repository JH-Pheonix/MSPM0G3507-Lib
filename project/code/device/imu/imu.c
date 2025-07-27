#include "imu.h"
#include "adis.h"

static imu_device_enum curr_device;
static bool imu_state = 1; // 表示初始化状态

void imu_init(imu_device_enum device)
{
    curr_device = device;

    switch (curr_device)
    {
    case IMU_DEVICE_660RA:
        imu_state = imu660ra_init();
        break;

    case IMU_DEVICE_660RB:
        imu_state = imu660rb_init();
        break;

    case IMU_DEVICE_963RA:
        imu_state = imu963ra_init();
        break;
    
    case IMU_DEVICE_ADIS16505:
        imu_state = adis_init();
        break;
    
    default:
        imu_state = 1; 
        break;
    }

    if(!imu_state) // 0 表示初始化成功, 1 表示初始化失败
    {
        // handler logic
        printf("IMU SUCCESS.\n");
    }
    else
    {
        // handler logic
        printf("IMU FAILED.\n");
    }
}

imu_data_t imu_get_data(void)
{
    imu_data_t data = {0}; // 初始化为0
    
    if (imu_state)
    {
        printf("IMU device not initialized.\n");
        return data; // 返回默认值
    }

    switch (curr_device)
    {
    case IMU_DEVICE_660RA:
        imu660ra_get_acc();
        imu660ra_get_gyro();

        data.accel_x = imu660ra_acc_x;
        data.accel_y = imu660ra_acc_y;
        data.accel_z = imu660ra_acc_z;
        
        data.gyro_x = imu660ra_gyro_x;
        data.gyro_y = imu660ra_gyro_y;
        data.gyro_z = imu660ra_gyro_z;
        break;

    case IMU_DEVICE_660RB:
        imu660rb_get_acc();
        imu660rb_get_gyro();

        data.accel_x = imu660rb_acc_x;
        data.accel_y = imu660rb_acc_y;
        data.accel_z = imu660rb_acc_z;

        data.gyro_x = imu660rb_gyro_x;
        data.gyro_y = imu660rb_gyro_y;
        data.gyro_z = imu660rb_gyro_z;
        break;

    case IMU_DEVICE_963RA:
        imu963ra_get_acc();
        imu963ra_get_gyro();

        data.accel_x = imu963ra_acc_x;
        data.accel_y = imu963ra_acc_y;
        data.accel_z = imu963ra_acc_z;

        data.gyro_x = imu963ra_gyro_x;
        data.gyro_y = imu963ra_gyro_y;
        data.gyro_z = imu963ra_gyro_z;
        break;

    case IMU_DEVICE_ADIS16505:
        adis_read_data(&data);
        break;

    default:
        printf("Unknown IMU device.\n");
        break;
    }
    
    return data;
}