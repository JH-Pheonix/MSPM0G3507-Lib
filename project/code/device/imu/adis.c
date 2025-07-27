#include "adis.h"
#include "pin.h"

// 需要定义SPI相关函数，如果没有实现，暂时注释掉
#define SPI_Transfer(x) // TODO: 实现SPI传输函数

static uint16 spi_get_data(uint16);

uint8 adis_init(void)
{
    spi_init(ADIS_SPI, SPI_MODE3, ADIS_SPEED, ADIS_SCK, ADIS_MOSI, ADIS_MISO , SPI_CS_NULL);
    gpio_init(ADIS_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    adis_reset();
    system_delay_ms(20);
    spi_get_data(filter_set_high);
    system_delay_us(20);
    spi_get_data(filter_set_low);
    system_delay_ms(400);
    spi_get_data(0x7200); // 设备识别码
    system_delay_us(20);
    uint16 id = spi_get_data(0x7000); // 读取设备ID
    system_delay_us(20);

    if(id == 0x4079) 
    {
        return 0;
    }
    else return 1;
}

void adis_read_data(imu_data_t *data)
{
    uint16 gy_z_high, gy_z_low, gy_x_high, gy_x_low, gy_y_high, gy_y_low;
    uint16 acc_x_high, acc_x_low, acc_y_high, acc_y_low, acc_z_high, acc_z_low;
    uint16 temp;
    uint32 gy_x_raw, gy_y_raw, gy_z_raw, acc_x_raw, acc_y_raw, acc_z_raw;
    
    gy_z_high = spi_get_data(GYZ_LOW);    // 与ADIS16505通信获取数据
    system_delay_us(20);
    gy_z_low = spi_get_data(GYX_OUT);
    system_delay_us(20);
    gy_x_high = spi_get_data(GYX_LOW);
    system_delay_us(20);
    gy_x_low = spi_get_data(GYY_OUT);
    system_delay_us(20);
    gy_y_high = spi_get_data(GYY_LOW);
    system_delay_us(20);
    gy_y_low = spi_get_data(ACC_X_HIGH);
    system_delay_us(20);
    acc_x_high = spi_get_data(ACC_X_LOW);
    system_delay_us(20);
    acc_x_low = spi_get_data(ACC_Y_HIGH);
    system_delay_us(20);
    acc_y_high = spi_get_data(ACC_Y_LOW);
    system_delay_us(20);
    acc_y_low = spi_get_data(ACC_Z_HIGH);
    system_delay_us(20);
    acc_z_high = spi_get_data(ACC_Z_LOW);
    system_delay_us(20);
    acc_z_low = spi_get_data(TEMP);
    system_delay_us(20);
    temp = spi_get_data(GYZ_OUT);
    system_delay_us(20);
    
    // 合并为32位数据
    gy_x_raw = ((uint32)gy_x_high<<16) | gy_x_low;
    gy_y_raw = ((uint32)gy_y_high<<16) | gy_y_low;
    gy_z_raw = ((uint32)gy_z_high<<16) | gy_z_low;
    acc_x_raw = ((uint32)acc_x_high<<16) | acc_x_low;
    acc_y_raw = ((uint32)acc_y_high<<16) | acc_y_low;
    acc_z_raw = ((uint32)acc_z_high<<16) | acc_z_low;
    
    // 转换为具有物理意义的数据
    data->gyro_x = (float)(gy_x_raw * Kg / 65536);
    data->gyro_y = (float)(gy_y_raw * Kg / 65536);
    data->gyro_z = (float)(gy_z_raw * Kg / 65536);
    data->accel_x = (float)(acc_x_raw * ACC_Kg / 65536);
    data->accel_y = (float)(acc_y_raw * ACC_Kg / 65536);
    data->accel_z = (float)(acc_z_raw * ACC_Kg / 65536);
}

void adis_reset()
{
    SPI_Transfer(RST1);
    system_delay_us(20);
    SPI_Transfer(RST1);
    system_delay_us(20);
    SPI_Transfer(RST2);
}

static uint16 spi_get_data(uint16 data_out)
{
    uint16 resp;
    gpio_low(ADIS_CS_PIN);
    spi_transfer_16bit(ADIS_SPI, &data_out, &resp, 1);
    gpio_high(ADIS_CS_PIN);
    return resp;
}