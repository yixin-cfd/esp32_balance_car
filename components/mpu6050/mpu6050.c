#include "mpu6050.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "MPU6050";

/**********************************************
函数名称：MPU_Init
函数功能：初始化MPU6050
函数参数：无
函数返回值：0,初始化成功  其他,初始化失败
**********************************************/
uint8_t MPU_Init(mpu6050_config_t *cfg)
{
    // i2c 初始化
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = cfg->sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = cfg->scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = cfg->freq,
    };
    ESP_ERROR_CHECK(i2c_param_config(MPU6050_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(MPU6050_I2C_PORT, conf.mode, 0, 0, 0));
    // mpu6050 初始化
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // 复位MPU6050
    vTaskDelay(pdMS_TO_TICKS(100));
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU6050
    MPU_Set_Gyro_Fsr(3);                     // 陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);                    // 加速度传感器,±2g
    MPU_Set_Rate(50);                        // 设置采样率50Hz
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);    // 关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);   // 关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效

    uint8_t res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if (res == MPU_ADDR) // 器件ID正确,即res = MPU_ADDR = 0x68
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
        MPU_Set_Rate(50);                        // 设置采样率为50Hz
    }
    else
        return 1; // 地址设置错误,返回1
    return 0;     // 地址设置正确,返回0
}

/**********************************************
函数名称：MPU_Set_Gyro_Fsr
函数功能：设置MPU6050陀螺仪传感器满量程范围
函数参数：fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
函数返回值：0,设置成功  其他,设置失败
**********************************************/
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); // 设置陀螺仪满量程范围
}

/**********************************************
函数名称：MPU_Set_Accel_Fsr
函数功能：设置MPU6050加速度传感器满量程范围
函数参数：fsr:0,±2g;1,±4g;2,±8g;3,±16g
函数返回值：0,设置成功  其他,设置失败
**********************************************/
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); // 设置加速度传感器满量程范围
}

/**********************************************
函数名称：MPU_Set_LPF
函数功能：设置MPU6050的数字低通滤波器
函数参数：lpf:数字低通滤波频率(Hz)
函数返回值：0,设置成功  其他,设置失败
**********************************************/
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data = 0;

    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU_Write_Byte(MPU_CFG_REG, data); // 设置数字低通滤波器
}

/**********************************************
函数名称：MPU_Set_Rate
函数功能：设置MPU6050的采样率(假定Fs=1KHz)
函数参数：rate:4~1000(Hz)  初始化中rate取50
函数返回值：0,设置成功  其他,设置失败
**********************************************/
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); // 设置数字低通滤波器
    return MPU_Set_LPF(rate / 2);                     // 自动设置LPF为采样率的一半
}

/**********************************************
函数名称：MPU_Get_Temperature
函数功能：得到温度传感器值
函数参数：无
函数返回值：温度值
**********************************************/
float MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    float temp;

    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((uint16_t)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp;
}

/**********************************************
函数名称：MPU_Get_Gyroscope
函数功能：得到陀螺仪值(原始值)
函数参数：gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
函数返回值：0,读取成功  其他,读取失败
**********************************************/
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    uint8_t buf[6], res;

    res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *gx = ((uint16_t)buf[0] << 8) | buf[1];
        *gy = ((uint16_t)buf[2] << 8) | buf[3];
        *gz = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
}

/**********************************************
函数名称：MPU_Get_Accelerometer
函数功能：得到加速度值(原始值)
函数参数：ax,ay,az:加速度传感器x,y,z轴的原始读数(带符号)
函数返回值：0,读取成功  其他,读取失败
**********************************************/
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    uint8_t buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *ax = ((uint16_t)buf[0] << 8) | buf[1];
        *ay = ((uint16_t)buf[2] << 8) | buf[3];
        *az = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
}

/**********************************************
函数名称：MPU_Write_Len
函数功能：IIC连续写(写器件地址、寄存器地址、数据)
函数参数：addr:器件地址      reg:寄存器地址
                 len:写入数据的长度  buf:数据区
函数返回值：0,写入成功  其他,写入失败
**********************************************/
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{

    // 创建 I2C 命令链
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    if (!i2c_cmd)
    {
        ESP_LOGE(TAG, "Error: I2C command link creation failed!");
        return ESP_FAIL;
    }

    // 开始 I2C 传输
    i2c_master_start(i2c_cmd);

    // 发送器件地址 (写模式)
    i2c_master_write_byte(i2c_cmd, (addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);

    // 发送寄存器地址
    i2c_master_write_byte(i2c_cmd, reg, I2C_MASTER_ACK);

    // 写入数据
    for (int i = 0; i < len; i++)
    {
        i2c_master_write_byte(i2c_cmd, buf[i], I2C_MASTER_ACK);
    }

    // 停止信号
    i2c_master_stop(i2c_cmd);

    // 执行 I2C 命令
    esp_err_t ret = i2c_master_cmd_begin(MPU6050_I2C_PORT, i2c_cmd, pdMS_TO_TICKS(1000));

    // 释放命令链
    i2c_cmd_link_delete(i2c_cmd);

    // 返回结果
    return ret;
}

/**********************************************
函数名称：MPU_Read_Len
函数功能：IIC连续读(写入器件地址后,读寄存器地址、数据)
函数参数：addr:器件地址        reg:要读的寄存器地址
                 len:要读取的数据长度  buf:读取到的数据存储区
函数返回值：0,读取成功  其他,读取失败
**********************************************/
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    if (!i2c_cmd)
    {
        ESP_LOGE(TAG, "Error i2c_cmd creat fail!");
        return ESP_FAIL;
    }
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(i2c_cmd, reg, I2C_MASTER_ACK);

    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
    for (int i = 0; i < len; i++)
    {
        if (i == len - 1)
            i2c_master_read_byte(i2c_cmd, &buf[i], I2C_MASTER_NACK);
        else
            i2c_master_read_byte(i2c_cmd, &buf[i], I2C_MASTER_ACK);
    }
    i2c_master_stop(i2c_cmd);
    esp_err_t ret = i2c_master_cmd_begin(MPU6050_I2C_PORT, i2c_cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(i2c_cmd);
    return ret;
}

/**********************************************
函数名称：MPU_Write_Byte
函数功能：IIC写一个字节
函数参数：data:写入的数据    reg:要写的寄存器地址
函数返回值：0,写入成功  其他,写入失败
**********************************************/
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // 在执行i2c之前，必须执行此函数 创建一个i2c 命令 链接，为之后的i2c操作执行，在执行完成之后需要销毁
    if (!cmd)
    {
        ESP_LOGE(TAG, "Error i2c cmd creat fail!");
        return ESP_FAIL;
    }
    i2c_master_start(cmd); // i2c运行开始函数。注意这不是真的开始，只是是一个开始标记，初始化cmd
    // 以下是i2c真正的读写操作
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);     // 查找代写设备
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);                                  // 设备内存地址
    i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);                                 // 设备内存存放数据
    i2c_master_stop(cmd);                                                             // i2c停止运行。并不是真正的停止，因为此时i2c还没有真正的运行，我认为这是一个标识，当时i2c运行的时候读取到此标志就停止运行。
    esp_err_t ret = i2c_master_cmd_begin(MPU6050_I2C_PORT, cmd, pdMS_TO_TICKS(1000)); // 按照cmd中的记录的操作顺序开始运行i2c （start-> write BH1750 地址 -> write BH1750_CMD  -> stop）
    i2c_cmd_link_delete(cmd);                                                         // 操作完成 删除cmd
    return ret;
}

/**********************************************
函数名称：MPU_Read_Byte
函数功能：IIC读一个字节
函数参数：reg:要读的寄存器地址
函数返回值：res:读取到的数据
**********************************************/
uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // 在执行i2c之前，必须执行此函数 创建一个i2c 命令 链接，为之后的i2c操作执行，在执行完成之后需要销毁
    if (!cmd)
    {
        ESP_LOGE(TAG, "Error i2c cmd creat fail!");
        return ESP_FAIL;
    }
    i2c_master_start(cmd); // i2c运行开始函数。注意这不是真的开始，只是是一个开始标记，初始化cmd
    // 以下是i2c真正的读写操作
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK); // 查找代写设备
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);                              // 设备内存地址
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);      // 查找代读设备
    i2c_master_read_byte(cmd, &data, I2C_MASTER_ACK);                                 // 设备内存存放数据
    i2c_master_stop(cmd);                                                             // i2c停止运行。并不是真正的停止，因为此时i2c还没有真正的运行，我认为这是一个标识，当时i2c运行的时候读取到此标志就停止运行。
    esp_err_t ret = i2c_master_cmd_begin(MPU6050_I2C_PORT, cmd, pdMS_TO_TICKS(1000)); // 按照cmd中的记录的操作顺序开始运行i2c （start-> write BH1750 地址 -> write BH1750_CMD  -> stop）
    i2c_cmd_link_delete(cmd);                                                         // 操作完成 删除cmd
    return ret;
}
