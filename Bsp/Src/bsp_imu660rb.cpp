//
// Created by 戴境宏 on 2026/3/31.
//

#include "bsp_imu660rb.h"
#include "main.h"

// 构造函数
IMU660RB::IMU660RB(SPI_HandleTypeDef* hspi) : imu_hspi(hspi) {}

// ================= 底层通信封装 =================
void IMU660RB::write_reg(uint8_t reg, uint8_t data)
{
    // 写操作：寄存器地址最高位必须为 0 (reg & 0x7F)
    uint8_t tx_data[2] = {(uint8_t)(reg & 0x7F), data};

    // 1. 拉低片选 (CS)，唤醒传感器
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    // 2. 发送寄存器地址和数据
    HAL_SPI_Transmit(imu_hspi, tx_data, 2, HAL_MAX_DELAY);
    // 3. 拉高片选 (CS)，结束通信
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

void IMU660RB::read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{
    // 读操作：寄存器地址最高位必须为 1 (reg | 0x80)
    uint8_t tx_reg = reg | 0x80;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    // 发送要读取的起始地址
    HAL_SPI_Transmit(imu_hspi, &tx_reg, 1, HAL_MAX_DELAY);
    // 连续接收 len 个字节的数据
    HAL_SPI_Receive(imu_hspi, buf, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

// ================= 业务层封装 =================
void IMU660RB::imu_init()
{
    // 1. 软件复位 (推荐做法，防止热启动状态机卡死)
    write_reg(0x12, 0x01); // CTRL3_C 寄存器，触发 SW_RESET
    HAL_Delay(50);         // 等待复位完成

    // 2. 配置加速度计 (CTRL1_XL: 0x10)
    // 0x40 代表 104Hz 采样率, 2g 量程，此处更改的通信频率，改为了0x50,即208HZ
    write_reg(0x10, 0x50);

    // 3. 配置陀螺仪 (CTRL2_G: 0x11)
    // 0x4C 代表 104Hz 采样率, 2000 dps 量程,此处更改的通信频率，改为了0x5C,即208HZ
    write_reg(0x11, 0x5C);

    // 4. 配置控制寄存器 (CTRL3_C: 0x12)
    // 防撕裂与连续读取配置
    write_reg(0x12, 0x44);
}

void IMU660RB::imu_get_6axis(int16_t* acc, int16_t* gyro)
{
    uint8_t raw_data[12];

    // LSM6DSR 数据寄存器从 0x22 (OUTX_L_G) 开始，连续 12 个字节
    read_regs(0x22, raw_data, 12);

    // 拼装陀螺仪数据 (小端模式：低位在前，高位在后)
    gyro[0] = (int16_t)((raw_data[1] << 8) | raw_data[0]); // Gyro X
    gyro[1] = (int16_t)((raw_data[3] << 8) | raw_data[2]); // Gyro Y
    gyro[2] = (int16_t)((raw_data[5] << 8) | raw_data[4]); // Gyro Z

    // 拼装加速度数据
    acc[0] = (int16_t)((raw_data[7] << 8) | raw_data[6]);  // Acc X
    acc[1] = (int16_t)((raw_data[9] << 8) | raw_data[8]);  // Acc Y
    acc[2] = (int16_t)((raw_data[11] << 8) | raw_data[10]); // Acc Z
}

//---------------------------------------------------------
// C 接口实现
//---------------------------------------------------------
extern "C"
{
    // 实例化全局 IMU 对象，绑定至 SPI1 句柄
    IMU660RB board_imu(&hspi1);

    void ImuInit(void)
    {
        board_imu.imu_init();
    }

    void ImuGet6Axis(int16_t* acc_data, int16_t* gyro_data)
    {
        board_imu.imu_get_6axis(acc_data, gyro_data);
    }
}