//
// Created by 戴境宏 on 2026/3/31.
//

#ifndef BALANCE_CAR_BSP_IMU660RB_H
#define BALANCE_CAR_BSP_IMU660RB_H

#pragma once

#include <stdint.h>
#include "spi.h"

#ifdef __cplusplus
class IMU660RB
{
public:
    // 构造函数，传入 SPI 句柄
    IMU660RB(SPI_HandleTypeDef* hspi);

    // 初始化 IMU，配置加速度计和陀螺仪的量程与频率
    void imu_init();

    // 读取六轴原始数据 (3轴加速度 + 3轴角速度)
    void imu_get_6axis(int16_t* acc, int16_t* gyro);

private:
    SPI_HandleTypeDef* imu_hspi;

    // 底层 SPI 寄存器读写接口
    void write_reg(uint8_t reg, uint8_t data);
    void read_regs(uint8_t reg, uint8_t* buf, uint16_t len);
};
#endif


#ifdef __cplusplus
extern "C"
{
#endif

    void ImuInit(void);
    // acc_data 和 gyro_data 必须是长度为 3 的 int16_t 数组
    void ImuGet6Axis(int16_t* acc_data, int16_t* gyro_data);

#ifdef __cplusplus
}
#endif
#endif //BALANCE_CAR_BSP_IMU660RB_H