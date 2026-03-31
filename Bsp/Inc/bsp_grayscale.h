//
// Created by 戴境宏 on 2026/3/31.
//

#ifndef BALANCE_CAR_BSP_GRAYSCALE_H
#define BALANCE_CAR_BSP_GRAYSCALE_H
#pragma once

#include <stdint.h>

#ifdef __cplusplus
class Grayscale
{
public:
    // 构造函数
    Grayscale();

    // 初始化灰度传感器模块
    void grayscale_init();

    // 扫描所有 8 个探头，将结果打包成 1 个字节 (8 bit) 返回
    uint8_t grayscale_read_all();

private:
    // 底层硬件接口：向 MUX 芯片发送 0~7 的二进制地址码 (严格参考官方位移逻辑)
    void set_mux_channel(uint8_t channel);

    // 内部微秒级延时函数 (参考官方 delay_us 逻辑)
    void delay_us(uint32_t us);
};
#endif


#ifdef __cplusplus
extern "C"
{
#endif

    void GrayscaleInit(void);
    uint8_t GrayscaleReadAll(void);

#ifdef __cplusplus
}
#endif
#endif //BALANCE_CAR_BSP_GRAYSCALE_H