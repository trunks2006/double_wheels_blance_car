//
// Created by 戴境宏 on 2026/3/29.
//

#ifndef BALANCE_CAR_BSP_ENCODER_H
#define BALANCE_CAR_BSP_ENCODER_H

#pragma once

#include <stdint.h>
#include "tim.h"

#ifdef __cplusplus
class Encoder
{
public:
    // 构造函数，传入对应的定时器句柄指针
    Encoder(TIM_HandleTypeDef* htim);

    // 初始化硬件定时器的编码器模式
    void encoder_init();

    // 获取自上次读取以来的脉冲增量（即瞬时速度）
    int16_t encoder_get_delta_pulses();

private:
    TIM_HandleTypeDef* encoder_htim;
};
#endif


#ifdef __cplusplus
extern "C"
{
#endif

    void EncoderInit(void);
    int16_t EncoderGetLeft(void);
    int16_t EncoderGetRight(void);

#ifdef __cplusplus
}

#endif

#endif //BALANCE_CAR_BSP_ENCODER_H