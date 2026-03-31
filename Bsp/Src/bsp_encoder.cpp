//
// Created by 戴境宏 on 2026/3/29.
//
#include "bsp_encoder.h"

// 构造函数
Encoder::Encoder(TIM_HandleTypeDef* htim) : encoder_htim(htim) {}

void Encoder::encoder_init()
{
    // 启动定时器的编码器模式，抓取全部通道(TI1和TI2)
    HAL_TIM_Encoder_Start(encoder_htim, TIM_CHANNEL_ALL);
    // 将计数器初始值清零
    __HAL_TIM_SET_COUNTER(encoder_htim, 0);
}

int16_t Encoder::encoder_get_delta_pulses()
{
    // 1. 读取当前定时器的计数值，并利用 int16_t 处理下溢出反转问题
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(encoder_htim);

    // 2. 每次读取后立刻清零
    __HAL_TIM_SET_COUNTER(encoder_htim, 0);

    return count;
}


extern "C"
{
    // 实例化左轮和右轮的编码器对象
    Encoder encoder_left(&htim3);
    Encoder encoder_right(&htim4);

    void EncoderInit(void)
    {
        encoder_left.encoder_init();
        encoder_right.encoder_init();
    }

    int16_t EncoderGetLeft(void)
    {
        return encoder_left.encoder_get_delta_pulses();
    }

    int16_t EncoderGetRight(void)
    {
        // 物理校正：由于面对面安装，必须取反一侧数据以统一方向逻辑
        return -encoder_right.encoder_get_delta_pulses();
    }
}