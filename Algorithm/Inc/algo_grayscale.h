//
// Created by 戴境宏 on 2026/4/19.
//

#ifndef BALANCE_CAR_ALGO_GRAYSCALE_H
#define BALANCE_CAR_ALGO_GRAYSCALE_H
#pragma once

#include <stdint.h>

class AlgoGrayscale {
public:
    AlgoGrayscale();

    // 核心计算函数：传入8位灰度数据，返回平滑的物理误差
    float calculate_error(uint8_t sensor_data);

    // 清空历史记忆（小车跌倒重置、或手动切换模式时调用）
    void reset();

private:
    float last_error;   // 记忆上一次的有效误差（用于丢线保护）
    float weights[8];   // 8个探头的物理位置权重
};
#endif //BALANCE_CAR_ALGO_GRAYSCALE_H