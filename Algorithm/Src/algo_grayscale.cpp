//
// Created by 戴境宏 on 2026/4/19.
//
#include "algo_grayscale.h"

AlgoGrayscale::AlgoGrayscale() : last_error(0.0f) {
    // 初始化权重数组（假设探头从左到右依次对应 bit0 到 bit7）
    // 越靠外侧，权重绝对值越大。中心区域为 ±1。
    weights[0] = -4.0f; // 最左侧
    weights[1] = -3.0f;
    weights[2] = -2.0f;
    weights[3] = -1.0f; // 中间偏左
    weights[4] =  1.0f; // 中间偏右
    weights[5] =  2.0f;
    weights[6] =  3.0f;
    weights[7] =  4.0f; // 最右侧
}

void AlgoGrayscale::reset() {
    last_error = 0.0f;
}

float AlgoGrayscale::calculate_error(uint8_t sensor_data) {
    float weight_sum = 0.0f;
    int active_count = 0;

    // 1. 遍历 8 个探头，计算被激活探头的权重总和
    for (int i = 0; i < 8; i++) {
        if (sensor_data & (1 << i)) { // 如果第 i 个 bit 为 1（压黑线）
            weight_sum += weights[i];
            active_count++;
        }
    }

    // 2. 丢线保护逻辑 (非常重要！)
    // 当所有探头都没压线 (active_count == 0) 时，说明车子冲出去了
    if (active_count == 0) {
        if (last_error > 0.0f) {
            return 5.0f;  // 之前偏右，现在完全丢线，说明从右边冲出去了，输出极限值强行拉回
        } else if (last_error < 0.0f) {
            return -5.0f; // 之前偏左，输出极限负值拉回
        } else {
            return 0.0f;  // 一上电就没看到线
        }
    }

    // 3. 计算重心误差 (加权平均)
    float current_error = weight_sum / (float)active_count;

    // 4. 更新历史记忆
    last_error = current_error;

    return current_error;
}