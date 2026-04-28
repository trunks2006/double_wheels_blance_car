//
// Created by 戴境宏 on 2026/4/19.
//
#include "algo_grayscale.h"

AlgoGrayscale::AlgoGrayscale() : last_error(0.0f) {
    // 权重配置保持不变
    weights[0] = -80.0f;
    weights[1] = -63.0f;
    weights[2] = -54.0f;
    weights[3] = -45.0f;
    weights[4] =  45.0f;
    weights[5] =  54.0f;
    weights[6] =  63.0f;
    weights[7] =  80.0f;
}

void AlgoGrayscale::reset() {
    last_error = 0.0f;
}

float AlgoGrayscale::calculate_error(uint8_t sensor_data) {
    float weight_sum = 0.0f;
    int active_count = 0;

    for (int i = 0; i < 8; i++) {
        if (sensor_data & (1 << i)) {
            weight_sum += weights[i];
            active_count++;
        }
    }

    // --- 极简误差计算逻辑 ---
    if (active_count == 0) {
        // 如果完全丢失黑线，原样返回上一次的误差（记忆特性）
        return last_error;
    }

    float current_error = weight_sum / (float)active_count;
    last_error = current_error;
    return current_error;
}