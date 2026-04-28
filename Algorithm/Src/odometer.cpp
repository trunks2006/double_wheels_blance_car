//
// Created by 戴境宏 on 2026/4/28.
//
#include "odometer.h"

Odometer::Odometer() : distance_cm(0.0f) {}

void Odometer::reset() {
    distance_cm = 0.0f;
}

void Odometer::update(int16_t enc_l, int16_t enc_r) {
    // 1. 计算双轮平均脉冲增量 (代表车体中心的前进量)
    float delta_pulse = ((float)enc_l + (float)enc_r) / 2.0f;

    // 2. 转换为厘米并累加
    distance_cm += (delta_pulse / PULSES_PER_CM);
}

float Odometer::get_distance_cm() {
    return distance_cm;
}