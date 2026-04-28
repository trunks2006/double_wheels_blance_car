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

    // 计算偏差
    float calculate_error(uint8_t sensor_data);
    void reset();

private:
    float last_error;
    float weights[8];
};
#endif //BALANCE_CAR_ALGO_GRAYSCALE_H