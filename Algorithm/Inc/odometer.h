//
// Created by 戴境宏 on 2026/4/28.
//

#ifndef BALANCE_CAR_ODOMETER_H
#define BALANCE_CAR_ODOMETER_H
#pragma once
#include <stdint.h>

class Odometer {
public:
    Odometer();

    void reset();                               // 清零里程
    void update(int16_t enc_l, int16_t enc_r);  // 传入编码器增量更新里程
    float get_distance_cm();                    // 获取当前累计行驶的厘米数

private:
    float distance_cm;                          // 累计行驶距离 (cm)
    // 换算常数：每厘米对应的脉冲数 (1540 / (6.5 * PI))
    const float PULSES_PER_CM = 75.42f;
};
#endif //BALANCE_CAR_ODOMETER_H