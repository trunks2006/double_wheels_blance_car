//
// Created by 戴境宏 on 2026/4/15.
//

#ifndef BALANCE_CAR_ANGLE_PID_H
#define BALANCE_CAR_ANGLE_PID_H
#pragma once
#include "pid_base.h"

class AnglePid : public PidBase {
public:
    AnglePid(float p, float i, float d, float max_o, float max_i);

    // 针对角度环的特殊计算：接收外部物理角速度作为阻尼
    float calc(float target, float current, float gyro_rate);
};
#endif //BALANCE_CAR_ANGLE_PID_H