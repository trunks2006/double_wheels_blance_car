//
// Created by 戴境宏 on 2026/4/15.
//

#ifndef BALANCE_CAR_TURN_PID_H
#define BALANCE_CAR_TURN_PID_H
#pragma once
#include "pid_base.h"

class TurnPid : public PidBase {
public:
    TurnPid(float p, float i, float d, float max_o, float max_i);

    // 针对转向的特殊设计：融合外部误差(如航向角/灰度)和内部 Z 轴角速度(阻尼)
    float calc(float target, float current, float gyro_z);
};
#endif //BALANCE_CAR_TURN_PID_H