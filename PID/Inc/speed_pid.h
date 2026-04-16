//
// Created by 戴境宏 on 2026/4/15.
//

#ifndef BALANCE_CAR_SPEED_PID_H
#define BALANCE_CAR_SPEED_PID_H

#pragma once
#include "pid_base.h"

class SpeedPid : public PidBase {
public:
    SpeedPid(float p, float i, float d, float max_o, float max_i);

    // 重写速度环特有的计算逻辑
    float calc(float target, float current) override;
};

#endif //BALANCE_CAR_SPEED_PID_H