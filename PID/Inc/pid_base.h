//
// Created by 戴境宏 on 2026/4/15.
//

#ifndef BALANCE_CAR_PID_BASE_H
#define BALANCE_CAR_PID_BASE_H
#pragma once

class PidBase {
public:
    float kp, ki, kd;
    float max_out;      // 最终输出限幅
    float max_integral; // 积分限幅 (防风饱和)

    float integral;
    float last_error;

    PidBase(float p, float i, float d, float max_o, float max_i);
    virtual ~PidBase() = default;

    // 清空历史状态 (摔倒或切换状态时调用)
    virtual void reset();

    // 虚函数：标准的位置式 PID 计算公式
    virtual float calc(float target, float current);
};
#endif //BALANCE_CAR_PID_BASE_H