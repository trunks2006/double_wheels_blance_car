//
// Created by 戴境宏 on 2026/4/15.
//
#include "pid_base.h"

PidBase::PidBase(float p, float i, float d, float max_o, float max_i)
    : kp(p), ki(i), kd(d), max_out(max_o), max_integral(max_i), integral(0.0f), last_error(0.0f) {}

void PidBase::reset() {
    integral = 0.0f;
    last_error = 0.0f;
}

float PidBase::calc(float target, float current) {
    float error = target - current;

    // 1. 积分累加与限幅（如果实例化时 ki 传了 0，这里算出来也只是一直加 0，没有任何副作用）
    integral += error;
    if (integral > max_integral) integral = max_integral;
    else if (integral < -max_integral) integral = -max_integral;

    // 2. 默认的微分计算 (当前误差 - 上次误差)
    float d_term = error - last_error;
    last_error = error;

    // 3. 统一输出公式
    float out = kp * error + ki * integral + kd * d_term;

    // 4. 最终输出限幅
    if (out > max_out) out = max_out;
    else if (out < -max_out) out = -max_out;

    return out;
}