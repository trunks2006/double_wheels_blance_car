//
// Created by 戴境宏 on 2026/4/15.
//
#include "turn_pid.h"

TurnPid::TurnPid(float p, float i, float d, float max_o, float max_i)
    : PidBase(p, i, d, max_o, max_i) {}

float TurnPid::calc(float target, float current, float gyro_z) {
    float error = target - current;

    integral += error;
    if (integral > max_integral) integral = max_integral;
    else if (integral < -max_integral) integral = -max_integral;

    // 【转向环核心覆盖】：使用 Z 轴陀螺仪数据 (gyro_z) 作为 D 项阻尼。
    float out = kp * error + ki * integral - kd * gyro_z;

    if (out > max_out) out = max_out;
    else if (out < -max_out) out = -max_out;

    return out;
}