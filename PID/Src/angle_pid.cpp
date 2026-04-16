//
// Created by 戴境宏 on 2026/4/15.
//
#include "angle_pid.h"

AnglePid::AnglePid(float p, float i, float d, float max_o, float max_i)
    : PidBase(p, i, d, max_o, max_i) {}

float AnglePid::calc(float target, float current, float gyro_rate) {
    float error = target - current;

    integral += error;
    if (integral > max_integral) integral = max_integral;
    else if (integral < -max_integral) integral = -max_integral;

    // 【直立环核心覆盖】：使用陀螺仪实测数据作为 D 项阻尼
    // 物理逻辑：车身往前倒(gyro_rate为正)时，阻尼要产生往后退的力(-)，故为减去
    float out = kp * error + ki * integral - kd * gyro_rate;

    if (out > max_out) out = max_out;
    else if (out < -max_out) out = -max_out;

    return out;
}