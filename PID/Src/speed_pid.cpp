//
// Created by 戴境宏 on 2026/4/15.
//

#include "../Inc/speed_pid.h"
#include <math.h>

SpeedPid::SpeedPid(float p, float i, float d, float max_o, float max_i)
    : PidBase(p, i, d, max_o, max_i) {}

float SpeedPid::calc(float target, float current) {
    float error = target - current;

    // 【预留的工程优化】：速度环死区 (Deadband)
    // 如果速度误差极其微小(比如只有 0.5 个脉冲)，视作 0 误差，不累加积分，防止小车原地抽搐
    if (fabs(error) < 0.5f) {
        error = 0.0f;
    }

    // 直接复用基类极其可靠的数学公式
    // 因为 PidBase::calc 已经完美处理了积分、限幅和微分
    // 这里如果以后想改，直接把基类的公式抄过来魔改即可
    return PidBase::calc(target, current);
}
