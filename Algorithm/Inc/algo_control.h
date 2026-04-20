//
// Created by 戴境宏 on 2026/4/15.
//

#pragma once

#include <stdint.h>

#ifdef __cplusplus
#include "angle_pid.h"
#include "speed_pid.h"
#include "turn_pid.h"
#include "algo_grayscale.h"
class AlgoControl {
public:
    // ================= 核心成员：三大控制环 =================
    AnglePid pid_upright;  // 直立环
    SpeedPid pid_velocity; // 速度环
    TurnPid  pid_turn;     // 转向环
    AlgoGrayscale grayscale_algo; // 灰度处理大脑
    // ================= 状态成员：滤波器 =================
    float lpf_alpha;             // 低通滤波系数
    float speed_left_filtered;   // 左轮平滑速度
    float speed_right_filtered;  // 右轮平滑速度

    // 构造函数
    AlgoControl(float alpha);

    // 系统状态控制
    void init();
    void stop();

    // 核心大循环运算接口
    void update(float sys_pitch, float sys_pitch_rate, float sys_yaw_rate,
                int16_t enc_left, int16_t enc_right,uint8_t gray_data,
                int16_t& pwm_left, int16_t& pwm_right);
};
#endif

// ================= C 语言调用接口 (门面模式) =================
#ifdef __cplusplus
extern "C" {
#endif

    void AlgoInit(void);
    void AlgoStop(void);
    void AlgoUpdate(float sys_pitch, float sys_pitch_rate, float sys_yaw_rate,
                    int16_t enc_left, int16_t enc_right,uint8_t gray_data,
                    int16_t* pwm_left, int16_t* pwm_right);

#ifdef __cplusplus
}
#endif