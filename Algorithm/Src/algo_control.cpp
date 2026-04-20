 //
// Created by 戴境宏 on 2026/4/15.
//
#include "algo_control.h"

// 构造函数：初始化三大环参数与滤波器状态
AlgoControl::AlgoControl(float alpha)
    : pid_upright(-1680.0f, 0.0f, -35.0f, 8200.0f, 0.0f),   // 直立环：纯 PD，限幅 8200
      pid_velocity(0.35f, 0.003f, 0.0f, 20.0f, 1800.0f), // 速度环：纯 PI，角度限幅 15度
      pid_turn(0.0f, 0.0f, 0.0f, 3000.0f, 0.0f),      // 转向环：暂不启用，限幅 3000
      lpf_alpha(alpha),
      speed_left_filtered(0.0f),
      speed_right_filtered(0.0f) {}

void AlgoControl::init() {
    pid_upright.reset();
    pid_velocity.reset();
    pid_turn.reset();
    speed_left_filtered = 0.0f;
    speed_right_filtered = 0.0f;
}

void AlgoControl::stop() {
    pid_upright.reset();
    pid_velocity.reset();
    pid_turn.reset();
}

void AlgoControl::update(float sys_pitch, float sys_pitch_rate, float sys_yaw_rate,
                         int16_t enc_left, int16_t enc_right,uint8_t gray_data,
                         int16_t& pwm_left, int16_t& pwm_right)
{
    // --- 1. 速度获取与滤波 ---
    speed_left_filtered = lpf_alpha * (float)enc_left + (1.0f - lpf_alpha) * speed_left_filtered;
    speed_right_filtered = lpf_alpha * (float)enc_right + (1.0f - lpf_alpha) * speed_right_filtered;
    float current_speed = (speed_left_filtered + speed_right_filtered) / 2.0f;

    // --- 2. 设定目标期望 ---
    float target_speed = 0.0f; // 目前强制站立

    // --- 3. 速度外环计算 ---
    float target_pitch = pid_velocity.calc(target_speed, current_speed);

    // =========================================================
    // 【极其重要的裸调补丁】
    // 在刚开始调直立环时，必须将外环屏蔽，并填入你测试出的"物理机械中值"！
    // 假设你测出小车在 -2.3 度时刚好能前后平衡，这里就填 -2.3f。
    // =========================================================
    //target_pitch = -0.94f; // <--- 调参时在 Live Watch 中直接修改这个值找重心！
    target_pitch += -1.32f;
    // --- 4. 直立内环计算 ---
    float base_pwm = pid_upright.calc(target_pitch, sys_pitch, sys_pitch_rate);

    // 循迹算法层测试：算出物理误差
    // ===================================
    float track_error = grayscale_algo.calculate_error(gray_data);

    // --- 5. 转向环计算 ---
    // 目前盲走模式：目标角速度为 0，利用 D 项抵抗左右晃动
    float turn_pwm = pid_turn.calc(0.0f, 0.0f, sys_yaw_rate);

    // 调直立环时，为了防止两个轮子转速不一产生干扰，可以先强制关掉差速
    turn_pwm = 0.0f;

    // --- 6. 动力融合与输出限幅 ---
    float final_pwm_l = base_pwm + turn_pwm;
    float final_pwm_r = base_pwm - turn_pwm;

    if (final_pwm_l > 8200.0f) final_pwm_l = 8200.0f;
    else if (final_pwm_l < -8200.0f) final_pwm_l = -8200.0f;

    if (final_pwm_r > 8200.0f) final_pwm_r = 8200.0f;
    else if (final_pwm_r < -8200.0f) final_pwm_r = -8200.0f;

    pwm_left = (int16_t)final_pwm_l;
    pwm_right = (int16_t)final_pwm_r;
}

extern "C" {
    // 实例化一个全局唯一的小车大脑，低通滤波 Alpha 设为 0.3
    AlgoControl robot_control(0.1f);

    void AlgoInit(void) {
        robot_control.init();
    }

    void AlgoStop(void) {
        robot_control.stop();
    }

    void AlgoUpdate(float sys_pitch, float sys_pitch_rate, float sys_yaw_rate,
                    int16_t enc_left, int16_t enc_right,uint8_t gray_data,
                    int16_t* pwm_left, int16_t* pwm_right)
    {
        robot_control.update(sys_pitch, sys_pitch_rate, sys_yaw_rate,
                             enc_left, enc_right,gray_data,
                             *pwm_left, *pwm_right);
    }
}