 //
// Created by 戴境宏 on 2026/4/15.
//
#include "algo_control.h"

// 构造函数：初始化三大环参数与滤波器状态
AlgoControl::AlgoControl(float alpha)
    : pid_upright(-1680.0f, 0.0f, -30.0f, 8200.0f, 0.0f),   // 直立环：纯 PD，限幅 8200
      pid_velocity(0.35f, 0.003f, 0.0f, 20.0f, 1800.0f), // 速度环：纯 PI，角度限幅 15度
      pid_turn(-60.0f, 0.0f, -30.0f, 3000.0f, 0.0f),      // 转向环：暂不启用，限幅 3000
      lpf_alpha(alpha),
      speed_left_filtered(0.0f),
      speed_right_filtered(0.0f) {}

void AlgoControl::init() {
    pid_upright.reset();
    pid_velocity.reset();
    pid_turn.reset();
    speed_left_filtered = 0.0f;
    speed_right_filtered = 0.0f;
    target_speed = 0.0f; // 初始静止
    target_turn = 0.0f;  // 初始不转向
    is_tracking_mode = false;
}

void AlgoControl::stop() {
    pid_upright.reset();
    pid_velocity.reset();
    pid_turn.reset();
}

void AlgoControl::update(float sys_pitch, float sys_pitch_rate, float sys_yaw,float sys_yaw_rate,
                         int16_t enc_left, int16_t enc_right,uint8_t gray_data,
                         int16_t& pwm_left, int16_t& pwm_right)
{
    // --- 1. 速度获取与滤波 ---
    speed_left_filtered = lpf_alpha * (float)enc_left + (1.0f - lpf_alpha) * speed_left_filtered;
    speed_right_filtered = lpf_alpha * (float)enc_right + (1.0f - lpf_alpha) * speed_right_filtered;
    float current_speed = (speed_left_filtered + speed_right_filtered) / 2.0f;

    // --- 2. 设定目标期望 ---
    //float target_speed = 0.0f; // 目前强制站立

    // --- 3. 速度外环计算 ---
    float target_pitch = pid_velocity.calc(target_speed, current_speed);


    target_pitch += -0.85f;
    // --- 4. 直立内环计算 ---
    float base_pwm = pid_upright.calc(target_pitch, sys_pitch, sys_pitch_rate);
    //base_pwm = 0.0f;//调试角度环用
    // 循迹算法层测试：算出物理误差
    // ===================================
    float track_error = grayscale_algo.calculate_error(gray_data);

    // --- 5. 转向环计算 ---
    // 目前盲走模式：目标角速度为 0，利用 D 项抵抗左右晃动
    //float turn_pwm = pid_turn.calc(target_turn, sys_yaw, sys_yaw_rate);
    float turn_pwm = 0.0f;
    if (is_tracking_mode) {
        // 【人格 A：循迹机器】
        // 目标是让黑线居中(误差0)，当前状态就是传感器的真实物理偏差
        turn_pwm = pid_turn.calc(0.0f, track_error, sys_yaw_rate);
    } else {
        // 【人格 B：盲走/锁向机器】
        // 目标是状态机下发的角度(通常是0)，当前状态是陀螺仪的积分角度
        turn_pwm = pid_turn.calc(target_turn, sys_yaw, sys_yaw_rate);
    }
    // 调直立环时，为了防止两个轮子转速不一产生干扰，可以先强制关掉差速
    //turn_pwm = 0.0f;

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

    void AlgoUpdate(float sys_pitch, float sys_pitch_rate,float sys_yaw, float sys_yaw_rate,
                    int16_t enc_left, int16_t enc_right,uint8_t gray_data,
                    int16_t* pwm_left, int16_t* pwm_right)
    {
        robot_control.update(sys_pitch, sys_pitch_rate,sys_yaw, sys_yaw_rate,
                             enc_left, enc_right,gray_data,
                             *pwm_left, *pwm_right);
    }
}