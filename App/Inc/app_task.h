//
// Created by 戴境宏 on 2026/4/28.
//

#ifndef BALANCE_CAR_APP_TASK_H
#define BALANCE_CAR_APP_TASK_H
#pragma once
#include <stdint.h>
#ifdef __cplusplus
#include "algo_control.h"
#include "bsp_indicator.h"
#include "odometer.h"
// 拨码开关对应的四大模式
enum ModeSelect {
    MODE_STAND      = 0, // 00: 任务1 - 纯直立
    MODE_A_TO_B     = 1, // 01: 任务2 - 直线盲走 (暂未实现)
    MODE_OVAL_TRACK = 2, // 10: 任务3 - 环形跑道 (暂未实现)
    MODE_FIGURE_8   = 3  // 11: 任务4 - 8字交叉 (暂未实现)
};

class AppTask {
public:
    AppTask(AlgoControl* algo, SystemIndicator* indicator);

    void init();
    // 每次 5ms 定时器中断
    void update_5ms(int16_t enc_l, int16_t enc_r, uint8_t gray_data);
private:
    AlgoControl* robot;
    SystemIndicator* led_beep;
    Odometer odom;
    ModeSelect current_mode;
    bool was_on_line;
    float cross_turn_dir;
    uint32_t state_timer;    // 用于 3 秒计时
    uint32_t beep_timer;
    // 内部方法
    ModeSelect read_dip_switches();
    void execute_mode_stand();
    void execute_mode_1(uint8_t gray_data);
    void execute_mode_oval_track(uint8_t gray_data);
    void execute_mode_figure_8(uint8_t gray_data);
};
#endif
// ================= C 语言调用接口 =================
#ifdef __cplusplus
extern "C" {
#endif
    void AppTaskInit(void);
    void AppTaskUpdate(int16_t enc_l, int16_t enc_r,uint8_t gray_data);
#ifdef __cplusplus
}
#endif
#endif //BALANCE_CAR_APP_TASK_H