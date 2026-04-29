//
// Created by 戴境宏 on 2026/4/28.
//
#include "app_task.h"
#include "main.h"   // 读取拨码开关 GPIO
#include "filter.h"
// ================= 常量配置 =================
// 巡航速度 (脉冲/5ms)，数值可以根据底盘响应微调，10 左右一般比较稳健
#define CRUISE_SPEED  7.5f
AppTask::AppTask(AlgoControl* algo, SystemIndicator* indicator)
    : robot(algo), led_beep(indicator), current_mode(MODE_STAND),
      was_on_line(false), cross_turn_dir(1.0f), state_timer(0), beep_timer(9999),
      line_lost_count(255) {}

void AppTask::init() {
    current_mode = read_dip_switches();
    state_timer = 0;
    line_lost_count = 255;
    was_on_line = false;// 初始化时强制停车
    if (current_mode == MODE_FIGURE_8) {
        // 【任务 4 专属】：欺骗大脑！
        // 初始设为 true，开机在白区 (0x00) 的第一帧就会强行触发“脱线边缘检测”。
        // 配合初始的 1.0f，翻转后变成 -1.0f，完美实现“开机原地右转 300ms 后冲刺”！
        was_on_line = true;
    }
    cross_turn_dir = 1.0f;
    robot->target_speed = 0.0f;
    robot->target_turn = 0.0f;
    led_beep->alarm_off();
}

// 读取拨码开关 PA1(SWB1) 和 PC3(SWB2)
ModeSelect AppTask::read_dip_switches() {
    uint8_t bit0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET ? 1 : 0;
    uint8_t bit1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_SET ? 1 : 0;
    uint8_t mode_val = (bit1 << 1) | bit0;
    return (ModeSelect)mode_val;
}

void AppTask::update_5ms(int16_t enc_l, int16_t enc_r,uint8_t gray_data) {
    // 每次中断都读取拨码开关，方便你在调试时随时切模式 (实战时可能需要锁死)
    current_mode = read_dip_switches();
    odom.update(enc_l, enc_r);
    // 模式分发
    switch (current_mode) {
        case MODE_STAND:
            execute_mode_stand();
            break;

        case MODE_A_TO_B:
            execute_mode_1(gray_data);
            break;
        case MODE_OVAL_TRACK:
            execute_mode_oval_track(gray_data);
            break;
        case MODE_FIGURE_8:
            execute_mode_figure_8(gray_data);
            break;
    }
}

// ================= 任务 1：纯直立 =================
void AppTask:: execute_mode_stand() {
    // 1. 下达静止指令，速度环的 I 项会把它死死钉在原地
    robot->target_speed = 0.0f;
    robot->target_turn = 0.0f;

    // 2. 计时器累加 (限制最大值防止溢出)
    if (state_timer < 1000) {
        state_timer++;
    }

    // 3. 仪式感展示：精准到达 3 秒 (600 * 5ms = 3000ms) 时，触发声光提示！
    if (state_timer == 600) {
        led_beep->green_on(); // 亮绿灯
        led_beep->beep_on();  // 滴——
    }
    // 响 200ms (40次中断) 后自动关闭，展示结束，继续保持静止
    else if (state_timer == 640) {
        led_beep->green_off();
        led_beep->beep_off();
    }
}
// ================= 任务 2：极其纯粹的视觉触发控制 =================
void AppTask::execute_mode_1(uint8_t gray_data) {

    // 终极判断：0x18 即二进制的 00011000 (中间两个探头踩到黑线)
    if (gray_data != 0x00|| state_timer > 0) {

        // 【满足条件】：立刻急刹车，死死站住
        robot->target_speed = 0.0f;
        robot->target_turn = 0.0f;

        // 【单次“滴”声逻辑】：刚踩线的瞬间响，0.2秒后自动关
        if (state_timer == 0) {
            led_beep->alarm_on();
        }
        if (state_timer < 1000) {
            state_timer++;
        }
        if (state_timer == 40) {
            led_beep->alarm_off();
        }

    } else {
        // 【不满足条件】：还没踩到线，锁定航向 0 度，踩死油门盲飙！
        robot->target_speed = CRUISE_SPEED;
        robot->target_turn = 0.0f;

        led_beep->alarm_off();
        state_timer = 0;       // 只要在跑，计时器就被死死压制在 0
    }
}
// ================= 任务 3：圆弧循迹与盲走 (终极防抖 + 终局刹车版) =================
void AppTask::execute_mode_oval_track(uint8_t gray_data) {

    // ==========================================
    // 1. 视觉防抖滤网 (过滤弯道颠簸引发的假脱线)
    // ==========================================
    if (gray_data == 0x00) {
        if (line_lost_count < 255) line_lost_count++; // 没看到线，计数累加
    } else {
        line_lost_count = 0; // 只要看到任何一帧黑线，瞬间清零！
    }

    // 判断真理：连续瞎了 10 帧 (50ms)，大脑才承认是真的脱线了！
    bool see_line = (line_lost_count < 10);

    // ==========================================
    // 2. 边缘检测：升级为“跳变计数器”
    // ==========================================
    if (see_line != was_on_line) {
        beep_timer = 0;

        // 【核心灵魂】：每次真正的踩线或脱线，计数器 +1
        state_timer++;

        if (!see_line) {
            FilterResetYaw(); // 刚出弯道脱线的瞬间，清零陀螺仪准备盲走
        }
        was_on_line = see_line;
    }

    // ==========================================
    // 3. 终点判定！(经历了 4 次跳变 = 跑完一圈回到了 A 点)
    // ==========================================
    if (state_timer >= 4) {
        robot->target_speed = 0.0f;       // 死锁刹车
        robot->target_turn = 0.0f;        // 方向回正
        robot->is_tracking_mode = false;  // 关闭循迹

        // 终点声光长鸣，宣告任务完美结束！
        led_beep->alarm_on();

        // 【极其关键】：直接 return，拒不执行后面的代码，让小车永远困在刹车状态！
        return;
    }

    // ==========================================
    // 4. 肌肉层执行 (保留了你调好的 -7.5f)
    // ==========================================
    robot->target_speed = CRUISE_SPEED;

    if (see_line) {
        // 线上模式：全速循迹
        robot->is_tracking_mode = true;
    } else {
        // 盲走模式：根据跳变次数区分不同赛段
        robot->is_tracking_mode = false;

        if (state_timer == 0) {
            robot->target_turn = 0.0f;       // 第 0 次跳变 (A->B)：笔直起跑
        } else if (state_timer == 2) {
            robot->target_turn = -8.0f;      // 第 2 次跳变 (B->C)：向右微调咬线
        }
    }

    // ==========================================
    // 5. 运行中的短促蜂鸣器 (标志点响应)
    // ==========================================
    if (beep_timer < 1000) {
        if (beep_timer == 0)  led_beep->alarm_on();
        if (beep_timer == 20) led_beep->alarm_off();
        beep_timer++;
    }
}
// ================= 任务 4：8字交叉 =================
void AppTask::execute_mode_figure_8(uint8_t gray_data) {

    // ==========================================
    // 1. 视觉防抖滤网
    // ==========================================
    if (gray_data == 0x00) {
        if (line_lost_count < 255) line_lost_count++;
    } else {
        line_lost_count = 0;
    }

    bool see_line = (line_lost_count < 10);

    // ==========================================
    // 2. 边缘检测与跳变计数
    // ==========================================
    if (see_line != was_on_line) {
        beep_timer = 0;

        // 【核心】：每次跳变（脱线或踩线），状态计数器加 1
        state_timer++;

        if (!see_line) {
            FilterResetYaw();
            // 【灵魂逻辑】：每次脱线，把转向系数乘 -1，实现“左转-右转”完美交替！
            cross_turn_dir = -cross_turn_dir;
        }

        was_on_line = see_line;
    }

    // ==========================================
    // 3. 【新增】：终点判定！(经历 5 次跳变 = 跑完 8 字回到 A 点)
    // ==========================================
    if (state_timer >= 5) {
        robot->target_speed = 0.0f;       // 死锁刹车
        robot->target_turn = 0.0f;        // 方向回正
        robot->is_tracking_mode = false;  // 关闭循迹

        // 终点长鸣，宣告完美跑完 8 字！
        led_beep->alarm_on();

        return; // 直接 return 锁死状态，绝不执行后续动力逻辑！
    }

    // ==========================================
    // 4. 基于“存在状态”和“时间”的双维动作执行
    // ==========================================
    if (see_line) {
        // 【线上模式】：全速循迹，不废话
        robot->target_speed = 7.5f;
        robot->is_tracking_mode = true;
    }
    else {
        // 【盲走模式】：时序分层控制 (白嫖 beep_timer！)
        robot->is_tracking_mode = false;

        // 目标偏航角：你调好的 23.5 度，并附带交替方向
        robot->target_turn = 24.5f * cross_turn_dir;

        // 60 次中断 * 5ms = 300ms。这段时间足够底层 PID 把车头拧到位了。
        if (beep_timer < 60) {
            robot->target_speed = 0.0f; // 【第一阶段：刹车，原地扭车头】
        } else {
            robot->target_speed = 9.0f; // 【第二阶段：发车冲刺】你调好的 9.0f 方便入弯
        }
    }

    // ==========================================
    // 5. 非阻塞蜂鸣器管理 (它在发声的同时，也充当了动作的秒表)
    // ==========================================
    if (beep_timer < 1000) {
        if (beep_timer == 0)  led_beep->alarm_on();
        if (beep_timer == 20) led_beep->alarm_off();
        beep_timer++;
    }
}

// ================= C 接口封装 =================
// 引用底层的全局对象
extern AlgoControl robot_control;
extern SystemIndicator board_indicator;

// 实例化 AppTask
AppTask app_task(&robot_control, &board_indicator);

extern "C" {
    void AppTaskInit(void) {
        app_task.init();
    }

    void AppTaskUpdate(int16_t enc_l, int16_t enc_r,uint8_t gray_data) {
        app_task.update_5ms(enc_l, enc_r,gray_data);
    }
}