//
// Created by 戴境宏 on 2026/4/11.
//

#ifndef BALANCE_CAR_BSP_INDICATOR_H
#define BALANCE_CAR_BSP_INDICATOR_H
#pragma once

#include <stdint.h>

#ifdef __cplusplus
class SystemIndicator
{
public:
    SystemIndicator();

    // 初始化所有指示器 (默认全部关闭)
    void init();

    // 蜂鸣器控制 (BEEP)
    void beep_on();
    void beep_off();
    void beep_toggle();

    // 红灯控制 (LED_RED)
    void red_on();
    void red_off();
    void red_toggle();

    // 绿灯控制 (LED_GREEN)
    void green_on();
    void green_off();
    void green_toggle();

    // 联动控制：一键声光报警 (方便电赛题目要求中的"声光提示")
    void alarm_on();
    void alarm_off();
};
#endif

//---------------------------------------------------------
// 提供给 C 语言环境的接口
//---------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

    void IndicatorInit(void);

    void BeepOn(void);
    void BeepOff(void);
    void BeepToggle(void);

    void LedRedOn(void);
    void LedRedOff(void);
    void LedRedToggle(void);

    void LedGreenOn(void);
    void LedGreenOff(void);
    void LedGreenToggle(void);

    void AlarmOn(void);
    void AlarmOff(void);

#ifdef __cplusplus
}
#endif
#endif //BALANCE_CAR_BSP_INDICATOR_H