//
// Created by 戴境宏 on 2026/4/11.
//
#include "bsp_indicator.h"
#include "main.h"

SystemIndicator::SystemIndicator() {}

void SystemIndicator::init()
{
    // 初始化时确保全部处于关闭状态，防止上电乱叫乱闪
    alarm_off();
}

// ================= 蜂鸣器控制 =================
void SystemIndicator::beep_on()     { HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET); }
void SystemIndicator::beep_off()    { HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET); }
void SystemIndicator::beep_toggle() { HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin); }

// ================= 红灯控制 =================
// 注意：如果你的 LED 是低电平点亮，请将这里的 SET 和 RESET 互换
void SystemIndicator::red_on()      { HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); }
void SystemIndicator::red_off()     { HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET); }
void SystemIndicator::red_toggle()  { HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); }

// ================= 绿灯控制 =================
void SystemIndicator::green_on()      { HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET); }
void SystemIndicator::green_off()     { HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET); }
void SystemIndicator::green_toggle()  { HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin); }

// ================= 联动控制 =================
void SystemIndicator::alarm_on()
{
    beep_on();
    red_on();
    green_on();
}

void SystemIndicator::alarm_off()
{
    beep_off();
    red_off();
    green_off();
}

//---------------------------------------------------------
// C 接口实现
//---------------------------------------------------------
extern "C"
{
    // 实例化全局指示器对象
    SystemIndicator board_indicator;

    void IndicatorInit(void) { board_indicator.init(); }

    void BeepOn(void) { board_indicator.beep_on(); }
    void BeepOff(void) { board_indicator.beep_off(); }
    void BeepToggle(void) { board_indicator.beep_toggle(); }

    void LedRedOn(void) { board_indicator.red_on(); }
    void LedRedOff(void) { board_indicator.red_off(); }
    void LedRedToggle(void) { board_indicator.red_toggle(); }

    void LedGreenOn(void) { board_indicator.green_on(); }
    void LedGreenOff(void) { board_indicator.green_off(); }
    void LedGreenToggle(void) { board_indicator.green_toggle(); }

    void AlarmOn(void) { board_indicator.alarm_on(); }
    void AlarmOff(void) { board_indicator.alarm_off(); }
}