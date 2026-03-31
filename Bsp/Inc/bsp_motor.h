//
// Created by 戴境宏 on 2026/3/29.
//

#ifndef BALANCE_CAR_BSP_MOTOR_H
#define BALANCE_CAR_BSP_MOTOR_H

#pragma once

#include <stdint.h>
#include "tim.h"
#ifdef __cplusplus
class TB6612
{
public:
    // 构造函数，传入定时器的最大 PWM 计数值 (如 8399) 用于抗饱和限幅
    TB6612(int16_t max_pwm_value);


    void motor_init();


    void motor_set_pwm(int16_t left_pwm, int16_t right_pwm);

private:
    // 保存最大 PWM 限幅值，用于物理安全保护
    int16_t max_pwm;
};
#endif


#ifdef __cplusplus
extern "C"
{
#endif


    void MotorInit(void);


    void MotorSetPWM(int16_t left_pwm, int16_t right_pwm);

#ifdef __cplusplus
}
#endif
#endif //BALANCE_CAR_BSP_MOTOR_H