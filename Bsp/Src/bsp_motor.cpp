//
// Created by 戴境宏 on 2026/3/29.
#include "bsp_motor.h"
#include "main.h"
#include <algorithm>


TB6612::TB6612(int16_t max_pwm_value) : max_pwm(max_pwm_value) {}  // 实例化，传入8399

void TB6612::motor_init()
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); //开启pwm通道
}

void TB6612::motor_set_pwm(int16_t left_pwm, int16_t right_pwm)
{
    left_pwm = std::clamp(left_pwm, (int16_t)-max_pwm, max_pwm);
    right_pwm = std::clamp(right_pwm, (int16_t)-max_pwm, max_pwm);  //输出保护

    // ================= 左轮控制 (Left Motor - A 通道) =================
    if (left_pwm > 0)
    {
        // 正转：IN1 = H, IN2 = L
        HAL_GPIO_WritePin(AIN_1_GPIO_Port, AIN_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN_2_GPIO_Port, AIN_2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, left_pwm);
    }
    else if (left_pwm < 0)
    {
        // 反转：IN1 = L, IN2 = H
        HAL_GPIO_WritePin(AIN_1_GPIO_Port, AIN_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN_2_GPIO_Port, AIN_2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, -left_pwm); // 定时器比较寄存器只收正数
    }
    else
    {
        // 短路制动：IN1 = H, IN2 = H, PWM = 0 (根据原厂数据手册要求)
        HAL_GPIO_WritePin(AIN_1_GPIO_Port, AIN_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN_2_GPIO_Port, AIN_2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    }

    // ================= 右轮控制 (Right Motor - B 通道) =================
    if (right_pwm > 0)
    {
        // 正转：IN1 = H, IN2 = L
        HAL_GPIO_WritePin(BIN_1_GPIO_Port, BIN_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN_2_GPIO_Port, BIN_2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, right_pwm);
    }
    else if (right_pwm < 0)
    {
        // 反转：IN1 = L, IN2 = H
        HAL_GPIO_WritePin(BIN_1_GPIO_Port, BIN_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN_2_GPIO_Port, BIN_2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -right_pwm);
    }
    else
    {
        // 短路制动：IN1 = H, IN2 = H, PWM = 0
        HAL_GPIO_WritePin(BIN_1_GPIO_Port, BIN_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN_2_GPIO_Port, BIN_2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    }
}


extern "C"
{
    // 全局实例化 TB6612 驱动对象，传入最大 PWM 计数值 8399
    TB6612 chassis_motors(8399);

    void MotorInit(void)
    {
        chassis_motors.motor_init();
    }

    void MotorSetPWM(int16_t left_pwm, int16_t right_pwm)
    {
        chassis_motors.motor_set_pwm(left_pwm, right_pwm);
    }
}