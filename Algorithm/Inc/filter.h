//
// Created by 戴境宏 on 2026/4/12.
//

#ifndef BALANCE_CAR_FILTER_H
#define BALANCE_CAR_FILTER_H
#pragma once
#include <stdint.h>

#ifdef __cplusplus
class ComplementaryFilter
{
public:
    ComplementaryFilter(float dt_sec, float filter_alpha);
    void reset();

    // 传入原始数据，输出平滑后的俯仰角、Y轴角速度(用于直立D项)、Z轴角速度(用于盲走/转向)
    void update(int16_t* raw_acc, int16_t* raw_gyro, float& pitch, float& pitch_rate, float& yaw_rate);

private:
    float dt;
    float alpha;
    float current_pitch;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif
    void FilterInit(void);
    void FilterUpdate(int16_t* acc, int16_t* gyro, float* pitch, float* pitch_rate, float* yaw_rate);
#ifdef __cplusplus
}
#endif
#endif //BALANCE_CAR_FILTER_H