//
// Created by 戴境宏 on 2026/4/12.
//
//
// Created by 戴境宏
//
#include "filter.h"
#include <math.h>

#define PI 3.1415926535f
#define RAD_TO_DEG (180.0f / PI)

// 陀螺仪灵敏度：2000dps量程下，约 0.07 mdps/LSB
#define GYRO_SCALE (0.07f)

ComplementaryFilter::ComplementaryFilter(float dt_sec, float filter_alpha)
    : dt(dt_sec), alpha(filter_alpha), current_pitch(0.0f) {}

void ComplementaryFilter::reset() { current_pitch = 0.0f; }

void ComplementaryFilter::update(int16_t* raw_acc, int16_t* raw_gyro, float& pitch, float& pitch_rate, float& yaw_rate)
{
    // ================= 1. 数据提取与方向纠正 =================
    // 刚才我们测出，车头前倾时 raw_acc[0] 是负数 (-2530)。
    // 为了符合控制理论习惯 (前倾角度为正)，我们在这里加上负号强行翻转 X 轴！
    float acc_x = -(float)raw_acc[0];
    float acc_z = (float)raw_acc[2];

    // Y 轴角速度 (测量车头前后仰俯的转动)
    // 根据右手定则，如果 X 轴反了，我们也要检查 Y 轴转动的符号。
    // 如果之后上车发现车倒了，只需要在这里给 raw_gyro[1] 加个负号即可。
    pitch_rate = (float)raw_gyro[1] * GYRO_SCALE;

    // Z 轴角速度 (测量左右转向，用于盲走和循迹)
    yaw_rate = (float)raw_gyro[2] * GYRO_SCALE;

    // ================= 2. 加速度计求绝对基准角 =================
    // atan2f 依靠比例求角度，完美无视量程差异(不管1g是16384还是8192都不影响)
    float acc_angle = atan2f(acc_x, acc_z) * RAD_TO_DEG;

    // ================= 3. 核心：一阶互补滤波 =================
    // alpha(0.98)：98% 相信陀螺仪的高频动态积分
    // (1-alpha)(0.02)：2% 依靠加速度计纠正陀螺仪的温漂
    current_pitch = alpha * (current_pitch + pitch_rate * dt) + (1.0f - alpha) * acc_angle;

    // 4. 数据输出
    pitch = current_pitch;
}


extern "C" {
    // 实例化滤波器：假设控制周期为 5ms (0.005s)，滤波系数设为 0.98
    ComplementaryFilter imu_filter(0.005f, 0.98f);

    void FilterInit(void) {
        imu_filter.reset();
    }

    void FilterUpdate(int16_t* acc, int16_t* gyro, float* pitch, float* pitch_rate, float* yaw_rate) {
        imu_filter.update(acc, gyro, *pitch, *pitch_rate, *yaw_rate);
    }
}