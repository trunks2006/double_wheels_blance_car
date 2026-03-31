//
// Created by 戴境宏 on 2026/3/31.
//

#include "bsp_grayscale.h"
#include "main.h"

// 构造函数
Grayscale::Grayscale() {}

void Grayscale::grayscale_init()
{

    set_mux_channel(0);
}


void Grayscale::set_mux_channel(uint8_t channel)
{


    HAL_GPIO_WritePin(GRAY_ADD_0_GPIO_Port, GRAY_ADD_0_Pin,
                     ((channel >> 0) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GRAY_ADD_1_GPIO_Port, GRAY_ADD_1_Pin,
                     ((channel >> 1) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GRAY_ADD_2_GPIO_Port, GRAY_ADD_2_Pin,
                     ((channel >> 2) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void Grayscale::delay_us(uint32_t us)
{

    uint32_t delay_loops = us * 42;
    for(volatile uint32_t i = 0; i < delay_loops; i++)
    {
        __NOP();
    }
}

// ================= 业务层扫描逻辑 =================
uint8_t Grayscale::grayscale_read_all()
{
    uint8_t sensor_data = 0; // 用一个 8 位整数打包 8 个通道的数据，比官方的数组更省内存且利于位运算

    for (uint8_t i = 0; i < 8; i++) // 对应官方的 GRAYSCALE_SENSOR_CHANNELS
    {
        // 1. 发送地址指令，选通第 i 个探头
        set_mux_channel(i);

        // 2. 严格遵循官方要求的硬件建立时间
        delay_us(50); // 参考官方手册：delay_us(50);

        // 3. 读取数据引脚状态
        if (HAL_GPIO_ReadPin(GRAY_DATA_IN_GPIO_Port, GRAY_DATA_IN_Pin) == GPIO_PIN_SET)
        {
            // 如果读到高电平 (官方手册中灯亮=1)，对应 bit 位置 1
            sensor_data |= (1 << i);
        }
    }

    return sensor_data;
}

//---------------------------------------------------------
// C 接口实现
//---------------------------------------------------------
extern "C"
{
    // 实例化全局灰度扫描对象
    Grayscale board_grayscale;

    void GrayscaleInit(void)
    {
        board_grayscale.grayscale_init();
    }

    uint8_t GrayscaleReadAll(void)
    {
        return board_grayscale.grayscale_read_all();
    }
}
