#include "bsp.hpp"

#include "bsp_can.hpp"
#include "bsp_led.hpp"

bool bsp_Init(void)
{
    /* 先初始化 LED（三灯全灭）：后面任何模块初始化失败，才有灯可以报错 */
    LED_Init();

    if (!CAN_Init())
    {
        LED_Set(LED_RED, 1U); // 红灯常亮 = CAN 初始化失败（配滤波器/启动/开中断任一步失败）
        return false;
    }

    return true;
}
