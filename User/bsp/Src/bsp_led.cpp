#include "bsp_led.hpp"
#include "main.h"

namespace
{
    /* 颜色 -> 引脚 的查表（下标即 LED_Color_t 的值）。
     * GPIOH 是 ((GPIO_TypeDef *) GPIOH_BASE) 形式的地址常量，故这里是静态初始化。*/
    GPIO_TypeDef *const kLedPort[3] = {LED_RED_GPIO_Port, LED_GREEN_GPIO_Port, LED_BLUE_GPIO_Port};
    const uint16_t kLedPin[3] = {LED_RED_Pin, LED_GREEN_Pin, LED_BLUE_Pin};

    /* 当前电平缓存：电平没变就不重复写 GPIO —— 少一次总线访问，也让"谁在驱动灯"更好追。*/
    uint8_t LedState[3] = {0U, 0U, 0U};
} // namespace

void LED_Init(void)
{
    /* 先摆全灭再交给别人驱动：避免上电瞬间残留电平让三灯齐亮。*/
    for (uint32_t i = 0U; i < 3U; ++i)
    {
        HAL_GPIO_WritePin(kLedPort[i], kLedPin[i], GPIO_PIN_RESET);
        LedState[i] = 0U;
    }
}

void LED_Set(LED_Color_t color, uint8_t on)
{
    const uint32_t idx = static_cast<uint32_t>(color);
    if (idx >= 3U)
    {
        return; // 越界保护：宁可不动灯，也不要写错引脚
    }

    const uint8_t level = (on != 0U) ? 1U : 0U;
    if (LedState[idx] == level)
    {
        return;
    }

    LedState[idx] = level;
    HAL_GPIO_WritePin(kLedPort[idx], kLedPin[idx], level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LED_SetAll(uint8_t red, uint8_t green, uint8_t blue)
{
    LED_Set(LED_RED, red);
    LED_Set(LED_GREEN, green);
    LED_Set(LED_BLUE, blue);
}

void LED_Off(void)
{
    LED_SetAll(0U, 0U, 0U);
}
