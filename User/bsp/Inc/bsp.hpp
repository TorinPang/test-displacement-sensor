#ifndef BSP_HPP
#define BSP_HPP

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 板级外设初始化总入口。
 * @note  必须在 MX_GPIO_Init() / MX_CAN1_Init() 之后、ThreadX 启动之前调用。
 *        顺序为 LED -> CAN：先把灯初始化好，CAN 初始化失败时才有灯可点。
 * @retval true  全部成功
 * @retval false 有模块初始化失败（此时红灯常亮，具体计数见 g_can_stat）
 */
bool bsp_Init(void);

#ifdef __cplusplus
}
#endif

#endif
