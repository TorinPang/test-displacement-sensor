#ifndef THREAD_HPP
#define THREAD_HPP

#include "tx_api.h"

/* app_threadx.c 是 C 文件，这里必须声明成 C 链接，
 * 否则 sensor_thread.cpp 里的定义会被 name mangle 成 _Z15TestThread_Initv，
 * C 那边调用时就变成 undefined reference。 */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建并启动应用线程（由 App_ThreadX_Init 在调度器启动前调用）。
 * @note  只是 tx_thread_create，线程体在调度器起来后才开始跑。
 */
void SensorThread_Init(void);

#ifdef __cplusplus
}
#endif

#endif
