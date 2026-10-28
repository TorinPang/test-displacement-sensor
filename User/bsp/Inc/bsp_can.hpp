#ifndef BSP_CAN_HPP
#define BSP_CAN_HPP

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CAN 运行统计与诊断信息（只用于观察，不参与控制逻辑）。
 * @note  Debug 时在 Live Watch 里整体 watch 这个结构体即可：
 *        tx_ok 不再增长  -> 发送侧有问题（邮箱满 / 没进总线）
 *        rx_frames 不涨  -> 滤波器、ID、波特率或硬件链路有问题
 *        err_cnt 在涨    -> 看 esr 里的 LEC/TEC/REC 定位是 ACK 错、位错还是 bus-off
 */
typedef struct
{
    volatile uint32_t tx_ok;            ///< 成功塞进邮箱的帧数
    volatile uint32_t tx_drop_mailbox;  ///< 邮箱满而直接丢弃的帧数（无 ACK 时会持续增长）
    volatile uint32_t tx_fail;          ///< HAL_CAN_AddTxMessage 返回失败的次数
    volatile uint32_t rx_frames;        ///< 收到的标准数据帧数
    volatile uint32_t err_cnt;          ///< 进入 HAL_CAN_ErrorCallback 的次数
    volatile uint32_t err_flags;        ///< HAL_CAN_GetError() 的位或（HAL_CAN_ERROR_xxx）
    volatile uint32_t esr;              ///< CAN1->ESR 快照：bit[2:0]=LEC, bit[23:16]=REC, bit[31:24]=TEC
} CAN_Stat_t;

/** CAN 全局统计（定义在 bsp_can.cpp） */
extern volatile CAN_Stat_t g_can_stat;

/**
 * @brief 初始化 CAN：配滤波器 -> 启动外设 -> 打开接收/错误中断。
 * @note  必须在 MX_CAN1_Init() 之后调用。
 * @retval true  三步全部成功
 * @retval false 任一步失败（句柄未就绪 / 参数非法）
 */
bool CAN_Init(void);

/**
 * @brief 发送一帧标准数据帧（阻塞式塞邮箱，不等待发送完成）。
 * @param hcan 指向 CAN 句柄的指针
 * @param Id   标准帧 ID（0x000~0x7FF）
 * @param msg  待发送数据
 * @param len  数据长度（0~8）
 * @retval true  已成功装入发送邮箱
 * @retval false 参数非法 / 邮箱已满 / HAL 返回失败（原因见 g_can_stat）
 * @note  该函数用于发送CAN数据，目前只支持标准帧
 */
bool CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Id, const uint8_t *msg, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
