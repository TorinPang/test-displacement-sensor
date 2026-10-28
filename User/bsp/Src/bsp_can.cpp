#include "bsp_can.hpp"
#include "displacement_sensor.hpp"

/* 本模块的统计量，供 Debug（Live Watch）观察 */
volatile CAN_Stat_t g_can_stat = {0};

bool CAN_Init(void)
{
    CAN_FilterTypeDef can_filter_st = {0};             ///< 定义过滤器结构体（置零，避免未初始化字段）

    /* 全通滤波：ID 与 MASK 全 0 => 总线上的帧一律收进 FIFO0。
     * 调试期这么配最省事（能确认总线上到底有没有帧，连自己发的也能看到）。
     * 注意：32 位掩码模式下标准帧 ID 位于 STDID[31:21]，
     *      将来若要精确只收某个 ID，必须写成 FilterIdHigh = (id << 5) 这类移位形式，
     *      直接写 0x0001 是收不到的。*/
    can_filter_st.FilterActivation = ENABLE;           ///< ENABLE使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;  ///< 设置过滤器模式--标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; ///< 过滤器的位宽 32 位
    can_filter_st.FilterIdHigh = 0x0000;               ///< ID高位
    can_filter_st.FilterIdLow = 0x0000;                ///< ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000;           ///< 过滤器掩码高位
    can_filter_st.FilterMaskIdLow = 0x0000;            ///< 过滤器掩码低位
    can_filter_st.FilterBank = 0;                      ///< 过滤器组号（CAN1 使用 0~13）
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;  ///< 必须与 NVIC 里打开的 RX0 中断对应

    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK)
    {
        return false;
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return false;
    }
    /* 接收中断 + 错误中断一起打开。
     * 注意：NVIC 里使能 CAN1_SCE 只是开了「中断通道」，
     *      外设侧 IER 的错误使能位要在这里一起激活，否则 SCE 永远不会进。*/
    if (HAL_CAN_ActivateNotification(&hcan1,
                                     CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR_WARNING |
                                         CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE) != HAL_OK)
    {
        return false;
    }

    return true;
}

bool CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Id, const uint8_t *msg, uint16_t len)
{
    CAN_TxHeaderTypeDef can_tx_header_st = {0}; ///< 定义发送报文头结构体（置零）
    uint32_t tx_mailbox = 0U;                   ///< 定义发送邮箱变量

    // 参数保护：句柄、数据、长度（标准帧数据长度 0~8）、11 位 ID 都要合法
    if ((hcan == NULL) || (msg == NULL) || (len > 8U) || (Id > 0x7FFU))
    {
        return false;
    }
    // 邮箱满了就放弃本帧，避免覆盖尚未发出的报文（丢弃次数计入统计，便于发现「静默不发」）
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        g_can_stat.tx_drop_mailbox++;
        return false;
    }

    can_tx_header_st.StdId = Id;          ///< 设置标准标识符
    can_tx_header_st.ExtId = 0x00;        ///< 扩展标识符设置为0
    can_tx_header_st.RTR = CAN_RTR_DATA;  ///< 设置为数据帧
    can_tx_header_st.IDE = CAN_ID_STD;    ///< 设置为标准帧
    can_tx_header_st.DLC = (uint8_t)len;  ///< 设置数据长度码

    if (HAL_CAN_AddTxMessage(hcan, &can_tx_header_st, msg, &tx_mailbox) != HAL_OK)
    {
        g_can_stat.tx_fail++;
        return false;
    }

    g_can_stat.tx_ok++;
    return true;
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t rx_data[8] = {0};
    uint16_t len;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }
    // 本工程只处理数据帧里的标准帧
    if ((rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA))
    {
        return;
    }
    len = (uint16_t)rx_header.DLC;
    if (len > 8U)
    {
        len = 8U; // 理论上 DLC 不会超过 8，防御性截断，保证上层只用 data[0..len-1]
    }

    g_can_stat.rx_frames++;

    DisplacementSensor *s = DisplacementSensor::Instance();
    if (hcan != s->cfg.hcan || rx_header.StdId != s->cfg.node_id)
        return;
    s->OnFrame(rx_data, len);
}

/**
 * @brief CAN 错误回调（HAL 弱函数覆盖）。
 * @note  总线错误（ACK 错、位错、CRC 错、bus-off 等）都会走到这里。
 *        现场只记快照供 Live Watch 观察，不做恢复动作（AutoBusOff 已使能，硬件自动恢复）。
 */
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if ((hcan == NULL) || (hcan->Instance != CAN1))
    {
        return;
    }

    g_can_stat.err_flags |= HAL_CAN_GetError(hcan);
    g_can_stat.esr = hcan->Instance->ESR; // LEC: 最近一次错误类型；TEC/REC: 收发错误计数
    g_can_stat.err_cnt++;
}
