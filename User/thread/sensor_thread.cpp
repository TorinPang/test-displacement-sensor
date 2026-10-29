#include "thread.hpp"

#include "bsp_can.hpp"
#include "bsp_led.hpp"
#include "displacement_sensor.hpp"

/* ============================== 本线程测什么 ==============================
 * 目标：验证 0x03（设波特率）这条指令 —— 把传感器从 500K 切到 kTargetBaud，
 *       主机跟着改位时序，确认新速率上真能通信，最后切回 500K。
 *
 * 为什么这条测试必须"主机自己也改"：
 *   0x03 是把波特率写进**传感器**的（掉电保持）。传感器一换，主机还按 500K 收发就
 *   全变成错误帧，表现和"传感器坏了"一模一样。所以流程必须是：
 *     发 0x03 -> 传感器在新速率上说话 -> 主机改位时序 -> 试探 -> （可选）切回 500K。
 *
 * 丢包的兜底：换完之后如果目标波特率不通，**不要原地判死**。传感器可能根本没收到 0x03
 *   （还在 500K），也可能收到了但我们算错了分频。这里直接**把 5 个波特率全扫一遍**，
 *   哪个速率能通就说明它在哪 —— 最坏情况也能自己找回来，不会像换 ID 那样永久失联。
 *
 * 位时序怎么来的（跟 Core/Src/can.c 里 500K 的配置对齐，只改分频）：
 *   can.c: Prescaler=6, BS1=10TQ, BS2=3TQ, SJW=2TQ -> 每 bit = 1+10+3 = 14 TQ，
 *   采样点 11/14 = 78.6%。CAN 时钟 = APB1 = 42MHz（42M/6/14 = 500K，正好对上），
 *   所以任意速率的分频 = 42MHz / (14 × 波特率)，五个值都是整数：
 *       500K->6   1M->3   250K->12   125K->24   100K->30
 *   只动 Prescaler、BS1/BS2/SJW 保持不变，改动面最小。
 *   ⚠️ 以后改主频/APB1 分频（比如 APB1 变成 84MHz），这张表必须重算。
 *
 * 报文：请求 04 <addr> 03 <code>；应答 LEN=4，status=0 成功（应答是在**旧**速率上回的）
 * ========================================================================= */

/* ------------------------------ 测试参数 ------------------------------ */
static constexpr SensorBaudCode kTargetBaud = SensorBaudCode::Mbps1;  ///< 想测哪个就改这里
static constexpr bool kRestoreBaud = true;   ///< 测完切回 500K（强烈建议开着，否则下次开机会失联）
static constexpr uint32_t kPollMs = 20U;     ///< 轮询周期
static constexpr uint32_t kProbeMs = 600U;   ///< 单个速率上观察多久（≈6 个应答超时）
static constexpr uint32_t kScanProbeMs = 400U; ///< 扫速率时每个看多久（5 个最坏 2s）
static constexpr uint32_t kSwitchMs = 100U;  ///< 发完 0x03 等传感器执行（含它在旧速率上回应答）

/* ------------------------------ 波特率表 ------------------------------ */
struct BaudEntry
{
    SensorBaudCode code;
    uint32_t prescaler;  ///< CAN_Prescaler = 42MHz / (14 × 波特率)
};

/* 只改 Prescaler：BS1/BS2/SJW 沿用 can.c 的 10/3/2，采样点都是 78.6% */
static constexpr BaudEntry kBaudTable[] = {
    {SensorBaudCode::Kbps500, 6U},   ///< 出厂默认，也是本工程的常态
    {SensorBaudCode::Mbps1, 3U},
    {SensorBaudCode::Kbps250, 12U},
    {SensorBaudCode::Kbps125, 24U},
    {SensorBaudCode::Kbps100, 30U},
};
static constexpr uint8_t kBaudNone = 0xFFU;  ///< g_baud_found 的"没扫到"标记

/* ------------------------------ 线程资源 ------------------------------ */
/* 栈为什么是静态数组而不是 tx_byte_allocate：
 *   TX_APP_MEM_POOL_SIZE 只有 1024 字节（AZURE_RTOS/App/app_azure_rtos_config.h），
 *   从字节池里要 1KB 栈必然失败，所以栈和控制块都做成静态变量。
 * 为什么必须显式 aligned(8)：
 *   ThreadX 要求线程栈起始地址 8 字节对齐（Cortex-M4 栈帧按双字压栈），
 *   ULONG 数组自身只保证 4 字节对齐，不加属性属于"能跑但是运气好"。 */
static constexpr ULONG kSensorStackBytes = 1024U;
static constexpr UINT kSensorPriority = 10U;   ///< 数值越小优先级越高；0 留给定时器线程
static constexpr uint32_t kHeartbeatMs = 500U; ///< 蓝灯心跳半周期

static TX_THREAD sensor_thread;
static ULONG sensor_stack[kSensorStackBytes / sizeof(ULONG)] __attribute__((aligned(8)));

/* 调试量故意做成全局（非 static）：Live Watch 里搜得到、停在调度器空转里也能求值。
 * 前面踩过坑 —— 文件内静态变量和函数局部静态在那种停止位置会显示 not available。 */
volatile uint32_t g_baud_stage = 0U;    ///< 0=未开始 1=查500K 2=已发0x03 3=切主机 4=已切换 5=切回 6=完成 0xEE=失败
volatile uint32_t g_baud_result = 0xFFU;///< 0=全过；0xE1~0xE7 是失败点，对照 TestFailed 的注释
volatile uint32_t g_baud_host = 0U;     ///< 主机当前位时序的波特率码（0=500K 1=1M 2=250K 3=125K 4=100K）
volatile uint32_t g_baud_found = 0xFFU; ///< 扫描时传感器实际停在哪个速率；0xFF=一个都不通
volatile uint32_t g_baud_ack = 0xFFU;   ///< 0x03 的应答状态码（0=传感器接受了新速率）
volatile uint32_t g_baud_ack_func = 0xFFU; ///< 那条应答的 FUNC（应为 0x03）
volatile uint32_t g_baud_frames = 0U;   ///< pos_rx_count 快照：在涨说明当前速率能收到位置帧

[[noreturn]] static void SensorThread_Entry(ULONG thread_input);

/** @brief 取某个波特率对应的 CAN 预分频；表里没有就返回 0。 */
static uint32_t PrescalerOf(SensorBaudCode code)
{
    for (const BaudEntry &e : kBaudTable)
    {
        if (e.code == code)
            return e.prescaler;
    }
    return 0U;
}

/**
 * @brief 把**主机**的 CAN 位时序切到 code，并重新起外设。
 * @param code 目标波特率
 * @retval true 切完且外设已重新进入 LISTENING（中断也重新打开了）
 * @note  顺序必须是 Stop -> 改 hcan1.Init -> HAL_CAN_Init -> CAN_Init()。
 *        最后一步复用 bsp 的 CAN_Init()：它负责"重配滤波器 + Start + 开接收/错误中断"，
 *        少了它会出现"初始化成功了但一个中断都收不到"（Start 之后 IER 是空的）。
 *        ⚠️ 这是测试代码，只在本测试里用；正式逻辑里切波特率应该由 bsp 提供接口。
 */
static bool HostSetBaud(SensorBaudCode code)
{
    const uint32_t prescaler = PrescalerOf(code);
    if (prescaler == 0U)
        return false;

    /* 只有 LISTENING 状态下 Stop 才会真的进初始化模式；其它状态它会直接返回错误，
     * 这里忽略返回值即可 —— 下面 HAL_CAN_Init 不检查状态，照样能重配 BTR。 */
    (void)HAL_CAN_Stop(&hcan1);

    hcan1.Init.Prescaler = prescaler;
    hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;      ///< 与 can.c 保持一致：14 TQ/bit，采样点 78.6%
    hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
    hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
        return false;

    if (!CAN_Init())  // 滤波器 + Start + 中断
        return false;

    g_baud_host = static_cast<uint32_t>(code);
    return true;
}

/**
 * @brief 在当前位时序上轮询，看有没有位置帧回来。
 * @param timeout_ms 观察时长
 * @retval true 这段时间内收到了至少一帧位置应答
 * @note  调用前必须先 HostSetBaud() 把主机切到想试的那个速率。
 */
static bool ProbeHere(uint32_t timeout_ms)
{
    DisplacementSensor *sensor = DisplacementSensor::Instance();
    const uint32_t frames0 = sensor->pos_rx_count;
    const uint32_t t0 = tx_time_get();

    do
    {
        sensor->Poll();
        tx_thread_sleep(kPollMs);
        sensor->Update();
        if (sensor->pos_rx_count != frames0)
        {
            return true;
        }
    } while (tx_time_get() - t0 < timeout_ms);

    return false;
}

/**
 * @brief 把 5 个波特率逐个试一遍，找出传感器现在停在哪个速率上。
 * @retval SensorBaudCode 的数值；kBaudNone(0xFF) 表示一个都不通
 * @note  返回时**主机停在找到的那个速率上**（没找到就停在最后试的那个），
 *        方便调用方接着发 0x03 把传感器拉回 500K。
 *        这是换波特率唯一的自救手段：不用知道它到底变成了多少，扫一遍就知道。
 */
static uint8_t ScanBaud(void)
{
    for (const BaudEntry &e : kBaudTable)
    {
        if (!HostSetBaud(e.code))
            continue;
        tx_thread_sleep(20U);  // 等外设起稳（刚离开初始化模式，第一帧容易发不出去）
        if (ProbeHere(kScanProbeMs))
        {
            return static_cast<uint8_t>(e.code);
        }
    }
    return kBaudNone;
}

/**
 * @brief 在当前速率上把传感器切回 500K，并让主机也跟着回 500K。
 * @param sensor 传感器实例
 * @retval true 切回后能正常通信
 * @note  失败路径专用：传感器要是被留在 1M 而我们不管，下次开机会以为它"坏了"。
 */
static bool RestoreDefaultBaud(DisplacementSensor *sensor)
{
    sensor->last_status = 0xFFU;
    sensor->last_ack_func = 0xFFU;
    if (!sensor->SetBaudRate(SensorBaudCode::Kbps500))  // 用当前速率发 03 ... 00
        return false;
    tx_thread_sleep(kSwitchMs);
    if (!HostSetBaud(SensorBaudCode::Kbps500))
        return false;
    tx_thread_sleep(20U);
    return ProbeHere(kProbeMs);
}

/**
 * @brief 测试失败：记下失败码，红灯常亮并停住（现场看 g_baud_result 就知道卡在哪一步）。
 * @param code 失败码
 *
 * 失败码对照：
 *   0xE1 500K 上就收不到位置帧     -> 接线/供电/传感器没上电，或它现在停在别的速率上
 *                                    （上次测试中途掉电会这样，扫过的结果在 g_baud_found）
 *   0xE2 切到目标速率后不通        -> 0x03 丢了（传感器还在 500K）或分频算错。已全扫一遍，
 *                                    g_baud_found 是它实际的位置；能通信的话已经拉回 500K
 *   0xE3 切回 500K 后不通          -> 见 g_baud_found，照着它重新配一次就能救回来
 *   0xE5 0x03 帧没发出去           -> 邮箱满 / 总线不通
 *   0xE6 主机位时序切不过去        -> HAL_CAN_Init / CAN_Init 失败（看 g_can_stat）
 *   0xE7 切回 500K 的 0x03 没发出去
 */
[[noreturn]] static void TestFailed(uint32_t code)
{
    g_baud_result = code;
    g_baud_stage = 0xEEU;
    LED_SetAll(1U, 0U, 0U);  // 红灯常亮
    for (;;)
    {
        tx_thread_sleep(1000U);
    }
}

/**
 * @brief 创建传感器线程（调度器启动前调用，本函数不阻塞）。
 * @note  TX_AUTO_START：创建即就绪，tx_kernel_enter 之后立刻开始跑。
 */
void SensorThread_Init(void)
{
    (void)tx_thread_create(&sensor_thread,
                           (CHAR *)"displacement sensor",
                           SensorThread_Entry,
                           0U,
                           sensor_stack,
                           kSensorStackBytes,
                           kSensorPriority,
                           kSensorPriority,
                           TX_NO_TIME_SLICE,
                           TX_AUTO_START);
}

/**
 * @brief 换波特率测试线程：500K 验证 -> 发 0x03 -> 主机跟随 -> 验证 -> 切回 500K -> 常态轮询。
 * @param thread_input 未使用（tx_thread_create 的第 4 个参数）
 *
 * 现象（Live Watch 看 g_baud_stage / g_baud_result / g_baud_host / g_baud_found）：
 *   stage=1 正在 500K 上确认通信
 *   stage=2 已发 0x03，等传感器执行（g_baud_ack 应该是 0x00）
 *   stage=3/4 主机已切到新速率，正在验证
 *   stage=5 正在切回 500K
 *   stage=6 完成，绿灯常亮 + 蓝灯 0.5s 心跳
 *   stage=0xEE 失败，红灯常亮，原因看 g_baud_result
 */
[[noreturn]] static void SensorThread_Entry(ULONG thread_input)
{
    (void)thread_input;

    DisplacementSensor *sensor = DisplacementSensor::Instance();

    /* 配置非法（node_id=0 / 分辨率<=0 / timeout < poll_period）：蓝灯常亮后停住 */
    if (!sensor->Init())
    {
        LED_SetAll(0U, 0U, 1U);
        for (;;)
        {
            tx_thread_sleep(1000U);
        }
    }

    /* 前置：把回传方式拉回查询模式。自动回传存在传感器里，上一次测试如果切过去了，
     * 这里的轮询会被自动回传挤掉，看起来像"换波特率失败"。 */
    {
        (void)sensor->SetMode(SensorMode::Query);
        tx_thread_sleep(50U);
    }

    /* ---------------- 阶段 1：先在当前（500K）确认通信 ---------------- */
    g_baud_stage = 1U;
    g_baud_host = static_cast<uint32_t>(SensorBaudCode::Kbps500);

    if (!ProbeHere(kProbeMs))
    {
        /* 出厂 500K 都问不到：先扫一遍，看它是不是上次被留在别的速率上了 */
        g_baud_found = ScanBaud();
        TestFailed(0xE1U);
    }

    /* ---------------- 阶段 2：下发 0x03 ---------------- */
    g_baud_stage = 2U;

    sensor->last_status = 0xFFU;  // 清掉 Init() 里 0x0B 的应答，才能确认这一次的结果
    sensor->last_ack_func = 0xFFU;
    if (!sensor->SetBaudRate(kTargetBaud))
    {
        TestFailed(0xE5U);  // 帧没进邮箱
    }

    tx_thread_sleep(kSwitchMs);  // 传感器执行 + 在**旧**速率上回那条应答
    g_baud_ack = sensor->last_status;
    g_baud_ack_func = sensor->last_ack_func;

    /* ---------------- 阶段 3：主机跟着切到目标速率 ---------------- */
    g_baud_stage = 3U;
    if (!HostSetBaud(kTargetBaud))
    {
        (void)HostSetBaud(SensorBaudCode::Kbps500);  // 切不动就先切回去，尽量别失联
        TestFailed(0xE6U);
    }
    tx_thread_sleep(20U);

    /* ---------------- 阶段 4：在新速率上验证 ---------------- */
    g_baud_stage = 4U;
    if (!ProbeHere(kProbeMs))
    {
        /* 目标速率不通：不猜，直接把 5 个速率全扫一遍，看它到底停在哪 */
        g_baud_found = ScanBaud();
        if (g_baud_found != kBaudNone)
        {
            (void)RestoreDefaultBaud(sensor);  // 能通信就顺手拉回 500K，别把传感器留在半路上
        }
        TestFailed(0xE2U);
    }

    /* ---------------- 阶段 5：切回 500K（强烈建议保留） ---------------- */
    g_baud_stage = 5U;
    if (kRestoreBaud)
    {
        sensor->last_status = 0xFFU;
        sensor->last_ack_func = 0xFFU;
        if (!sensor->SetBaudRate(SensorBaudCode::Kbps500))
        {
            TestFailed(0xE7U);
        }
        tx_thread_sleep(kSwitchMs);

        if (!HostSetBaud(SensorBaudCode::Kbps500))
        {
            TestFailed(0xE6U);
        }
        tx_thread_sleep(20U);

        if (!ProbeHere(kProbeMs))
        {
            g_baud_found = ScanBaud();
            TestFailed(0xE3U);  // 照 g_baud_found 重新配一次就能救回来
        }
    }

    /* ---------------- 阶段 6：完成，常态化轮询 ---------------- */
    g_baud_stage = 6U;
    g_baud_result = 0U;

    for (;;)
    {
        sensor->Poll();
        tx_thread_sleep(kPollMs);
        sensor->Update();

        g_baud_frames = sensor->pos_rx_count;

        /* 绿常亮 + 蓝 0.5s 心跳 = 当前速率通信正常；蓝常亮 = 掉线 */
        const uint32_t now = tx_time_get();
        if (sensor->state == DisplacementSensor::SENSOR_ONLINE)
        {
            LED_SetAll(0U, 1U, ((now / kHeartbeatMs) & 1U) ? 1U : 0U);
        }
        else
        {
            LED_SetAll(0U, 0U, 1U);
        }
    }
}
