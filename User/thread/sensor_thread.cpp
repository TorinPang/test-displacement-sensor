#include "thread.hpp"

#include "bsp_led.hpp"
#include "displacement_sensor.hpp"

/* ============================== 本线程测什么 ==============================
 * 目标：验证传感器的"自动回传"能力 —— 主机只切一次模式，之后完全不发读指令，
 *       让传感器按固定周期自己往总线上发数据。
 *
 * 和查询模式的区别（重要）：
 *   查询模式：线程每 30ms 发 04 01 01 00，传感器才回一帧；不发就一帧都没有。
 *   自动回传：发 05(设周期) + 04(设模式) 之后，主机闭嘴，传感器自己发。
 *             **此时绝不能再调 Poll()/PollVelocity()**，否则轮询指令和自动回传
 *             会撞在同一条应答通道上（表现为位置帧丢一半、速度帧和位置帧交替错乱）。
 *
 * 本文件不动 module 层：0x05/0x04 直接用 DisplacementSensor 的公开接口组帧。
 * ========================================================================= */

/* ------------------------------ 测试参数 ------------------------------ */
static constexpr SensorMode kAutoMode = SensorMode::AutoVelocity;  ///< 想测速度回传就改成 AutoVelocity
static constexpr uint16_t kAutoPeriodUs = 30000U;                  ///< 自动回传周期，单位 **us**（30ms）；手册范围 50~65535
static constexpr ULONG kLoopPeriodMs = 10U;        ///< 主循环节拍：比回传周期快，帧一到就能算出来
static constexpr uint32_t kRetryMs = 2000U;        ///< 连续这么久没有**数据帧**进来，就重发一次模式指令

/* ------------------------------ 线程资源 ------------------------------ */
/* 栈为什么是静态数组而不是 tx_byte_allocate：
 *   TX_APP_MEM_POOL_SIZE 只有 1024 字节（AZURE_RTOS/App/app_azure_rtos_config.h），
 *   从字节池里要 1KB 栈必然失败，所以栈和控制块都做成静态变量。
 * 为什么必须显式 aligned(8)：
 *   ThreadX 要求线程栈起始地址 8 字节对齐（Cortex-M4 栈帧按双字压栈），
 *   ULONG 数组自身只保证 4 字节对齐，不加属性属于"能跑但是运气好"。 */
static constexpr ULONG kSensorStackBytes = 1024U;  ///< 只用浮点换算，1KB 够用（实测栈帧 32B）
static constexpr UINT kSensorPriority = 10U;       ///< 数值越小优先级越高；0 留给定时器线程
static constexpr uint32_t kHeartbeatMs = 500U;     ///< 蓝灯心跳半周期

static TX_THREAD sensor_thread;
static ULONG sensor_stack[kSensorStackBytes / sizeof(ULONG)] __attribute__((aligned(8)));

/* 调试量故意做成全局（非 static）：Live Watch 里搜得到、停在调度器空转里也能求值。
 * 前面踩过坑 —— 文件内静态变量和函数局部静态在那种停止位置会显示 not available。 */
volatile uint32_t g_auto_ready = 0U;        ///< 1 = 模式指令已装进邮箱（**不代表传感器已执行**，执行结果看 last_status）
volatile uint32_t g_auto_retry_count = 0U;  ///< 重发模式指令的次数。稳定运行时应停在 0 或 1
volatile uint32_t g_auto_data_frames = 0U;  ///< 数据帧总数快照（pos+vel），用来肉眼判断回传有没有在跑
volatile uint32_t g_report_mode = 0U;       ///< 期望的回传方式：0xAA=位置 / 0x02=速度 / 0=查询

[[noreturn]] static void SensorThread_Entry(ULONG thread_input);

/**
 * @brief 组帧并下发"切自动回传"指令：先 0x05 设周期，再 0x04 设模式。
 * @param sensor    传感器实例
 * @param mode      AutoPosition / AutoVelocity / Query
 * @param period_us 自动回传周期，单位 **us**（50~65535，30ms 写 30000）
 * @retval true 两帧都已装入发送邮箱（**不代表传感器已执行**，结果看 last_status / last_ack_func）
 * @note  顺序必须是先周期后模式：反了会先用传感器里的旧周期回传一小段。
 *        退回 Query 时只发 0x04 不碰周期。
 */
static bool StartAutoReport(DisplacementSensor *sensor, SensorMode mode, uint16_t period_us)
{
    const uint8_t param = static_cast<uint8_t>(mode);

    if (mode == SensorMode::Query)
    {
        g_report_mode = 0U;
        return sensor->SendInstruction(InstructionType::SetMode, &param, 1U);
    }

    /* 50us 是手册下限；再快 500kbps 总线也扛不住 */
    if (period_us < 50U)
        return false;

    /* 0x05：DATA 为小端 u16，单位 us。模块里的 SetReportPeriod 就是干这个的，直接复用 */
    if (!sensor->SetReportPeriod(period_us))
        return false;

    /* 0x04：DATA 一个字节 = 回传方式 */
    if (!sensor->SendInstruction(InstructionType::SetMode, &param, 1U))
        return false;

    g_report_mode = param;
    return true;
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
 * @brief 自动回传测试线程：切一次模式，然后只收不发。
 * @param thread_input 未使用（tx_thread_create 的第 4 个参数）
 *
 * 怎么判断"自动回传通没通"（Live Watch 里看）：
 *   pos_rx_count / vel_rx_count —— 在涨 = 数据帧确实自己来了
 *   err_count                   —— 在涨 = 帧到了但被判成坏帧
 *   last_ack_func / last_status —— 切换指令有没有被接受（0x04 + 0x00 = 成功）
 *   g_auto_retry_count          —— 应该停在 0/1；一直涨说明一帧数据都没收到，在反复重发
 *
 * 特别注意 err_count 这一项：如果它猛涨而 pos_rx_count 不动，说明自动回传的帧头
 * **不是 0x01**（很可能是 0x04，说明书没写清），会被模块的 OnFrame 当设置类应答丢掉。
 * 这种情形的解决办法是改 module 的 OnFrame 按 LEN 兼容，但本次按要求没动模块。
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

    sensor->SetZeroHard();

    /* 切自动回传。注意 Init() 里刚发过一条 0x0B，它的应答也走这条通道，不影响。 */
    g_auto_ready = StartAutoReport(sensor, kAutoMode, kAutoPeriodUs) ? 1U : 0U;

    uint32_t last_data_frames = sensor->pos_rx_count + sensor->vel_rx_count;
    uint32_t last_retry_ms = tx_time_get();

    for (;;)
    {
        tx_thread_sleep(kLoopPeriodMs);

        /* 1) 换算 + 判在线。自动回传模式下没有要发的东西，只读最新一帧 */
        sensor->Update();

        /* 2) 兜底重发：一帧数据都没进来就重发模式指令。
         *    典型场景：传感器上电比主机慢、总线还没起来时那条指令被丢、传感器被复位过。 */
        const uint32_t data_frames = sensor->pos_rx_count + sensor->vel_rx_count;
        const uint32_t now = tx_time_get();
        if (data_frames == last_data_frames)
        {
            if (now - last_retry_ms >= kRetryMs)
            {
                g_auto_ready = StartAutoReport(sensor, kAutoMode, kAutoPeriodUs) ? 1U : 0U;
                g_auto_retry_count++;
                last_retry_ms = now;
            }
        }
        else
        {
            last_data_frames = data_frames;
            g_auto_data_frames = data_frames;
            last_retry_ms = now;
        }

        /* 3) 灯语：在线 = 绿常亮 + 蓝 0.5s 心跳；离线 = 蓝常亮。
         *    直接用 Update() 维护好的 state，省一次 AliveCheck。 */
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
