#include "thread.hpp"

#include "bsp_led.hpp"
#include "displacement_sensor.hpp"

/* ============================== 线程资源 ==================================
 * 栈为什么是静态数组而不是 tx_byte_allocate：
 *   TX_APP_MEM_POOL_SIZE 只有 1024 字节（AZURE_RTOS/App/app_azure_rtos_config.h），
 *   从字节池里要 1KB 栈必然失败，所以栈和控制块都做成静态变量。
 * 为什么必须显式 aligned(8)：
 *   ThreadX 要求线程栈起始地址 8 字节对齐（Cortex-M4 栈帧按双字压栈），
 *   ULONG 数组自身只保证 4 字节对齐，不加属性属于"能跑但是运气好"。
 * ========================================================================= */
static constexpr ULONG kSensorStackBytes = 1024U;     ///< 线程栈大小（字节），只用浮点换算，1KB 够用
static constexpr UINT kSensorPriority = 10U;          ///< 数值越小优先级越高；0 留给定时器线程
static constexpr uint8_t kVelocityPollEvery = 5U;     ///< 每 5 拍位置插 1 拍速度（共用应答通道，别每拍都发）
static constexpr uint32_t kHeartbeatMs = 500U;        ///< 蓝灯心跳半周期

static TX_THREAD sensor_thread;
static ULONG sensor_stack[kSensorStackBytes / sizeof(ULONG)] __attribute__((aligned(8)));

[[noreturn]] static void SensorThread_Entry(ULONG thread_input);

/**
 * @brief 创建传感器轮询线程（调度器启动前调用，本函数不阻塞）。
 * @note  TX_AUTO_START：创建即就绪，tx_kernel_enter 之后立刻开始跑。
 *        优先级/抢占门限都填同一个值，不走抢占门限机制（tx_user.h 里已禁用它）。
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
 * @brief 传感器线程主体：每拍"发读指令 -> 睡一个周期 -> 换算 + 判在线 -> 刷灯"。
 * @param thread_input 未使用（tx_thread_create 的第 4 个参数）
 *
 * 顺序上的两个坑：
 *   1. 先 Poll 再 sleep：应答帧是在 sleep 期间由 CAN 中断收进 OnFrame 的，
 *      醒来紧接着 Update 就能读到这一拍的数据，延迟只有一个周期。
 *      反过来（先 sleep 再 Poll）会白等一个周期才拿到数据。
 *   2. Init 放在线程里而不是 TestThread_Init 里：Init 会下发一条 0x0B 指令，
 *      在调度器起来后发才能拿到返回值、也方便红灯报错。
 */
[[noreturn]] static void SensorThread_Entry(ULONG thread_input)
{
    (void)thread_input;

    DisplacementSensor *sensor = DisplacementSensor::Instance();

    /* 配置非法（node_id=0 / 分辨率<=0 / timeout < poll_period）：蓝灯常亮后停住，
     * 现场用 Live Watch 看 sensor->cfg 就知道哪一项写错了。 */
    if (!sensor->Init())
    {
        LED_SetAll(0U, 0U, 1U);
        for (;;)
        {
            tx_thread_sleep(1000U);
        }
    }

    /* poll_period_ms 允许被配成 0（Init 只校验 timeout >= period），
     * 直接拿去做 sleep 会变成"让出 CPU 后立刻回来"的满速空转，把邮箱打爆，这里兜一下。 */
    const ULONG period = (sensor->cfg.poll_period_ms == 0U) ? 1U : sensor->cfg.poll_period_ms;
    uint8_t velocity_div = 0U;
    sensor->SetZeroHard();
    for (;;)
    {
        /* 1) 查询模式（默认）下传感器不主动广播：这一拍不发读指令，下一拍就没有新帧 */
        sensor->Poll();

        /* 2) 速度帧和位置帧共用同一条应答通道，每拍都发会把位置刷新率拖垮，隔几拍插一次 */
        if (++velocity_div >= kVelocityPollEvery)
        {
            velocity_div = 0U;
            sensor->PollVelocity();
        }

        /* 3) 睡满一个轮询周期，期间不占 CPU（应答帧走 CAN 中断） */
        tx_thread_sleep(period);

        /* 4) 浮点换算 + 在线判定都在这里做（中断里只存原始值） */
        sensor->Update();

        /* 5) 灯语：绿=在线（蓝 0.5s 心跳），红=离线。
         *    直接用 Update() 维护好的 state，省一次 AliveCheck。 */
        if (sensor->state == DisplacementSensor::SENSOR_ONLINE)
        {
            LED_SetAll(0U, 1U, ((tx_time_get() / kHeartbeatMs) & 1U) ? 1U : 0U);
        }
        else
        {
            LED_SetAll(0U, 0U, 1U);
        }
    }
}
