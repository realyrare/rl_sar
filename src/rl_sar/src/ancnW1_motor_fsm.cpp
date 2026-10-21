/*
 * Copyright (c) 2024-2025 ancn
 * SPDX-License-Identifier: Apache-2.0
 *
 * ancnW1 电机状态机实现
 * ----------------------------------------
 * 16 个电机需要 3 帧才能全部覆盖（每帧 6 槽）：
 *   帧 1：电机 1-6（IDs = {1,2,3,4,5,6}）
 *   帧 2：电机 7-12
 *   帧 3：电机 13-16（剩余 2 槽填空，对应 IDs = {13,14,15,16,0,0}）
 *
 * 反馈：每电机独立一帧（手册 §3.3.2：12 字节），由 CanInterface 后台线程缓存
 */

#include "ancnW1_motor_fsm.hpp"

#include <iostream>
#include <algorithm>

namespace ancn_w1
{

/**
 * @brief 构造函数：注入底层依赖并初始化默认电机 ID 列表
 *
 * @param can    CAN 接口实例（裸指针，**不接管所有权**）
 * @param driver 协议封装实例（裸指针，**不接管所有权**）
 *
 * @note 采用裸指针而非智能指针是刻意的：本对象是 RL_Real_AncnW1 的成员变量，
 *       而 can_/driver_ 同样是它的成员，三者同生命周期且由后者统一销毁。
 *       用 shared_ptr 反而会掩盖真实的所有权关系。
 *       约束：必须在 Start() 之前保证这两个指针有效。
 *
 * @note 默认按"单总线 1~16 连续编号"填充 motor_ids_。
 *       若厂商答复需要拆多条 CAN 总线（README §待确认问题 Q2），
 *       应调用 SetMotorIds() 覆盖成实际编号。
 */
MotorFsm::MotorFsm(CanInterface *can, MotorDriver *driver)
    : can_(can), driver_(driver)
{
    // 默认 ancnW1 = 电机 1-16
    for (size_t i = 0; i < CanInterface::kMaxMotorId; ++i)
    {
        motor_ids_[i] = static_cast<uint8_t>(i + 1);
    }
    motor_count_ = CanInterface::kMaxMotorId;
}

/**
 * @brief 覆盖本状态机管理的电机 ID 列表
 *
 * @param ids 电机编号数组（长度固定 16）。
 *            其中不在 [1,16] 范围内的元素视为"空位"并被忽略。
 *
 * @note motor_count_ 用 count_if 统计**有效 ID 的个数**，而不是数组长度。
 *       这样后续 Tick() 遍历、SendCommandFrames() 分帧都只处理真实存在的电机，
 *       避免对不存在的电机发帧或误判"某电机掉线"。
 */
void MotorFsm::SetMotorIds(const std::array<uint8_t, CanInterface::kMaxMotorId> &ids)
{
    motor_ids_ = ids;
    motor_count_ = std::count_if(ids.begin(), ids.end(),
                                   [](uint8_t id) { return id >= 1 && id <= CanInterface::kMaxMotorId; });
}

/**
 * @brief 查询当前状态（供 RL 层判断是否允许下发运行指令）
 * @return 当前 MotorFsmState
 *
 * @note 无锁读取 state_：本类的所有方法都被设计为在**同一个控制线程**内调用
 *       （RobotControl → Tick / SetCommand → State），因此不存在数据竞争。
 *       若将来引入外部的跨线程查询，需要把 state_ 改为 atomic 或加互斥。
 */
MotorFsmState MotorFsm::State() const
{
    return state_;
}

/**
 * @brief 状态枚举 → 可打印字符串（供日志与人机界面显示）
 * @param s 状态枚举
 * @return 静态字符串常量，不需要调用方释放
 *
 * @note 所有 case 已穷举，末尾的 return "?" 只是兜底
 *       ——防止将来新增枚举值时编译器未告警而返回随机值。
 */
const char *MotorFsm::StateName(MotorFsmState s)
{
    switch (s)
    {
    case MotorFsmState::Disabled:   return "Disabled";
    case MotorFsmState::Enabling:   return "Enabling";
    case MotorFsmState::Enabled:    return "Enabled";
    case MotorFsmState::Disabling:  return "Disabling";
    case MotorFsmState::Zeroing:    return "Zeroing";
    case MotorFsmState::Fault:      return "Fault";
    }
    return "?";
}

/**
 * @brief 请求使能全部电机
 *
 * 下发使能帧后进入 Enabling，由 Tick() 轮询各电机的状态码 bit0，
 * 全部变 1 且持续满足 kEnableSettleTime 后才算真正 Enabled。
 *
 * @note **必须先 ClearFault()**。这里拒绝从 Fault 直接进入，
 *       是要求操作者显式确认故障，避免程序自动反复尝试使能
 *       而过流/过温的电机被反复上电。
 *
 * @note 之所以要先 Enabling 而不是直接 Enabled：
 *       使能指令是单向广播、无从站应答，必须靠反馈帧里 bit0 的变化来确认，
 *       这个确认过程需要若干毫秒时间。
 */
void MotorFsm::Enable()
{
    if (state_ == MotorFsmState::Fault)
    {
        std::cerr << "[MotorFsm] cannot Enable from Fault; call ClearFault() first" << std::endl;
        return;
    }
    SendCommandFrames(0xFC);  // 使能指令码（手册 §3.3.1.1）
    state_ = MotorFsmState::Enabling;
    state_enter_time_ = std::chrono::steady_clock::now();
    std::cout << "[MotorFsm] Enable() → Enabling" << std::endl;
}

/**
 * @brief 请求失能（断电自由态）
 *
 * @note **刻意不加任何状态前置检查** —— 包括 Fault 在内的任何状态下都能执行。
 *       这是安全设计：失能等价于"松手/泄力"，必须无条件生效，
 *       绝不允许因为状态判断而让"停机"指令被吞掉。
 */
void MotorFsm::Disable()
{
    SendCommandFrames(0xFD);  // 失能指令码（手册 §3.3.1.2）
    state_ = MotorFsmState::Disabling;
    state_enter_time_ = std::chrono::steady_clock::now();
    std::cout << "[MotorFsm] Disable() → Disabling" << std::endl;
}

/**
 * @brief 把当前位置设为机械零位（标定用）
 *
 * @warning 要求当前处于 **Enabled**。
 *          设零位会让电机在过程中短暂失能并重算零点，
 *          若在运行中调用可能导致关节位置跳变、机器人姿态突变。
 *          因此必须先在 Enabled 且静止、 robot 已摆好姿态时进行。
 *
 * @note Zero 过程会让电机短暂失去使能位，
 *       所以 Tick() 在检测到零位完成后会**自动补发一次使能帧**，
 *       无需外部再调 Enable()。
 */
void MotorFsm::Zero()
{
    if (state_ != MotorFsmState::Enabled)
    {
        std::cerr << "[MotorFsm] Zero() requires Enabled state, currently " << StateName(state_) << std::endl;
        return;
    }
    SendCommandFrames(0xEF);  // 设零位指令码（手册 §3.3.1.4）
    state_ = MotorFsmState::Zeroing;
    state_enter_time_ = std::chrono::steady_clock::now();
    std::cout << "[MotorFsm] Zero() → Zeroing" << std::endl;
}

/**
 * @brief 清除故障状态，回到 Disabled
 *
 * @note 回到的是 **Disabled 而不是 Enabled**：
 *       故障后电机必然已失能，不可能"恢复"，必须重新走一遍
 *       ClearFault → Enable 的完整流程。这样语义清晰，
 *       也避免外部误以为调完就能直接跑。
 *
 * @note 本函数只清软件标记，**不向总线发任何帧**，
 *       也不重置 RecordFault() 记录的上次故障码（保留以便事后追溯）。
 */
void MotorFsm::ClearFault()
{
    state_ = MotorFsmState::Disabled;
    state_enter_time_ = std::chrono::steady_clock::now();
    std::cout << "[MotorFsm] ClearFault() → Disabled" << std::endl;
}

/**
 * @brief 状态机推进，由 RobotControl() 每个控制周期（200Hz）调用一次
 *
 * ============================ 状态转移图 ============================
 *
 *        Enable()            全部 motor bit0=1 且 elapsed ≥ kEnableSettleTime
 *   Disabled ─────────→ Enabling ───────────────────────────────────→ Enabled
 *      ↑                  │  │                                          │
 *      │                  │  └─ 超时 2s 未全部使能 ─→ Fault              │ Zero()
 *      │                  ↓                                             ↓
 *      │                Fault ←───────────────────────────────────  Zeroing
 *      │                  ↑                                             │
 *      │   Disable()      │          Zero 完成后自动补发一次使能帧        │
 *      └────────────── Disabling                                        │
 *                          │                                            │
 *                          └─ 全部 bit0=0 → Disabled                    │
 *                             超时 2s     → Fault                       │
 *                                            elapsed ≥ kZeroSettleTime ─┘
 *
 *   ⚠ 任何状态下检测到故障都会直接跳到 Fault，必须人工 ClearFault() 才能恢复，
 *     Fault 状态下 SetCommand 不再下发任何运行帧（见 rl_real_ancnW1.cpp）。
 *
 * ============================ 每周期做两件事 ============================
 *
 *   ① 故障检测（所有状态都执行）：逐个读 16 个电机的反馈
 *        · 长时间（> kCommTimeout）没收到反馈     → CommTimeout 故障
 *        · 状态码 bit1~bit6 任一置位              → 对应类型的硬件故障
 *
 *   ② 状态推进：按 switch(state_) 检查转移条件
 *
 * @note 转移条件依赖 elapsed（自**进入当前状态**起经过的时长），
 *       每次状态变化都会重置 state_enter_time_，所以各阶段超时互不干扰。
 *
 * @note 为什么 Enabled 状态不再重发使能帧？
 *       手册未要求周期性重发使能，且使能帧会让电机短暂脱离运行模式。
 *       使能状态的维持由 SetCommand 持续下发运行帧自然保证。
 */
void MotorFsm::Tick()
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - state_enter_time_);

    // 1) 故障检测（任何状态都检查）
    for (size_t i = 0; i < motor_count_; ++i)
    {
        uint8_t mid = motor_ids_[i];
        std::array<uint8_t, CanInterface::kFeedbackLen> raw{};
        if (!can_->GetLatestFeedback(mid, &raw))
        {
            // 未收到反馈 → 检查是否超时
            if (elapsed > kCommTimeout && state_ != MotorFsmState::Disabled)
            {
                RecordFault(mid, MotorFault::CommTimeout);
                state_ = MotorFsmState::Fault;
                std::cerr << "[MotorFsm] motor " << static_cast<int>(mid)
                          << " no feedback for " << elapsed.count() << "ms → Fault" << std::endl;
                return;
            }
            continue;
        }

        MotorFeedback fb;
        if (!driver_->ParseFeedback(raw.data(), raw.size(), &fb))
        {
            continue;
        }

        // 检查故障位：状态码 bit1~bit6 任一为 1 即判定故障
        //
        // 掩码 0x7E = 0b0111_1110，刻意排除了两个位：
        //   · bit0 = 电机使能状态（表示该电机已使能，是正常状态不是故障）
        //   · bit7 = 手册 §3.3.2 标注为"预留"，不可用作判据
        //
        // 手册 §3.3.3 定义的各位含义：
        //   bit1 过压 / bit2 过流 / bit3 过温 / bit4 过速
        //   bit5 编码器故障 / bit6 欠压或预驱故障
        // 发生故障时电机会自行失能，同时把对应状态位置 1。
        if (fb.status & 0x7E)
        {
            MotorFault ft = MotorFault::None;
            if (fb.status & 0x02) ft = MotorFault::Overvoltage;
            else if (fb.status & 0x04) ft = MotorFault::Overcurrent;
            else if (fb.status & 0x08) ft = MotorFault::Overtemperature;
            else if (fb.status & 0x10) ft = MotorFault::Overspeed;
            else if (fb.status & 0x20) ft = MotorFault::EncoderFault;
            else if (fb.status & 0x40) ft = MotorFault::Undervoltage;

            RecordFault(mid, ft);
            state_ = MotorFsmState::Fault;
            std::cerr << "[MotorFsm] motor " << static_cast<int>(mid)
                      << " fault 0x" << std::hex << static_cast<int>(fb.status)
                      << " (" << StateName(state_) << " → Fault)" << std::dec << std::endl;
            return;
        }
    }

    // 2) 状态推进
    switch (state_)
    {
    case MotorFsmState::Enabling:
        if (AllMotorsHaveBit(0x01, true))
        {
            // 所有电机已使能
            if (elapsed >= kEnableSettleTime)
            {
                state_ = MotorFsmState::Enabled;
                std::cout << "[MotorFsm] all motors enabled → Enabled" << std::endl;
            }
        }
        else if (elapsed > std::chrono::milliseconds(2000))
        {
            // 2 秒还没全部使能 → 故障
            std::cerr << "[MotorFsm] Enable timeout (2s), not all motors enabled → Fault" << std::endl;
            state_ = MotorFsmState::Fault;
        }
        break;

    case MotorFsmState::Enabled:
        // 维持状态（不发任何指令；SetCommand 由 RL 层调用）
        break;

    case MotorFsmState::Disabling:
        if (AllMotorsHaveBit(0x01, false))
        {
            state_ = MotorFsmState::Disabled;
            std::cout << "[MotorFsm] all motors disabled → Disabled" << std::endl;
        }
        else if (elapsed > std::chrono::milliseconds(2000))
        {
            std::cerr << "[MotorFsm] Disable timeout → Fault" << std::endl;
            state_ = MotorFsmState::Fault;
        }
        break;

    case MotorFsmState::Zeroing:
        if (elapsed >= kZeroSettleTime)
        {
            // 设零位完成 → 重新发一帧使能保持使能态
            SendCommandFrames(0xFC);
            state_ = MotorFsmState::Enabled;
            std::cout << "[MotorFsm] Zero done → Enabled" << std::endl;
        }
        break;

    case MotorFsmState::Disabled:
    case MotorFsmState::Fault:
        // 静默状态
        break;
    }
}

void MotorFsm::SendCommandFrames(uint8_t cmd_byte)
{
    // 手册 §3.2.2：一帧 CAN-FD 最多携带 6 个电机的指令，
    // 槽位隐含电机索引（Byte2-11 → 第1槽，Byte12-21 → 第2槽，……Byte52-61 → 第6槽）。
    // 所以 ancnW1 的 16 个电机要按每 6 个一组切分成若干帧发送。
    //
    // ⚠️ 架构待厂商确认（见 README §待厂商确认的问题 Q2）：
    //   单帧只有 6 个槽位，且帧内没有任何字段标识"这是第几组"。
    //   若 16 个电机挂在同一条 CAN 总线上，则第 2、3 帧会因字节布局相同
    //   而被同一批电机（槽位对应的那 6 个）重复接收 —— 电机 7~16 收不到指令。
    //   届时必须二选一：
    //     ① 拆成多条 CAN 总线（如 6+6+4，每条总线独立 socket）
    //     ② 由厂商提供 CANID 重映射机制，让 16 个电机分时占用 1~6 槽位
    //   当前实现按"每组 6 个、MSGID 递增"发送，语义正确，
    //   但实际能否寻址到电机 7~16 取决于上述答复。

    constexpr size_t kSlotsPerFrame = 6;

    for (size_t base = 0; base < motor_count_; base += kSlotsPerFrame)
    {
        const size_t slots = std::min(kSlotsPerFrame, motor_count_ - base);
        std::array<uint8_t, CanInterface::kMaxFrameLen> frame{};

        // Byte0-1：MSGID（递增，便于从站回显时区分帧序）
        const uint16_t msgid = static_cast<uint16_t>(++special_msgid_);
        frame[0] = static_cast<uint8_t>(msgid & 0xFF);
        frame[1] = static_cast<uint8_t>((msgid >> 8) & 0xFF);

        // 6 个槽位：Byte2~Byte10 = 0xFF，Byte11 = 指令码
        // 手册 §3.3.1.1（使能 0xFC）/ §3.3.1.2（失能 0xFD）/ §3.3.1.4（设零位 0xEF）
        for (size_t slot = 0; slot < kSlotsPerFrame; ++slot)
        {
            const size_t b = 2 + slot * 10;
            for (int i = 0; i < 9; ++i)
            {
                frame[b + i] = 0xFF;
            }
            frame[b + 9] = cmd_byte;
        }

        // 手册 §3.2.2 注1：下发字节数可按最大槽位数截短以节省带宽
        //   1槽→12B, 2槽→24B, 3槽→32B, 4槽→48B, 5/6槽→64B
        size_t frame_len;
        switch (slots)
        {
            case 1:  frame_len = 12; break;
            case 2:  frame_len = 24; break;
            case 3:  frame_len = 32; break;
            case 4:  frame_len = 48; break;
            default: frame_len = 64; break;  // 5 或 6
        }

        // 主站 CANID = 0x00（手册 §3.2.1）
        can_->Send(0x00, frame.data(), frame_len);
    }
}

/**
 * @brief 检查"所有电机"的状态位是否都已达到期望值
 *
 * @param bit_mask 要检查的掩码，例如 0x01 检查使能位
 * @param set      true=期望该位为 1；false=期望该位为 0
 * @return true=全部电机都满足期望；false=至少有一个未满足
 *
 * 典型用法：
 *   AllMotorsHaveBit(0x01, true)  → 判断"是否全部使能成功"（Enabling → Enabled）
 *   AllMotorsHaveBit(0x01, false) → 判断"是否全部失能成功"（Disabling → Disabled）
 *
 * @note **取不到反馈的电机按"未满足"处理**（continue 而不是计数），
 *       这样在部分电机尚未上电或尚未回复时不会误判为已就绪。
 *       这是保守策略：宁可多等几个周期，也不能在电机未真正使能时就放行。
 *
 * @note 为什么这里不匹配 Count？
 *       简单地用 matched == motor_count_ 会在 motor_count_ == 0 时
 *       得到 0 == 0 → true，即"没有任何电机也算全部满足"，
 *       进而让状态机在当前完全空转的情况下错误地推进到 Enabled。
 *       下面显式排除这种空集情况。
 */
bool MotorFsm::AllMotorsHaveBit(uint8_t bit_mask, bool set) const
{
    // 边界防护：没有可管理的电机时，绝不应判定为"全部满足"
    if (motor_count_ == 0)
    {
        return false;
    }

    int matched = 0;
    for (size_t i = 0; i < motor_count_; ++i)
    {
        uint8_t mid = motor_ids_[i];
        std::array<uint8_t, CanInterface::kFeedbackLen> raw{};
        if (!can_->GetLatestFeedback(mid, &raw))
        {
            continue;  // 未收到反馈，按未满足处理（保守）
        }

        MotorFeedback fb;
        if (!driver_->ParseFeedback(raw.data(), raw.size(), &fb))
        {
            continue;  // 解析失败同样按未满足处理
        }

        bool has_bit = (fb.status & bit_mask) != 0;
        if (has_bit == set)
        {
            ++matched;
        }
    }
    return matched == static_cast<int>(motor_count_);
}

/**
 * @brief 记录最近一次故障（供上位机/日志事后追溯）
 *
 * @param motor_id 出故障的电机编号
 * @param type     故障类型
 *
 * @note 这里**加了 fault_mtx_ 锁**，与 State() 无锁形成对比：
 *       写入发生在控制线程（Tick 检测到故障时），
 *       而读取（LastFault）常被日志线程或 TCP 桥接线程调用，
 *       两者不在同一线程，因此必须互斥。
 *
 * @note 时间戳用**单调时钟**（steady_clock）而不是系统时钟：
 *       单调时钟不受 NTP 校时影响，用于计算"距今多久"永远可靠。
 */
void MotorFsm::RecordFault(uint8_t motor_id, MotorFault type)
{
    std::lock_guard<std::mutex> lock(fault_mtx_);
    last_fault_.motor_id = motor_id;
    last_fault_.type = type;
    last_fault_.timestamp_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

/**
 * @brief 查询最近一次故障记录（线程安全）
 * @return 故障记录结构体（含电机号、类型、时间戳）
 *
 * @note 返回的是**拷贝**而非引用，调用方可以安心持有，
 *       不会因为后续新的故障写入而看到内容突变。
 */
MotorFaultRecord MotorFsm::LastFault() const
{
    std::lock_guard<std::mutex> lock(fault_mtx_);
    return last_fault_;
}

}  // namespace ancn_w1
