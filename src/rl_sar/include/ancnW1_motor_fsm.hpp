/*
 * Copyright (c) 2024-2025 ancn
 * SPDX-License-Identifier: Apache-2.0
 *
 * ancnW1 电机初始化状态机
 * ----------------------------------------
 * 封装"使能 → 运行 → 失能 → 设零位"的状态转换逻辑。
 *
 * 状态转换图：
 *   Disabled ──Enable()──> Enabling ──all enabled──> Enabled
 *   Enabled  ──Zero()───> Zeroing   ──done─────────> Enabled
 *   Enabled  ──Disable()─> Disabling ──all disabled─> Disabled
 *   *        ──fault───> Fault        ──ClearFault()──> Disabled
 *
 * 安全设计：
 *   - 使能前必须收到每个电机的 bit0=1 才进 Enabled
 *   - 任何电机出现 bit1~bit6 任一故障 → 立即进 Fault
 *   - 通信超时（>100ms 未收到反馈）→ 进 Fault
 *   - Fault 状态下不再发任何运行指令（防止故障电机被持续驱动）
 *
 * 依赖：
 *   - CanInterface（接收反馈、发送指令）
 *   - MotorDriver（组帧）
 */

#ifndef ANCNW1_MOTOR_FSM_HPP
#define ANCNW1_MOTOR_FSM_HPP

#include <array>
#include <cstdint>
#include <chrono>
#include <mutex>

#include "ancnW1_can_interface.hpp"
#include "ancnW1_motor_driver.hpp"

namespace ancn_w1
{

/// 电机状态机状态
enum class MotorFsmState
{
    Disabled,   ///< 初始/失能态：未发送使能指令，或全部电机 bit0=0
    Enabling,   ///< 已发出使能帧，等待 bit0=1
    Enabled,    ///< 所有电机使能，可以发运行指令
    Disabling,  ///< 已发出失能帧，等待 bit0=0
    Zeroing,    ///< 已发出设零位帧（手动流程，短暂状态）
    Fault,      ///< 检测到故障，停止一切输出
};

/// 故障类型（用于上报上层）
enum class MotorFault : uint8_t
{
    None = 0,
    Overvoltage = 0x02,
    Overcurrent = 0x04,
    Overtemperature = 0x08,
    Overspeed = 0x10,
    EncoderFault = 0x20,
    Undervoltage = 0x40,
    CommTimeout,  ///< 通信超时（非状态码 bit，来自应用层判断）
};

/// 故障记录（最近一次）
struct MotorFaultRecord
{
    uint8_t motor_id = 0;       ///< 故障电机 ID
    MotorFault type = MotorFault::None;  ///< 故障类型
    uint64_t timestamp_us = 0;   ///< 故障发生时刻（steady_clock 微秒）
};

/**
 * @brief 电机初始化状态机
 *
 * 用法（控制循环中每 dt 调用一次 Tick）：
 *   MotorFsm fsm(&can, &driver);
 *   fsm.SetMotorIds({1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16});  // 物理电机 ID 列表
 *   fsm.Start();  // 触发 Disabled → Enabling
 *
 *   while (running) {
 *       fsm.Tick();
 *       if (fsm.State() == MotorFsmState::Enabled) {
 *           // 安全发运行指令
 *       }
 *       if (fsm.State() == MotorFsmState::Fault) {
 *           // 报警，等待人工处理
 *       }
 *   }
 */
class MotorFsm
{
public:
    /// 通信超时阈值：超过此时间未收到反馈 → 报 CommTimeout 故障
    static constexpr std::chrono::milliseconds kCommTimeout{100};

    /// 使能后稳定等待时间（手册 §3.3.1.1：使能后电机进入阻抗控制模式）
    static constexpr std::chrono::milliseconds kEnableSettleTime{50};

    /// 设零位后稳定等待时间
    static constexpr std::chrono::milliseconds kZeroSettleTime{100};

    MotorFsm(CanInterface *can, MotorDriver *driver);

    /// 设置本机器人包含的电机 ID 列表（ancnW1 = [1..16]）
    void SetMotorIds(const std::array<uint8_t, CanInterface::kMaxMotorId> &ids);

    /// 当前状态（线程安全读）
    MotorFsmState State() const;

    /// 当前状态字符串（用于日志）
    static const char *StateName(MotorFsmState s);

    /**
     * @brief 请求使能（Disabled → Enabling）
     * @note 立即发送使能帧；Tick() 内根据反馈推进状态
     */
    void Enable();

    /// 请求失能（任何状态 → Disabling）
    void Disable();

    /// 请求设零位（仅在 Enabled 有效 → Zeroing）
    void Zero();

    /// 清除故障（Fault → Disabled）
    void ClearFault();

    /// 周期性调用：检查反馈、推进状态、检测故障
    /// @note 应在每个控制 dt 调用
    void Tick();

    /// 是否处于可以发运行指令的安全状态
    bool CanSendRunCommand() const { return State() == MotorFsmState::Enabled; }

    /// 获取最近一次故障
    MotorFaultRecord LastFault() const;

private:
    /// 发送使能/失能/设零位帧（3 帧覆盖 16 电机）
    void SendCommandFrames(uint8_t last_byte);

    /// 检查所有电机反馈，更新状态/故障
    void ProcessFeedback();

    /// 是否所有目标电机都处于指定 bit 状态
    bool AllMotorsHaveBit(uint8_t bit_mask, bool set) const;

    /// 记录一次故障
    void RecordFault(uint8_t motor_id, MotorFault type);

    CanInterface *can_;
    MotorDriver *driver_;

    /// 本机器人包含的电机 ID（ancnW1 = 1..16）
    std::array<uint8_t, CanInterface::kMaxMotorId> motor_ids_{};
    size_t motor_count_ = 0;

    /// 状态机当前状态
    MotorFsmState state_ = MotorFsmState::Disabled;

    /// 特殊指令（使能/失能/设零位）的 MSGID 计数器，每发一组帧递增一次
    /// 用途：多帧下发时让从站回显可区分帧序；不使用 MSGID 时也可填 0（手册 §3.2.2 注3）
    uint16_t special_msgid_ = 0;

    /// 状态进入时刻（用于超时判断）
    std::chrono::steady_clock::time_point state_enter_time_;

    /// 最近一次故障
    MotorFaultRecord last_fault_{};
    mutable std::mutex fault_mtx_;
};

}  // namespace ancn_w1

#endif  // ANCNW1_MOTOR_FSM_HPP
