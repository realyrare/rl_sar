/*
 * Copyright (c) 2024-2025 ancn
 * SPDX-License-Identifier: Apache-2.0
 *
 * ancnW1 电机协议封装实现
 * ----------------------------------------
 * 核心：线性映射 + 帧组装 + 反馈解析
 *
 * 映射公式（手册 §3.3.1.3）：
 *   对称（位置/速度/力矩）：uint16 = (value - min) / (max - min) * 65535
 *     0x8000 = 32768 → 32768/65535 ≈ 0.5 → (max + min)/2 = 0（对称时）
 *   非对称（Kp/Kd）：uint16 = (value / max) * 65535
 *     0x0000 = 0；0xFFFF = max
 *
 * 帧布局（手册 §3.2.2）：
 *   Byte0-1:   MSGID（小端：低字节在前）
 *   Byte2-11:  电机 1 指令
 *   Byte12-21: 电机 2 指令
 *   Byte22-31: 电机 3 指令
 *   Byte32-41: 电机 4 指令
 *   Byte42-51: 电机 5 指令
 *   Byte52-61: 电机 6 指令
 *   Byte62-63: 保留
 */

#include "ancnW1_motor_driver.hpp"

#include <algorithm>
#include <cstring>

namespace ancn_w1
{

MotorDriver::MotorDriver()
{
    // 默认所有电机用通用阈值（手册 §3.3.4）
    MotorTypeConfig default_cfg;
    default_cfg.p_min = -12.5f;
    default_cfg.p_max = 12.5f;
    default_cfg.v_min = -65.0f;
    default_cfg.v_max = 65.0f;
    default_cfg.kp_max = 500.0f;
    default_cfg.kd_max = 5.0f;
    default_cfg.t_min = -30.0f;
    default_cfg.t_max = 30.0f;
    for (size_t i = 0; i <= CanInterface::kMaxMotorId; ++i)
    {
        types_[i] = default_cfg;
    }
}

void MotorDriver::SetMotorType(uint8_t motor_id, const MotorTypeConfig &cfg)
{
    if (motor_id >= 1 && motor_id <= CanInterface::kMaxMotorId)
    {
        types_[motor_id] = cfg;
    }
}

void MotorDriver::SetAllMotorTypes(const MotorTypeConfig &cfg)
{
    for (size_t i = 1; i <= CanInterface::kMaxMotorId; ++i)
    {
        types_[i] = cfg;
    }
}

const MotorTypeConfig &MotorDriver::GetMotorType(uint8_t motor_id) const
{
    return types_[motor_id];
}

// ============================================================================
// 线性映射函数（手册 §3.3.1.3 下发 / §3.3.2 反馈）
//
// 电机协议里所有物理量都用 uint16 传输，区别只在于"0 点"落在哪里：
//
//   ┌────────────┬──────────────────┬────────┬─────────────────────┐
//   │ 映射方式   │ 用于             │ 0 点   │ 取值范围            │
//   ├────────────┼──────────────────┼────────┼─────────────────────┤
//   │ 对称映射   │ 位置/速度/力矩   │ 0x8000 │ MIN ~ MAX（含负数） │
//   │ 非对称映射 │ 刚度 Kp/阻尼 Kd  │ 0x0000 │ 0 ~ MAX（仅非负）   │
//   └────────────┴──────────────────┴────────┴─────────────────────┘
//
// 为什么 0x8000 恰好代表物理量 0？
//   对称区间的 min = -max（例如 P_MIN=-12.5、P_MAX=+12.5），代入公式：
//      raw(0) = (0 - (-12.5)) / (12.5 - (-12.5)) × 65535
//             = 12.5 / 25 × 65535 = 32767.5 → 四舍五入成 32768 = 0x8000 ✓
//   所以直接用一个线性公式即可，不需要为 0 点写任何特判。
// ============================================================================

/**
 * @brief 物理量 → uint16（对称映射，用于位置 / 速度 / 力矩）
 *
 * 公式： raw = round( (val - min) / (max - min) × 65535 )
 *
 * @param val 物理量：位置 rad / 速度 rad/s / 力矩 N·m
 * @param min 该型号的负向阈值，例如 P_MIN = -12.5
 * @param max 该型号的正向阈值，例如 P_MAX = +12.5
 * @return 0~65535 的编码值，其中 0x8000 代表物理量 0
 *
 * @note val 超出 [min,max] 时会被 clamp 到边界。
 *       真正的限幅发生在 ComputeOutput 侧（rl_sdk.cpp 的 torque_limits），
 *       这里的 clamp 只是兜底，防止 uint16 溢出后出现"数值回绕"这种灾难性错误。
 * @note max<=min 属配置错误（例如 yaml 填反），这里返回 0x8000（物理量 0）
 *       而不是除零崩溃 ——真机宁可不动也不能崩。
 */
uint16_t MotorDriver::ToUint16Symmetric(float val, float min, float max)
{
    if (max <= min)
    {
        return 0x8000;  // 退化区间 → 返回物理量 0，避免除零
    }
    float ratio = (val - min) / (max - min);   // 归一化到 [0,1]
    ratio = std::clamp(ratio, 0.0f, 1.0f);     // 超限截断，防 uint16 回绕
    return static_cast<uint16_t>(ratio * 65535.0f + 0.5f);  // +0.5 实现四舍五入
}

/**
 * @brief uint16 → 物理量（对称反映射，用于解析电机回传的位置/速度/力矩）
 *
 * 公式： val = min + (raw / 65535) × (max - min)
 *
 * @param raw 电机回传的 16 位原值，0x8000 表示物理量 0
 * @param min 该型号负向阈值
 * @param max 该型号正向阈值
 * @return 解码后的物理量（rad / rad/s / N·m）
 *
 * @note raw 类型是 uint16，天然落在 [0,65535]，因此这里无需再做 clamp。
 */
float MotorDriver::FromUint16Symmetric(uint16_t raw, float min, float max)
{
    return min + (static_cast<float>(raw) / 65535.0f) * (max - min);
}

/**
 * @brief 物理量 → uint16（非对称映射，用于刚度 Kp 与阻尼 Kd）
 *
 * 公式： raw = round( val / max × 65535 )
 *
 * @param val Kp 或 Kd 值，必须 ≥ 0（负数会被 clamp 到 0）
 * @param max 上限，取自手册 §3.3.4：KP_MAX 统一为 500，KD_MAX 多为 5
 *            （P85MAX-S / P100MAX-S 的 KD_MAX 是 50）
 * @return 0~65535，其中 0x0000 代表 0
 *
 * @note 与对称映射的区别：Kp/Kd 没有负值，所以 0 点就是 0x0000，
 *       65535 的全部分辨率都给了正区间 —— 精度约为对称映射的两倍。
 */
uint16_t MotorDriver::ToUint16Asymmetric(float val, float max)
{
    if (max <= 0.0f)
    {
        return 0;   // 配置错误 → 返回 0（相当于无刚度/无阻尼）
    }
    float ratio = val / max;
    ratio = std::clamp(ratio, 0.0f, 1.0f);
    return static_cast<uint16_t>(ratio * 65535.0f + 0.5f);
}

/**
 * @brief uint16 → 物理量（非对称反映射，用于解析 Kp/Kd）
 * @param raw  16 位原值，0x0000 表示 0
 * @param max  上限（KP_MAX 或 KD_MAX）
 * @return 0 ~ max 之间的物理量
 */
float MotorDriver::FromUint16Asymmetric(uint16_t raw, float max)
{
    return (static_cast<float>(raw) / 65535.0f) * max;
}

void MotorDriver::BuildEnableFrame(std::array<uint8_t, 64> *frame) const
{
    // 手册 §3.3.1.1：Byte0-1 = MSGID 0x0001（小端：01 00）
    // Byte2-61 = 6 个槽位全填 FF FF FF FF FF FF FF FF FF FC
    frame->fill(0xFF);
    frame->at(0) = 0x01;
    frame->at(1) = 0x00;
    // 6 个槽位的 Byte11（每槽第 10 字节 = 起始 Byte + 9）填 0xFC
    for (int i = 0; i < 6; ++i)
    {
        frame->at(2 + i * 10 + 9) = 0xFC;
    }
}

void MotorDriver::BuildDisableFrame(std::array<uint8_t, 64> *frame) const
{
    // 手册 §3.3.1.2：Byte11 = 0xFD，其余 0xFF
    BuildEnableFrame(frame);  // 复用结构
    for (int i = 0; i < 6; ++i)
    {
        frame->at(2 + i * 10 + 9) = 0xFD;
    }
}

void MotorDriver::BuildZeroFrame(std::array<uint8_t, 64> *frame) const
{
    // 手册 §3.3.1.4：Byte11 = 0xEF，其余 0xFF
    BuildEnableFrame(frame);
    for (int i = 0; i < 6; ++i)
    {
        frame->at(2 + i * 10 + 9) = 0xEF;
    }
}

void MotorDriver::BuildRunFrame(
    const std::array<uint8_t, 6> &motor_ids,
    const std::array<MotorRunCommand, 6> &cmds,
    uint16_t msgid,
    std::array<uint8_t, 64> *frame) const
{
    frame->fill(0);

    // MSGID（Byte0-1，小端）
    frame->at(0) = static_cast<uint8_t>(msgid & 0xFF);
    frame->at(1) = static_cast<uint8_t>((msgid >> 8) & 0xFF);

    // 6 个槽位
    for (int slot = 0; slot < 6; ++slot)
    {
        size_t base = 2 + slot * 10;
        uint8_t mid = motor_ids[slot];

        // 槽位未使用（motor_ids[slot] = 0）：手册允许填 0 或维持上次值
        // 这里填 0xFF（类似使能指令的 FF FF ... 但 Byte11 = 0 不是合法指令码）
        // 实际：手册 §3.2.2 说"其他给 0 或者维持上次下发的数据"
        // 为安全，填 0（电机将解释为"零位零速零刚度零阻尼零力矩" + 0x0000 = 非对称零）
        // 但电机读到的 Kp=0、Kd=0 可能进入纯阻尼模式
        // 稳妥做法：若该槽未用，对应 cmds 填零 + 不发送该帧
        // 这里选择仍然发送（手册允许"按需截短"但我们的封装统一 64 字节）
        if (mid == 0 || mid > CanInterface::kMaxMotorId)
        {
            continue;  // 该槽位全填 0
        }

        const MotorTypeConfig &cfg = types_[mid];
        const MotorRunCommand &c = cmds[slot];

        // 位置（Byte2-3，对称映射）
        uint16_t p_raw = ToUint16Symmetric(c.q, cfg.p_min, cfg.p_max);
        frame->at(base + 0) = static_cast<uint8_t>(p_raw & 0xFF);
        frame->at(base + 1) = static_cast<uint8_t>((p_raw >> 8) & 0xFF);

        // 速度（Byte4-5，对称映射）
        uint16_t v_raw = ToUint16Symmetric(c.dq, cfg.v_min, cfg.v_max);
        frame->at(base + 2) = static_cast<uint8_t>(v_raw & 0xFF);
        frame->at(base + 3) = static_cast<uint8_t>((v_raw >> 8) & 0xFF);

        // Kp（Byte6-7，非对称映射）
        uint16_t kp_raw = ToUint16Asymmetric(c.kp, cfg.kp_max);
        frame->at(base + 4) = static_cast<uint8_t>(kp_raw & 0xFF);
        frame->at(base + 5) = static_cast<uint8_t>((kp_raw >> 8) & 0xFF);

        // Kd（Byte8-9，非对称映射）
        uint16_t kd_raw = ToUint16Asymmetric(c.kd, cfg.kd_max);
        frame->at(base + 6) = static_cast<uint8_t>(kd_raw & 0xFF);
        frame->at(base + 7) = static_cast<uint8_t>((kd_raw >> 8) & 0xFF);

        // 力矩（Byte10-11，对称映射）
        uint16_t t_raw = ToUint16Symmetric(c.tau, cfg.t_min, cfg.t_max);
        frame->at(base + 8) = static_cast<uint8_t>(t_raw & 0xFF);
        frame->at(base + 9) = static_cast<uint8_t>((t_raw >> 8) & 0xFF);
    }
}

bool MotorDriver::ParseFeedback(const uint8_t *data, size_t len, MotorFeedback *fb) const
{
    if (data == nullptr || len < 12 || fb == nullptr)
    {
        return false;
    }

    // MSGID（Byte0-1，小端）
    fb->msgid = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);

    // 位置（Byte2-3，对称映射；注意：反馈帧不带 motor_id，所以用哪个阈值取决于上层）
    // 这里用通用阈值（手册 §3.3.4 全型号 P_MIN/P_MAX 都是 ±12.5）
    uint16_t p_raw = static_cast<uint16_t>(data[2]) | (static_cast<uint16_t>(data[3]) << 8);
    fb->q = FromUint16Symmetric(p_raw, -12.5f, 12.5f);

    // 速度（Byte4-5）
    uint16_t v_raw = static_cast<uint16_t>(data[4]) | (static_cast<uint16_t>(data[5]) << 8);
    // 速度阈值因型号不同（P: ±65; W110-S/W150-S: ±300; W190-S: ±200）
    // 用保守的 ±65（P 系列）；解析后由调用方根据电机 ID 重新计算或标记
    fb->dq = FromUint16Symmetric(v_raw, -65.0f, 65.0f);

    // 力矩（Byte6-7）
    uint16_t t_raw = static_cast<uint16_t>(data[6]) | (static_cast<uint16_t>(data[7]) << 8);
    // 力矩阈值因型号不同；用保守的 ±30（P55 / W110-S / W150-S）
    fb->tau = FromUint16Symmetric(t_raw, -30.0f, 30.0f);

    // 温度（Byte8）
    fb->temp_raw = data[8];

    // 状态码（Byte9）
    fb->status = data[9];

    // 电压（Byte10）
    fb->voltage = data[10];

    // Byte11 预留
    return true;
}

}  // namespace ancn_w1
