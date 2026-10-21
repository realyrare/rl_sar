/*
 * Copyright (c) 2024-2025 ancn
 * SPDX-License-Identifier: Apache-2.0
 *
 * ancnW1 电机协议封装
 * ----------------------------------------
 * 在 CAN-FD 字节流之上封装：
 *   - 线性映射函数（float ↔ uint16，按手册 §3.3.4 阈值）
 *   - 指令帧组装（使能/失能/设零位/运行）
 *   - 反馈帧解析（位置/速度/扭矩/温度/状态码/电压）
 *
 * 设计原则：
 *   - 所有阈值都可配置（BOM 确认后填入）
 *   - 默认值取手册 §3.3.4 通用表
 *   - 帧组装与电机类型解耦：每帧 6 个槽，每个槽用对应 motor_id 的阈值
 */

#ifndef ANCNW1_MOTOR_DRIVER_HPP
#define ANCNW1_MOTOR_DRIVER_HPP

#include <array>
#include <cstdint>

#include "ancnW1_can_interface.hpp"

namespace ancn_w1
{

/// 单个电机的协议阈值（手册 §3.3.4）
struct MotorTypeConfig
{
    float p_min = -12.5f;   ///< 位置最小值（rad），手册通用值
    float p_max = 12.5f;    ///< 位置最大值（rad）
    float v_min = -65.0f;   ///< 速度最小值（rad/s），P 系列默认
    float v_max = 65.0f;    ///< 速度最大值（rad/s）
    float kp_max = 500.0f;  ///< 刚度最大值（手册全部型号统一 500）
    float kd_max = 5.0f;    ///< 阻尼最大值（P 系列默认 5，P85MAX-S / P100MAX-S 是 50）
    float t_min = -30.0f;   ///< 力矩最小值（N·m），P55 默认；按 BOM 修改
    float t_max = 30.0f;    ///< 力矩最大值（N·m）
};

/// 解析后的电机反馈（手册 §3.3.2）
struct MotorFeedback
{
    float q = 0.0f;         ///< 输出端位置（rad）
    float dq = 0.0f;        ///< 输出端速度（rad/s）
    float tau = 0.0f;       ///< 输出端扭矩（N·m）
    uint8_t temp_raw = 0;   ///< 温度原始字节（0~255，实际温度 = byte - 40 ℃，手册 §3.3.2）
    uint8_t status = 0;     ///< 状态码（bit0=使能, bit1=过压, bit2=过流, bit3=过温, bit4=过速, bit5=编码器, bit6=欠压, bit7=预留）
    uint8_t voltage = 0;    ///< 母线电压（0~255 V）
    uint16_t msgid = 0;     ///< MSGID（回显主机发送，用于多帧区分）

    // 便捷状态位读取
    bool Enabled() const { return (status & 0x01) != 0; }
    bool Overvoltage() const { return (status & 0x02) != 0; }
    bool Overcurrent() const { return (status & 0x04) != 0; }
    bool Overtemperature() const { return (status & 0x08) != 0; }
    bool Overspeed() const { return (status & 0x10) != 0; }
    bool EncoderFault() const { return (status & 0x20) != 0; }
    bool Undervoltage() const { return (status & 0x40) != 0; }

    /// 实际温度（℃），手册公式：T = byte - 40
    float TemperatureC() const { return static_cast<float>(temp_raw) - 40.0f; }
};

/// 单个电机的运行指令（手册 §3.3.1.3）
struct MotorRunCommand
{
    float q = 0.0f;        ///< 目标位置（rad）；纯力矩模式置 0
    float dq = 0.0f;       ///< 目标速度（rad/s）；纯位置模式置 0
    float kp = 0.0f;       ///< 刚度（0~KP_MAX）；纯力矩模式置 0
    float kd = 0.0f;       ///< 阻尼（0~KD_MAX）；纯力矩模式置 0
    float tau = 0.0f;      ///< 前馈力矩（N·m）；可正可负
};

/**
 * @brief 电机协议封装
 *
 * 核心职责：
 *   1. 维护 16 个电机的型号阈值表（SetMotorType / SetAllMotorTypes）
 *   2. 把 6 个电机的运行指令打包成 64 字节 CAN-FD 帧
 *   3. 把 12 字节反馈解析为 MotorFeedback 结构
 *   4. 提供 float ↔ uint16 的对称线性映射（手册 §3.3.1.3）
 *
 * 关键映射规则（手册 §3.3.1.3）：
 *   - 位置/速度/力矩：0x8000 对应 0，对称映射
 *   - 刚度/阻尼：0x0000 对应 0，0xFFFF 对应 MAX
 */
class MotorDriver
{
public:
    MotorDriver();

    /**
     * @brief 设置单个电机的型号阈值
     * @param motor_id 电机 ID（1~16）
     * @param cfg 该型号的阈值表
     *
     * ancnW1 默认（待 BOM 确认后调整）：
     *   - 腿（ID 1~12）：P65MAX-S 或 P85MAX-S
     *   - 轮（ID 13~16）：W150-S 或 W190-S
     */
    void SetMotorType(uint8_t motor_id, const MotorTypeConfig &cfg);

    /// 全部电机同一型号（用于早期调试/缺 BOM 时）
    void SetAllMotorTypes(const MotorTypeConfig &cfg);

    /// 获取电机阈值（用于诊断/日志）
    const MotorTypeConfig &GetMotorType(uint8_t motor_id) const;

    /**
     * @brief 组装"使能"指令帧（手册 §3.3.1.1）
     * @param frame 64 字节输出
     * @return true=成功
     *
     * 帧内容：MSGID=0x0001，6 个槽位全填 FF FF FF FF FF FF FF FF FF FC
     * 用途：上电后先发一帧让所有电机进入使能态
     */
    void BuildEnableFrame(std::array<uint8_t, 64> *frame) const;

    /// 组装"失能"指令帧（手册 §3.3.1.2，Byte11=0xFD）
    void BuildDisableFrame(std::array<uint8_t, 64> *frame) const;

    /// 组装"设置机械零位"指令帧（手册 §3.3.1.4，Byte11=0xEF）
    void BuildZeroFrame(std::array<uint8_t, 64> *frame) const;

    /**
     * @brief 组装"运行"指令帧（手册 §3.3.1.3）
     * @param motor_ids 6 个电机 ID（用于确定用哪个电机的阈值做映射）
     * @param cmds 6 个电机的运行指令
     * @param msgid 帧 ID（推荐 1~65535 递增，便于在反馈中识别帧对应）
     * @param[out] frame 64 字节输出
     *
     * 注意：
     *   - motor_ids 和 cmds 长度必须为 6；不在 1~16 的槽位应填 0xFF 指令码（手册 §3.2.2）
     *   - ancnW1 需要 3 帧覆盖 16 个电机：电机 1-6, 7-12, 13-16（最后帧第 5、6 槽填空）
     */
    void BuildRunFrame(
        const std::array<uint8_t, 6> &motor_ids,
        const std::array<MotorRunCommand, 6> &cmds,
        uint16_t msgid,
        std::array<uint8_t, 64> *frame) const;

    /**
     * @brief 解析反馈帧（手册 §3.3.2）
     * @param data 12 字节输入
     * @param len 字节数（必须 ≥12）
     * @param[out] fb 解析后的反馈
     * @return true=解析成功
     *
     * 注意：
     *   - 反馈帧的 CAN ID 不参与解析（电机 ID 由电机自己决定）；
     *     但电机只在自己分配到的槽位回应，不会越界
     */
    bool ParseFeedback(const uint8_t *data, size_t len, MotorFeedback *fb) const;

    /**
     * @brief float → uint16 对称映射（位置/速度/力矩用，0x8000 = 0）
     */
    static uint16_t ToUint16Symmetric(float val, float min, float max);

    /**
     * @brief uint16 → float 对称反映射
     */
    static float FromUint16Symmetric(uint16_t raw, float min, float max);

    /**
     * @brief float → uint16 非对称映射（Kp/Kd 用，0x0000 = 0，0xFFFF = MAX）
     */
    static uint16_t ToUint16Asymmetric(float val, float max);

    /// uint16 → float 非对称反映射
    static float FromUint16Asymmetric(uint16_t raw, float max);

private:
    /// 每电机一个阈值表（索引 0 不用）
    MotorTypeConfig types_[CanInterface::kMaxMotorId + 1];
};

}  // namespace ancn_w1

#endif  // ANCNW1_MOTOR_DRIVER_HPP
