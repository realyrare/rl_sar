/*
 * Copyright (c) 2024-2025 ancn
 * SPDX-License-Identifier: Apache-2.0
 *
 * ancnW1 真机 RL 节点
 * ----------------------------------------
 * 继承 RL 基类，实现 GetState / SetCommand / RunModel。
 *
 * 硬件：
 *   - 16 个智身新创 CAN-FD 关节电机（12 腿 + 4 轮）
 *   - IMU（独立 SPI/I2C 接口，本文件留 TODO）
 *   - 1 路 socketcan（连接电机控制板）
 *
 * 待 BOM 确认（厂商）：
 *   - 12 个腿关节的具体型号 → torque_limits / KD_MAX
 *   - 4 个轮毂的具体型号 → V_MAX / T_MAX
 *   - 电机 CAN ID 分配方式（出厂默认 0x03，需重设为 1~16）
 *   - 反馈帧 CAN ID（每电机独立 / 共用）
 */

#ifndef RL_REAL_ANCNW1_HPP
#define RL_REAL_ANCNW1_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm_ancnW1.hpp"

#include "ancnW1_can_interface.hpp"
#include "ancnW1_motor_driver.hpp"
#include "ancnW1_motor_fsm.hpp"

#include <array>
#include <atomic>
#include <memory>
#include <mutex>

namespace ancn_w1
{

/**
 * @brief ancnW1 真机 RL 节点
 *
 * 启动流程：
 *   1. 构造 → 读 yaml → 加载 FSM → 打开 CAN 接口 → 配置电机类型表 → 启动接收线程
 *   2. 等 100ms 让反馈填充 → 调 motor_fsm_.Enable() 异步发起使能
 *   3. 等 motor_fsm_.State() == Enabled（用户在键盘按 'M' 也可触发）
 *   4. 进入主循环：
 *        loop_keyboard (50Hz)  → 处理键盘输入
 *        loop_control (200Hz)  → GetState + MotorFsm::Tick + SetCommand
 *        loop_rl      (50Hz)   → Forward + 决策 + ComputeOutput
 *
 * 线程模型：
 *   - 主线程：3 个 LoopFunc 协作
 *   - CanInterface 后台线程：持续接收反馈
 *   - IMU（TODO）：另开线程 / SPI 中断
 */
class RL_Real_AncnW1 : public RL
{
public:
    RL_Real_AncnW1(int argc, char **argv);
    ~RL_Real_AncnW1();

    /// 启动所有循环（构造完成后调用）
    void Run();

private:
    // ===== RL 接口实现 =====
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;

    // ===== 控制循环 =====
    void RobotControl();
    void RunModel();

    // ===== IMU（待 SPI/I2C 驱动确认后实现） =====
    /// 初始化 IMU（设置 SPI/I2C 总线、配置采样率）
    bool InitImu();
    /// 从 IMU 读最新一帧并写入 imu_quat_/imu_gyro_/imu_accel_（线程安全）
    void UpdateImuData();

    // ===== 电机命令组装辅助 =====
    /// 把 RobotCommand（policy 顺序）按 joint_mapping 重排，并按 leg/wheel 转换电机工作模式
    /// 然后拆成 3 帧（电机 1-6, 7-12, 13-16）
    void AssembleRunFrames(
        const RobotCommand<float> *command,
        std::array<std::array<uint8_t, CanInterface::kMaxFrameLen>, 3> *frames,
        uint16_t msgid);

    /// 配置电机类型表（BOM 确认后填具体型号；目前全部用通用阈值）
    void ConfigureMotorTypes();

    // ===== 硬件 =====
    CanInterface can_;             ///< CAN-FD socketcan 接口
    MotorDriver driver_;           ///< 电机协议封装
    MotorFsm motor_fsm_;           ///< 电机状态机（使能/失能/故障）

    // ===== 控制循环 =====
    std::shared_ptr<LoopFunc> loop_keyboard_;
    std::shared_ptr<LoopFunc> loop_control_;
    std::shared_ptr<LoopFunc> loop_rl_;

    // ===== IMU 缓存 =====
    std::mutex imu_mtx_;
    float imu_quat_[4] = {1.0f, 0.0f, 0.0f, 0.0f};   ///< w, x, y, z
    float imu_gyro_[3] = {0.0f, 0.0f, 0.0f};          ///< rad/s
    float imu_accel_[3] = {0.0f, 0.0f, 0.0f};         ///< m/s^2

    // ===== 帧 ID 计数器 =====
    uint16_t msgid_counter_ = 0;

    // ===== 启动状态 =====
    bool can_opened_ = false;
    bool imu_initialized_ = false;
};

}  // namespace ancn_w1

#endif  // RL_REAL_ANCNW1_HPP
