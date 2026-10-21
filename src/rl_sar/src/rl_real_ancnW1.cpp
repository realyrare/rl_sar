/*
 * Copyright (c) 2024-2025 ancn
 * SPDX-License-Identifier: Apache-2.0
 *
 * ancnW1 真机节点实现
 * ----------------------------------------
 * 启动流程详见 include/rl_real_ancnW1.hpp 注释。
 *
 * 本文件是 motor_driver + motor_fsm + RL 的"装配层"：
 *   - 构造时建立 3 个模块的依赖关系
 *   - SetCommand() 把 RobotCommand 转换为 3 帧 CAN-FD
 *   - GetState() 把电机反馈填回 RobotState
 *
 * 关键设计决策：
 *   1. 电机 Kp/Kd 字段强制置 0（避免与 tau 里的 PD 项重复 → 等效刚度从 3*rl_kp 变回 2*rl_kp）
 *   2. 轮子电机用纯速度模式（dq 有值，q=0x8000 编码的 0）
 *   3. CAN ID 始终用 0x00（手册 §3.2.1 主站 CANID）
 *   4. msgid 单调递增，便于上层定位丢失/乱序的帧
 */

#include "rl_real_ancnW1.hpp"

#include <chrono>
#include <iostream>
#include <thread>

#include "logger.hpp"

namespace ancn_w1
{

// ============================================================================
// 构造 / 析构
// ============================================================================

RL_Real_AncnW1::RL_Real_AncnW1(int argc, char **argv)
    : motor_fsm_(&can_, &driver_)
{
    (void)argc;
    (void)argv;

    // 1. 读 yaml 配置
    this->robot_name = "ancnW1";
    this->ReadYaml(this->robot_name, "base.yaml");

    // 2. 加载状态机（fsm_ancnW1 已在 fsm_robot/fsm_ancnW1.hpp 通过 FSMManager 注册）
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[ancnW1] FSM not registered for "
                  << this->robot_name << std::endl;
    }

    // 3. RL 框架初始化
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    this->InitRL(this->robot_name);

    // 4. 配置电机类型表（BOM 确认后填具体型号阈值）
    ConfigureMotorTypes();

    // 5. 打开 CAN-FD 接口（默认 "can0"，可通过环境变量 CAN_IFACE 覆盖）
    const char *env_iface = std::getenv("CAN_IFACE");
    std::string can_iface = env_iface ? env_iface : CanInterface::kDefaultIfname;
    can_opened_ = can_.Open(can_iface);
    if (!can_opened_)
    {
        std::cout << LOGGER::ERROR << "[ancnW1] failed to open CAN interface '"
                  << can_iface << "', check `ip link` and root permission" << std::endl;
    }
    else
    {
        can_.StartReceiveThread();
        // 等待 100ms 让首批反馈填充
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 6. 初始化 IMU（TODO：等待 SPI/I2C 协议确认）
    imu_initialized_ = InitImu();
    if (!imu_initialized_)
    {
        std::cout << LOGGER::WARNING << "[ancnW1] IMU not initialized, using stub "
                  << "(q=identity, gyro=0, accel=0). OBSERVATIONS WILL BE WRONG."
                  << std::endl;
    }

    // 7. 启动电机使能序列（仅在 CAN 打开时）
    if (can_opened_)
    {
        motor_fsm_.Enable();
    }

    // 8. 创建控制循环
    this->loop_keyboard_ = std::make_shared<LoopFunc>(
        "loop_keyboard", 0.05, std::bind(&RL_Real_AncnW1::KeyboardInterface, this));
    this->loop_control_ = std::make_shared<LoopFunc>(
        "loop_control", this->params.Get<float>("dt"),
        std::bind(&RL_Real_AncnW1::RobotControl, this));
    this->loop_rl_ = std::make_shared<LoopFunc>(
        "loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"),
        std::bind(&RL_Real_AncnW1::RunModel, this));
}

RL_Real_AncnW1::~RL_Real_AncnW1()
{
    // 1. 停止循环
    if (loop_keyboard_) loop_keyboard_->shutdown();
    if (loop_control_) loop_control_->shutdown();
    if (loop_rl_) loop_rl_->shutdown();

    // 2. 失能所有电机（安全收尾）
    if (can_opened_ && motor_fsm_.State() == MotorFsmState::Enabled)
    {
        motor_fsm_.Disable();
        // 等待 100ms 让失能指令发出
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 3. 关闭 CAN 接口（自动停止接收线程）
    can_.Close();

    std::cout << LOGGER::INFO << "[ancnW1] shutdown complete" << std::endl;
}

void RL_Real_AncnW1::Run()
{
    if (!loop_keyboard_ || !loop_control_ || !loop_rl_)
    {
        std::cout << LOGGER::ERROR << "[ancnW1] loops not initialized" << std::endl;
        return;
    }
    loop_keyboard_->start();
    loop_control_->start();
    loop_rl_->start();
    float dt = this->params.Get<float>("dt");
    int dec = this->params.Get<int>("decimation");
    std::cout << LOGGER::INFO << "[ancnW1] running (keyboard=20Hz, control="
              << static_cast<int>(1.0f / dt) << "Hz, rl="
              << static_cast<int>(1.0f / (dt * dec)) << "Hz)" << std::endl;
}

// ============================================================================
// 配置电机类型表
// ============================================================================

void RL_Real_AncnW1::ConfigureMotorTypes()
{
    // TODO：BOM 确认后按关节型号填具体阈值
    //
    // 当前使用手册 §3.3.4 通用值（P 系列默认）：
    //   P_MAX=P_MIN=±12.5；V_MAX=V_MIN=±65；KP_MAX=500；KD_MAX=5；T_MAX=T_MIN=±30
    //
    // 若腿部用 P85MAX-S → T 改为 ±300, KD_MAX=50
    // 若轮子用 W150-S  → V 改为 ±300, T 改为 ±30
    // 若轮子用 W190-S  → V 改为 ±200, T 改为 ±50

    MotorTypeConfig default_cfg;
    default_cfg.p_min = -12.5f;
    default_cfg.p_max = 12.5f;
    default_cfg.v_min = -65.0f;
    default_cfg.v_max = 65.0f;
    default_cfg.kp_max = 500.0f;
    default_cfg.kd_max = 5.0f;
    default_cfg.t_min = -30.0f;
    default_cfg.t_max = 30.0f;

    driver_.SetAllMotorTypes(default_cfg);

    // 示例：BOM 确认后用以下方式单独设置
    // MotorTypeConfig leg_cfg = default_cfg;
    // leg_cfg.t_min = -300.0f; leg_cfg.t_max = 300.0f;  // P85MAX-S
    // leg_cfg.kd_max = 50.0f;
    // for (uint8_t id : {1,2,3,5,6,7,9,10,11,13,14,15}) {
    //     driver_.SetMotorType(id, leg_cfg);
    // }
    // MotorTypeConfig wheel_cfg = default_cfg;
    // wheel_cfg.v_min = -200.0f; wheel_cfg.v_max = 200.0f;  // W190-S
    // wheel_cfg.t_min = -50.0f; wheel_cfg.t_max = 50.0f;
    // for (uint8_t id : {4,8,12,16}) {
    //     driver_.SetMotorType(id, wheel_cfg);
    // }
}

// ============================================================================
// IMU（占位实现，待 SPI/I2C 协议确认后填具体驱动）
// ============================================================================

bool RL_Real_AncnW1::InitImu()
{
    // TODO：替换为真实 IMU 驱动
    //   - 候选：ICM-42688 / BMI088 / MPU6050（SPI）
    //   - 总线：RK3588 SPI0 或 I2C（视硬件决定）
    //   - 采样率：≥500Hz（与控制频率匹配）
    //   - 返回 true 表示成功
    return false;  // 占位：未实现
}

void RL_Real_AncnW1::UpdateImuData()
{
    if (imu_initialized_)
    {
        // TODO：从真实 IMU 读 1 帧，填到 imu_quat_/imu_gyro_/imu_accel_
        // std::lock_guard<std::mutex> lock(imu_mtx_);
        // imu_quat_[0] = ...;
        // imu_quat_[1] = ...;
        // ...
        return;
    }

    // 桩：保持初始值（identity 四元数 + 零角速度）
    // 警告：观测中的 ang_vel 和 gravity_vec 会全为 0，策略会跑飞！
    return;
}

// ============================================================================
// RL 接口实现
// ============================================================================

std::vector<float> RL_Real_AncnW1::Forward()
{
    // 与 go2 实现对齐（rl_real_go2.cpp:260）：
    //   1. model_mutex 保护模型热加载（try_to_lock 避免阻塞控制循环）
    //   2. 支持 history buffer（base.yaml 的 observations_history，ancnW1 为空 → 走 else 分支）
    //   3. clip_actions 裁剪（base.yaml 的 clip_actions_lower/upper）
    //
    // 注意：不要在这里直接 ComputeObservation() 后无条件 forward，
    //       否则模型重载时会与 Forward() 竞争导致崩溃。

    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // 模型正在重新初始化 → 返回上一周期的动作，避免阻塞控制循环
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() &&
        !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions,
                     this->params.Get<std::vector<float>>("clip_actions_lower"),
                     this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

/**
 * @brief 【RL 框架回调】把硬件状态填成策略期望的形式
 *
 * 由 RobotControl() 每个控制周期调用一次（dt=0.005s → 200Hz）。
 * 这是 rl_sdk 基类的纯虚函数，真机必须实现。
 *
 * 本函数只做两件事：
 *   ① 取 IMU 数据（姿态 + 角速度 + 加速度）
 *   ② 取 16 个电机的 CAN 反馈，并把"硬件物理顺序"重排成"策略顺序"
 *
 * ================== 索引换算（全模块最容易出错的地方）==================
 *
 * 系统里同时存在三种顺序，弄混会让机器人"换了另一条腿在动"：
 *
 *   ① policy 顺序 —— 策略/模型输入顺序，由 joint_names 定义，索引 0~15
 *        [fl_hip, fl_thigh, fl_calf, fr_hip, fr_thigh, fr_calf,
 *         hl_hip, hl_thigh, hl_calf, hr_hip, hr_thigh, hr_calf,  ← 先 12 个腿关节
 *         fl_wheel, fr_wheel, hl_wheel, hr_wheel]                ← 再 4 个轮子
 *
 *   ② physical 顺序 —— 硬件 / URDF / MJCF 顺序，按"每条腿"连续排列
 *        每条腿 4 个关节 (hip, thigh, calf, wheel)，所以轮子落在 3/7/11/15
 *
 *   ③ motor_id —— CAN 总线上电机的编号 = physical 索引 + 1，取值 1~16
 *
 *   换算关系（查表，base.yaml 提供）：
 *        physical_idx = joint_mapping[policy_idx]
 *        motor_id     = physical_idx + 1
 *
 *   实例（务必核对，轮子并不在最后）：
 *        policy 0  (fl_hip)   → physical 0  → motor 1
 *        policy 3  (fr_hip)   → physical 4  → motor 5
 *        policy 12 (fl_wheel) → physical 3  → motor 4     ← 注意不是 13
 *        policy 15 (hr_wheel) → physical 15 → motor 16
 *
 * @param[out] state 输出状态结构，各字段含义与单位：
 *        state->imu.quaternion[4]        姿态四元数 [w, x, y, z]，单位四元数（模长=1）
 *        state->imu.gyroscope[3]         机体系角速度 rad/s
 *        state->imu.accelerometer[3]     机体系加速度 m/s^2
 *        state->motor_state.q[16]        关节位置 rad       （下标为 policy 顺序）
 *        state->motor_state.dq[16]       关节速度 rad/s
 *        state->motor_state.tau_est[16]  关节力矩估计 N·m  （电机回传，非估算）
 *        注：motor_state.ddq / cur 反馈帧未提供，保持 0
 *
 * @note 某个电机本周期没取到反馈时，**保留上一次的值**而不是填 0。
 *       理由：16 个电机各自独立回帧，偶发丢一帧属正常；若填 0 会
 *       制造"关节瞬间跳到 0"的伪跳变，而策略观测包含 dof_pos/dof_vel，
 *       这种假跳变会让策略输出一个很大的纠错动作，真机可能直接弹起来。
 */
void RL_Real_AncnW1::GetState(RobotState<float> *state)
{
    if (state == nullptr)
    {
        return;
    }

    // ------------------------------------------------------------------
    // 步骤 1：IMU
    // imu_quat_/imu_gyro_/imu_accel_ 可能被 IMU 后台线程异步更新，
    // 因此用 imu_mtx_ 保护，避免读到"半更新"的数据（例如新的四元数配旧的角速度）。
    // ------------------------------------------------------------------
    {
        std::lock_guard<std::mutex> lock(imu_mtx_);
        // 注意顺序：rl_sdk 约定 quaternion 为 [w, x, y, z]（w 在前）
        state->imu.quaternion[0] = imu_quat_[0];   // w（实部）
        state->imu.quaternion[1] = imu_quat_[1];   // x
        state->imu.quaternion[2] = imu_quat_[2];   // y
        state->imu.quaternion[3] = imu_quat_[3];   // z
        for (int i = 0; i < 3; ++i)
        {
            state->imu.gyroscope[i]     = imu_gyro_[i];    // rad/s
            state->imu.accelerometer[i] = imu_accel_[i];   // m/s^2
        }
    }

    // ------------------------------------------------------------------
    // 步骤 2：16 个电机反馈，按 joint_mapping 从 physical 顺序取回 policy 顺序
    // ------------------------------------------------------------------
    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    int num_dofs = this->params.Get<int>("num_of_dofs");   // ancnW1 = 16

    // policy_idx 遍历策略期望的顺序，换算成 physical_idx 后去 CAN 取对应电机
    for (int policy_idx = 0; policy_idx < num_dofs; ++policy_idx)
    {
        // policy → physical：查 joint_mapping 表
        int physical_idx = joint_mapping[policy_idx];

        // physical → motor_id：<｜hy_place▁holder▁no▁813｜> 电机的 CAN 编号从 1 起算，而 physical 索引从 0 起算，故需 +1
        uint8_t motor_id = static_cast<uint8_t>(physical_idx + 1);

        std::array<uint8_t, CanInterface::kFeedbackLen> raw{};
        MotorFeedback fb;

        // 两步：先从 per-motor 缓存取原始 12 字节，再按该电机型号阈值解析成物理量
        if (can_.GetLatestFeedback(motor_id, &raw) &&
            driver_.ParseFeedback(raw.data(), raw.size(), &fb))
        {
            // 写回时下标用 policy_idx —— 完成 physical → policy 的重排
            state->motor_state.q[policy_idx]       = fb.q;     // rad
            state->motor_state.dq[policy_idx]      = fb.dq;    // rad/s
            state->motor_state.tau_est[policy_idx] = fb.tau;   // N·m
            // ddq / cur 留 0（反馈帧无此字段；如需角加速度可另行差分外推）
        }
        // else：取不到反馈 → 不写，保留 state 里上一次的值（见 @note）
    }
}

/**
 * @brief 【RL 框架回调】把策略算出的关节指令下发给电机
 *
 * 由 RobotControl() 每个控制周期调用一次（200Hz），是 rl_sdk 的纯虚函数。
 *
 * 处理流程：
 *   ① 安全检查（空指针 / CAN 未开 / 电机状态机处于 Fault）
 *   ② 调 AssembleRunFrames() 把 16 个关节的指令拆成 3 帧 64 字节
 *   ③ 以主站 CANID=0x00 依次发出
 *
 * @param[in] command 策略输出的关节命令，字段含义（下标为 policy 顺序）：
 *        command->motor_command.q[16]     目标位置 rad
 *        command->motor_command.dq[16]    目标速度 rad/s
 *        command->motor_command.kp[16]    刚度（rl_kp，腿 40 / 轮 0）
 *        command->motor_command.kd[16]    阻尼（rl_kd，腿 1.0 / 轮 0.6）
 *        command->motor_command.tau[16]   前馈力矩 N·m（已含 rl_kp 的 PD 项）
 *
 * @note 为何 Fault 时直接 return 而不是发"零位零力矩"？
 *       故障可能源于通信异常，此时继续发帧只会加剧总线负担且无法保证送达；
 *       真正的断电保护由 MotorFsm 负责（它会把状态推向失能）。
 */
void RL_Real_AncnW1::SetCommand(const RobotCommand<float> *command)
{
    if (command == nullptr || !can_opened_)
    {
        return;
    }

    // 故障状态：不下发任何运行指令，由 MotorFsm 负责把电机导向失能
    if (motor_fsm_.State() == MotorFsmState::Fault)
    {
        return;
    }

    // 1. 按 joint_mapping 重排 + 拆成 3 帧（每帧最多 6 个电机槽位，见手册 §3.2.2）
    //    msgid_counter_ 递增：下发的 MSGID 会被电机原样回显，
    //    便于在多帧接收时判断反馈对应哪一次下发（手册 §3.2.2 注3）
    std::array<std::array<uint8_t, CanInterface::kMaxFrameLen>, 3> frames;
    AssembleRunFrames(command, &frames, msgid_counter_++);

    // 2. 发送。主站 CANID = 0x00（手册 §3.2.1）
    for (const auto &frame : frames)
    {
        can_.Send(0x00, frame.data(), frame.size());
    }
}

// ============================================================================
// 控制循环
// ============================================================================

void RL_Real_AncnW1::RobotControl()
{
    // 1. 更新 IMU 数据（IMU 未初始化时桩函数直接返回，原值保持）
    UpdateImuData();

    // 2. 推进电机状态机（检查反馈、超时、故障）
    //    Tick() 内部会读所有电机反馈，检测到故障会进 Fault
    motor_fsm_.Tick();

    // 3. 读电机反馈 → 填 robot_state（按 joint_mapping 重排）
    this->GetState(&this->robot_state);

    // 4. FSM 决策（GetUp/GetDown/RL/Jitter/StepInPlace/StepTurn 等）
    //    把目标指令写入 this->robot_command
    this->StateController(&this->robot_state, &this->robot_command);

    // 5. 清输入
    this->control.ClearInput();

    // 6. 故障保护：Fault 状态下不发运行指令
    if (motor_fsm_.State() == MotorFsmState::Fault)
    {
        MotorFaultRecord rec = motor_fsm_.LastFault();
        std::cout << LOGGER::WARNING << "[ancnW1] motor fault, motor_id="
                  << static_cast<int>(rec.motor_id) << ", type="
                  << static_cast<int>(rec.type) << " — ClearFault() to retry" << std::endl;
        return;
    }

    // 7. 发送指令到电机（按 leg/wheel 分流后打包 3 帧 CAN-FD）
    this->SetCommand(&this->robot_command);
}

// ============================================================================
// 电机命令组装
// ============================================================================

void RL_Real_AncnW1::AssembleRunFrames(
    const RobotCommand<float> *command,
    std::array<std::array<uint8_t, CanInterface::kMaxFrameLen>, 3> *frames,
    uint16_t msgid)
{
    // 算法：
    //   1. 把 RobotCommand 的 16 个电机值（policy 顺序）按 joint_mapping 重排
    //      → physical 顺序数组（索引 i = physical index = motor id - 1）
    //   2. 关键修正：电机 Kp/Kd 字段强制置 0
    //      → 避免与 tau 里的 PD 项重复（rl_sar 内部 tau 已含 2*rl_kp 的 PD）
    //      → 真机等效刚度 = 2*rl_kp，与 Isaac 训练一致
    //   3. 按物理电机 ID 1-16 顺序拆成 3 帧：
    //      帧 1: 电机 1-6,  帧 2: 电机 7-12,  帧 3: 电机 13-16（剩 2 槽填空）

    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    int num_dofs = this->params.Get<int>("num_of_dofs");

    // 1) policy → physical 重排
    std::array<MotorRunCommand, CanInterface::kMaxMotorId> physical_cmds{};
    for (int policy_idx = 0; policy_idx < num_dofs; ++policy_idx)
    {
        int physical_idx = joint_mapping[policy_idx];
        if (physical_idx < 0 || physical_idx >= CanInterface::kMaxMotorId)
        {
            continue;
        }

        MotorRunCommand cmd;
        cmd.q = command->motor_command.q[policy_idx];
        cmd.dq = command->motor_command.dq[policy_idx];
        cmd.kp = command->motor_command.kp[policy_idx];   // 先取原值
        cmd.kd = command->motor_command.kd[policy_idx];
        cmd.tau = command->motor_command.tau[policy_idx];
        physical_cmds[physical_idx] = cmd;
    }

    // 2) Kp/Kd **原样透传**，不要清零！
    //
    //    依据 config.yaml 的注释："position term twice (output_dof_tau +
    //    SetCommand's kp*(cmd_q - q)), so effective stiffness/damping = 2x，
    //    legs 40/1.0 == Isaac stiffness 80 / damping 2.0"
    //
    //    推导（手册 §3.1 阻抗模型）：
    //      电机力矩 = Kp*(q_cmd - q_fb) + Kd*(dq_cmd - dq_fb) + tau_ff
    //    透传时 Kp=rl_kp=40、tau_ff=output_dof_tau=40*(target-pos)-1.0*vel：
    //      = 40*(target-pos) + 1.0*(0-vel) + [40*(target-pos) - 1.0*vel]
    //      = 80*(target-pos) - 2.0*vel           ← 正好等于 Isaac stiffness 80 / damping 2.0 ✓
    //
    //    若在此把 Kp 清零，等效只剩 40 —— 机器人刚度腰斩、站不起来。
    //
    //    轮子自动安全：rl_kp 最后 4 项是 0，所以 motor_command.kp=0，
    //    天然满足手册"轮毂电机不支持位置控制"的禁令，无需额外处理。

    // 3) 拆 3 帧：电机 1-6 / 7-12 / 13-16
    std::array<uint8_t, 6> ids_1 = {1, 2, 3, 4, 5, 6};
    std::array<uint8_t, 6> ids_2 = {7, 8, 9, 10, 11, 12};
    std::array<uint8_t, 6> ids_3 = {13, 14, 15, 16, 0, 0};   // 最后 2 槽空

    std::array<MotorRunCommand, 6> cmds_1, cmds_2, cmds_3;
    for (int i = 0; i < 6; ++i)
    {
        cmds_1[i] = physical_cmds[i];          // physical 0-5  → motor 1-6
        cmds_2[i] = physical_cmds[i + 6];      // physical 6-11 → motor 7-12
        cmds_3[i] = physical_cmds[i + 12];     // physical 12-17 → motor 13-16
    }

    driver_.BuildRunFrame(ids_1, cmds_1, msgid, &(*frames)[0]);
    driver_.BuildRunFrame(ids_2, cmds_2, msgid, &(*frames)[1]);
    driver_.BuildRunFrame(ids_3, cmds_3, msgid, &(*frames)[2]);
}

// ============================================================================
// 模型推理 + 决策循环
// ============================================================================

void RL_Real_AncnW1::RunModel()
{
    // 参照 go2 RunModel（rl_real_go2.cpp:212）
    // 注意：ancnW1 不观测 lin_vel（base.yaml observations 没有该项）

    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;

        // 1. 填充观测（FSM 推入的指令已在上一周期写入 robot_command，
        //    RL 框架会在 ComputeOutput 时用 obs.actions = 历史 actions 做延迟输入）
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.commands[0] = this->control.x;
        this->obs.commands[1] = this->control.y;
        this->obs.commands[2] = this->control.yaw;
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        // 2. 模型推理（带 mutex 保护，模型热加载时不阻塞）
        this->obs.actions = this->Forward();

        // 3. ComputeOutput：把 actions 转成 (pos, vel, tau) 目标
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        // 4. 推入命令队列（基类 StateController 在 control loop 里消费）
        if (!this->output_dof_pos.empty())
        {
            this->output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            this->output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            this->output_dof_tau_queue.push(this->output_dof_tau);
        }
    }
}

}  // namespace ancn_w1

// ============================================================================
// main
// ============================================================================

int main(int argc, char **argv)
{
    ancn_w1::RL_Real_AncnW1 node(argc, argv);
    node.Run();
    return 0;
}
