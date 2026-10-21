/*
 * Copyright (c) 2024-2025 ancn
 * SPDX-License-Identifier: Apache-2.0
 *
 * ancnW1 状态机（RLFSM 层）
 * ----------------------------------------
 * 4 个核心状态：
 *   Passive      → 电机零刚度、零力矩（靠电机阻尼抱住，可徒手掰动）
 *   GetUp        → 关节插值到 default_dof_pos（站起来）
 *   GetDown      → 关节插值回 start_state（趴下）
 *   RLLocomotion → 由 RL 策略控制
 *
 * 状态转换：
 *   Passive --[0/A]--> GetUp --[1/DPadUp]--> RLLocomotion
 *   GetUp --[9/B]--> GetDown
 *   GetDown --[P/LB_X]--> Passive
 *   RLLocomotion --[9/B]--> GetDown
 *   任意 --[P/LB_X]--> Passive
 *
 * ==================== RLFSMState 生命周期 ====================
 * 由 FSMManager 驱动，所有状态派生自基类 RLFSMState：
 *
 *   Enter()       进入状态时调用**一次**，做初始化
 *   Run()         每个控制周期（200Hz）调用，负责写 fsm_command
 *   Exit()        离开状态时调用**一次**，做清理
 *   CheckChange() 每个控制周期在 Run() **之后**调用；
 *                 返回目标状态名即触发转移，返回 state_name_ 表示不转移
 *
 * 基类提供的关键成员：
 *   fsm_command → RobotCommand*，本状态要下发给关节的指令（本状态负责填充）
 *   fsm_state   → RobotState*，当前机器人状态（只读）
 *   rl.now_state→ RobotState，插值起点快照
 *   state_name_ → 本状态的字符串名称
 *
 * ==================== 操作键位（键盘 / 手柄）====================
 *   ┌──────┬────────────┬──────────────────────────┐
 *   │ 按键 │ 手柄       │ 动作                     │
 *   ├──────┼────────────┼──────────────────────────┤
 *   │ 0    │ A          │ 进入 GetUp（起立）       │
 *   │ 1    │ RB + 方向上│ 进入 RLLocomotion（行走）│
 *   │ 9    │ B          │ 进入 GetDown（趴下）     │
 *   │ P    │ LB + X     │ 急停 → Passive           │
 *   └──────┴────────────┴──────────────────────────┘
 *   键盘与手柄**平权**（逻辑或），任一满足即触发。
 *
 * ancnW1 特殊：
 *   - default_dof_pos 已经是"站立位姿"（前后腿折叠方向不同），无需 pre_running_pos 过渡
 *   - 轮子（policy idx 12-15）始终 kp=0（手册禁止轮毂电机位置控制）
 *   - 腿（policy idx 0-11）用 fixed_kp/kd（Passive）或 rl_kp/rl_kd（RLLocomotion）
 *
 * 未实现状态（Jitter/StepInPlace/StepTurn）：
 *   base.yaml 里有对应配置项，但本次只搭核心 4 状态。后续按需补
 */

#ifndef ANCNW1_FSM_HPP
#define ANCNW1_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

namespace ancnW1_fsm
{

/**
 * @brief Passive：电机零刚度、零力矩（电机内部阻尼抱住）
 */
class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    /// 进入 Passive：仅提示可用操作
    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    /**
     * @brief 每周期下发"被动保持"指令
     *
     * 逐关节填写 motor_command（下标均为 policy 顺序）：
     *   q   = 0        目标位置无意义（kp=0 时电机不理睬该字段）
     *   dq  = 0        同理
     *   kp  = 0        **关键**：零刚度 → 电机不做位置伺服，可被徒手掰动
     *   kd  = fixed_kd 保留阻尼，让关节对手的外力有"粘滞感"
     *   tau = 0        无前馈力矩
     *
     * @note 为什么 kd 不能也置 0？
     *       若 kd=0，腿部将完全没有支撑力矩，会在重力作用下自由松垮砸下去，
     *       可能砸伤自身结构或线缆。保留 fixed_kd 相当于给关节一个"缓冲"，
     *       既能任人推动，又不会失控坠落 —— 这是调试时的安全姿态。
     */
    void Run() override
    {
        int num_dofs = rl.params.Get<int>("num_of_dofs");
        for (int i = 0; i < num_dofs; ++i)
        {
            // 全部零刚度 + 阻尼抱住；力矩为 0
            fsm_command->motor_command.q[i] = 0;
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = rl.params.Get<std::vector<float>>("fixed_kd")[i];
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    /// Passive 无资源需要清理
    void Exit() override {}

    /**
     * @brief 判断是否要离开 Passive
     * @return "RLFSMStateGetUp"=收到起立指令；state_name_=保持不动
     *
     * @note Passive 是初始状态，因此这里**只接受到 GetUp 的单向出口**，
     *       不接受切到 GetDown / RLLocomotion —— 必须先站起来。
     */
    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

/**
 * @brief GetUp：从 start_state 插值到 default_dof_pos
 *
 * ancnW1 的 default_dof_pos 已经是站立姿态，所以不用 pre_running_pos 过渡
 * （直接 1 段插值，约 2 秒）
 */
class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    /// 起立插值进度 0.0→1.0，由 Interpolate() 内部按 dt 推进
    float percent_getup = 0.0f;

    /**
     * @brief 进入 GetUp：做插值起点快照
     *
     * 做了两件事：
     *   ① rl.now_state = 当前实际关节位置 —— 作为本次插值的**起点**
     *   ② rl.start_state = 该快照 —— 保存到全局，供后续 GetDown 插回来用
     *
     * @note 为什么要在 Enter 里快照而不是每周期读实时位置？
     *       因为插值需要一条固定的起点，若每周期读实时值，
     *       起点会追着终点跑，导致"永远追不上、动作变慢甚至停在半路"。
     */
    void Enter() override
    {
        percent_getup = 0.0f;
        rl.now_state = *fsm_state;      // ① 插值起点 = 此刻真实位置
        rl.start_state = rl.now_state;  // ② 备份，供 GetDown 使用
    }

    /**
     * @brief 每周期推进插值并下发途中点位
     *
     * @note Interpolate() 是基类方法，职责为：
     *         1. percent 按 dt/duration 递增，并 clamp 到 [0,1]
     *         2. 在 起点→终点 之间做线性插值
     *         3. 用 fixed_kp/fixed_kd 作为途中增益（最后一个参数 true）
     *         4. 打印进度百分比，到达 1.0 时输出 "Getting up 100%"
     *       本状态只管调用，具体 PD 由基类填进 fsm_command。
     */
    void Run() override
    {
        // 1 段插值：current pos → default_dof_pos，2 秒，fixed gains
        Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true);
    }

    /// GetUp 无资源需要清理
    void Exit() override {}

    /**
     * @brief 判断是否离开 GetUp
     *
     * 优先级：
     *   ① P / LB+X（急停）—— **无条件最高优先级**，任何时候都能立即回 Passive
     *   ② 起立未完成（percent<1）—— 除急停外不接受任何切换，
     *      避免在起身一半时切到行走导致摔倒
     *   ③ 起立完成（percent>=1）—— 接受 1→行走 或 9→趴下
     *
     * @return 目标状态名；state_name_ 表示保持
     */
    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

/**
 * @brief GetDown：从 current 插值回 start_state
 */
class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    /// 趴下插值进度 0.0→1.0
    float percent_getdown = 0.0f;

    /// 进入 GetDown：以当前实际位置作为插值起点（与 GetUp 同样的快照理由）
    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;   // 起点 = 此刻真实位置
    }

    /**
     * @brief 每周期推进插值：当前姿态 → 初始趴姿
     *
     * @note 终点是 **rl.start_state**（不是某个"趴下配置"）：
     *       start_state 是在**首次进入 GetUp 时**记录下来的当时关节位置，
     *       也就是机器人最初趴着的样子。这样 GetDown 能精确回到出发点，
     *       不依赖 yaml 里再维护一份"趴姿"。
     *
     * @note 隐含前提：必须先经历过一次 GetUp，start_state 才有意义。
     *       由于 Passive 只能 → GetUp，这条路径天然得到保证。
     */
    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    /// GetDown 无资源需要清理
    void Exit() override {}

    /**
     * @brief 判断是否离开 GetDown
     *
     * 与 GetUp 不同，这里 **percent>=1.0 会自动回 Passive**：
     * 趴下动作完成后机器人应处于失能/松垮状态，
     * 若继续停留在 GetDown 并保持 fixed_kp 的位置伺服，
     * 电机会持续出力维持姿态、白白发热。
     *
     * @return Passive（急停 或 趴下完成）；GetUp（中途反悔）；否则保持
     */
    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

/**
 * @brief RLLocomotion：策略控制
 *
 * 进入时初始化 RL（读 robot_lab/config.yaml + 加载模型）
 * Run() 内调 RLControl()，由基类把 output_dof_pos/vel/tau 填入 fsm_command
 */
class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    /**
     * @brief 进入行走状态：加载 RL 配置与模型
     *
     * 流程：
     *   ① episode_length_buf = 0 —— 重置观测历史计数（防止带着上一段的历史观测推理）
     *   ② 指定 config_name = "robot_lab" —— 即 policy/ancnW1/robot_lab/config.yaml
     *   ③ InitRL() —— 读配置、加载 .pt 模型、初始化观测 scaler，**可能抛异常**
     *   ④ now_state 快照 —— 供基类 RLControl 使用
     *
     * @note 为何 InitRL 要 try/catch 并退回 Passive？
     *       模型文件路径错误、torch 版本不匹配、显存不足都会导致加载失败。
     *       若不捕获，异常会一路抛到主循环导致进程崩溃；
     *       而此刻机器人正站着，崩溃意味着立刻失去控制而摔倒。
     *       回 Passive（松弛、可徒手扶住）是此刻最安全的落点。
     */
    void Enter() override
    {
        rl.episode_length_buf = 0;

        // 加载 ancnW1/robot_lab/config.yaml
        rl.config_name = "robot_lab";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);   // 失败会 throw
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");   // 安全兜底
        }
    }

    /**
     * @brief 每周期把模型输出透传给关节
     *
     * @note rl.rl_init_done 的置位逻辑：
     *       Enter 里若 InitRL 成功会置 true；失败则 catch 里置 false。
     *       此处再次置 true 是为了在极端情况下（例如外部把该标志清掉）
     *       保证本状态运行期间标记是一致的。
     *
     * @note 打印用了 ANSI 转义序列 "\r\033[K"：
     *       \r 回到行首，\033[K 清除到行尾 —— 实现**原地刷新**，
     *       让 control.x/y/yaw 在一行上持续更新，而不是刷屏滚动。
     *       生产环境若日志量大可考虑降频打印。
     *
     * @note 真正的策略输出来自基类 RLControl()：
     *       它从 output_dof_pos / vel / tau 三个队列里取最新值填入 fsm_command。
     *       这些队列由 RunModel() 在**另一个循环**中按 dt*decimation 推入
     *       —— 即模型推理与控制下发是解耦的两个频率。
     */
    void Run() override
    {
        if (!rl.rl_init_done)
        {
            rl.rl_init_done = true;
        }

        // 实时打印策略的速度指令（原地刷新）
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                  << "RL Controller [" << rl.config_name << "] x:"
                  << rl.control.x << " y:" << rl.control.y
                  << " yaw:" << rl.control.yaw << std::flush;

        // 基类 RLControl()：从 output_dof_pos/vel/tau 队列取最新值，填到 fsm_command
        // 注意：output_dof_* 队列由 RunModel() 在另一个 loop（dt*decimation）推入
        RLControl();
    }

    /**
     * @brief 离开行走状态：清掉初始化标记
     *
     * 置 rl_init_done = false 意味着**下次再进入本状态时会重新走 Enter→InitRL**，
     * 而不是复用上次加载的模型。这样能支持热切换模型/配置。
     */
    void Exit() override
    {
        rl.rl_init_done = false;
    }

    /**
     * @brief 判断是否离开行走状态
     *
     * 判断顺序即优先级（**急停永远排第一**）：
     *   ① P / LB+X  → Passive        急停，无条件生效
     *   ② 9 / B     → GetDown        正常趴下
     *   ③ 0 / A     → GetUp          重新起立（异常姿态纠正）
     *   ④ 1 / RB+上 → RLLocomotion   返回自身，等于不转移
     *
     * @return 目标状态名；state_name_ 表示保持当前状态
     *
     * @note 第 ④ 项返回自身名字，看起来冗余，
     *       但它显式表达了"已注册该按键、此处有意不做处理"，
     *       便于将来需要时在对应分支插入逻辑。
     */
    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }
        return state_name_;
    }
};

}  // namespace ancnW1_fsm

/**
 * @brief ancnW1 状态机工厂（注册到 FSMManager）
 *
 * GetType() 返回的字符串必须与 base.yaml 的顶层 key 一致："ancnW1"
 */
class AncnW1FSMFactory : public FSMFactory
{
public:
    AncnW1FSMFactory(const std::string& initial) : initial_state_(initial) {}

    /**
     * @brief 按状态名创建对应的状态对象（工厂方法）
     *
     * @param context   框架传入的上下文指针，此处为 RL*（需向下转型）
     * @param state_name 要创建的状态名，例如 "RLFSMStateGetUp"
     * @return 状态实例；**未知状态名返回 nullptr**
     *
     * @note 为何用 void* 而不是 RL*？
     *       基类 FSMFactory 是通用抽象，不能绑定具体 RL 类型，
     *       因此这里向下转型一次。调用方保证传入的确实是 RL*。
     *
     * @note 返回 nullptr 时 FSMManager 会拒绝该次状态切换，
     *       比对未知状态做兜底处理更安全 —— 宁可切换失败，也不进错状态。
     */
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<ancnW1_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<ancnW1_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<ancnW1_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<ancnW1_fsm::RLFSMStateRLLocomotion>(rl);
        return nullptr;
    }

    /**
     * @brief 返回本工厂的类型标识
     *
     * @warning 该字符串必须与 **base.yaml 的顶层 key 完全一致**（此处为 "ancnW1"），
     *          否则框架在按 robot_name 查找工厂时匹配不到，导致启动即失败。
     */
    std::string GetType() const override { return "ancnW1"; }

    /**
     * @brief 列出本工厂支持的所有状态名
     * @return 状态名列表，用于启动时的合法性校验与日志显示
     */
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion"
        };
    }

    /// 返回上电后的初始状态（构造时指定，默认 Passive）
    std::string GetInitialState() const override { return initial_state_; }

private:
    /// 初始状态名，由构造函数注入，便于测试时改成其他状态
    std::string initial_state_;
};

/**
 * @brief 静态注册本工厂到框架的 FSMManager
 *
 * 宏展开后会在静态初始化阶段（main 之前）创建一个工厂实例并登记，
 * 因此无需在别处手动 new 或注册 —— 只要这个头文件被链接进来即可。
 *
 * 第二个参数 **"RLFSMStatePassive"** 指定上电初始状态：
 * 机器人通电后应处于松弛可扶住的安全姿态，而不是立刻站立。
 */
REGISTER_FSM_FACTORY(AncnW1FSMFactory, "RLFSMStatePassive")

#endif  // ANCNW1_FSM_HPP
