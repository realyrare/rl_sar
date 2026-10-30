/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 * 这是一个轮腿四足机器人的编排层,提供了「趴下 / 起身 / 下趴 / 原地抖动 / 原地踏步 / 
 原地踏步转圈 / RL 策略行走」七种模式,机械腿用位置环做运动学踏步,轮子用位置环兼当刹车与差速转向,
 所有模式可通过键盘或手柄实时切换
 */

#ifndef ANCNW1_FSM_HPP
#define ANCNW1_FSM_HPP

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "fsm.hpp"
#include "rl_sdk.hpp"

//状态类 + 工具函数
namespace ancnW1_fsm
{

// Wheel height gained per radian of the swing parameter s, measured on the URDF
// with a numeric IK around the standing pose (s = 0.25 -> 4.4 cm, s = 0.35 -> 7.1 cm).
// Only used for the on-screen printout, the gait itself is pure kinematics.
static constexpr float LIFT_M_PER_RAD = 0.203f;

// Wheel geometry taken from the URDF / standing FK: wheel radius 0.11 m, half track
// (|y| of the wheel centre) 0.22 m. Skid-steering the body with a yaw rate w
// therefore needs a differential wheel speed of w * HALF_TRACK / WHEEL_RADIUS = 2w.
static constexpr float WHEEL_RADIUS = 0.11f;
static constexpr float HALF_TRACK = 0.22f;

// Gait generator shared by the two stepping states. Fills target[] in policy order.
// The wheels are commanded to  wheel_ref + wheel_dir[leg] * wheel_spin, so
//   wheel_dir = {0,0,0,0}                 -> wheels held, the body cannot drift
//   wheel_dir = {+1,-1,+1,-1} (left/right) -> differential drive, turn on the spot
// swing_leg / swing_peak report which leg (pair) is in the air and how high.
inline void BuildStepTargets(RL &rl,
                             float gait_time,
                             const std::vector<float> &wheel_ref,
                             const float wheel_dir[4],
                             float wheel_spin,
                             std::vector<float> &target,
                             int &swing_leg,
                             float &swing_peak,
                             bool &trot)
{
    const float period = rl.params.Get<float>("step_period", 1.0f);
    const float lift = rl.params.Get<float>("step_lift", 0.45f);
    const float stance_drop = rl.params.Get<float>("step_stance_drop", 0.10f);
    const float calf_ratio = rl.params.Get<float>("step_calf_ratio", 2.0f);
    trot = rl.params.Get<bool>("step_trot", true);

    float phase = std::fmod(gait_time, period) / period;   // 0..1 inside the cycle
    if (phase < 0.0f) phase += 1.0f;

    float offsets[4];
    float duty;
    const float duty_cfg = rl.params.Get<float>("step_duty", 0.0f);
    if (trot)
    {
        // diagonal pairs: fl+hr and fr+hl. The two planted wheels sit on a line
        // through the centre of mass, so the pair in the air really leaves the
        // ground (verified: contact alternates between 2 diagonal wheels and 4).
        offsets[0] = 0.00f; offsets[3] = 0.00f;
        offsets[1] = 0.50f; offsets[2] = 0.50f;
        duty = (duty_cfg > 0.0f) ? duty_cfg : 0.40f;
    }
    else
    {
        // 4-beat crawl: one wheel in the air at a time, three always planted
        offsets[2] = 0.00f;   // hl
        offsets[0] = 0.25f;   // fl
        offsets[3] = 0.50f;   // hr
        offsets[1] = 0.75f;   // fr
        duty = (duty_cfg > 0.0f) ? duty_cfg : 0.25f;
    }
    duty = std::min(duty, 0.49f);

    // smooth 0 -> 1 -> 0 bump over the swing window
    auto swing = [&](int leg) -> float
    {
        float x = phase - offsets[leg];
        x -= std::floor(x);
        if (x >= duty) return 0.0f;
        return 0.5f * (1.0f - std::cos(2.0f * 3.14159265f * x / duty));
    };

    target = rl.params.Get<std::vector<float>>("default_dof_pos");
    const int thigh_idx[4] = {1, 4, 7, 10};   // fl, fr, hl, hr (policy order)
    const int calf_idx[4] = {2, 5, 8, 11};
    swing_leg = -1;
    swing_peak = 0.0f;
    for (int leg = 0; leg < 4; ++leg)
    {
        float sw = swing(leg);
        // sw = 1 -> leg in the air (lifted), sw = 0 -> leg planted and slightly
        // extended, which raises the body and adds clearance for the other pair
        float s = lift * sw - stance_drop * (1.0f - sw);
        float sign = (leg < 2) ? -1.0f : 1.0f;   // front legs fold the other way
        target[thigh_idx[leg]] += sign * s;
        target[calf_idx[leg]] -= sign * calf_ratio * s;
        if (s > swing_peak)
        {
            swing_peak = s;
            swing_leg = leg;
        }
    }

    for (int leg = 0; leg < 4; ++leg)
    {
        const int w = 12 + leg;
        target[w] = ((size_t)w < wheel_ref.size() ? wheel_ref[w] : 0.0f) + wheel_dir[leg] * wheel_spin;
    }
}

// Writes a stepping target into the motor command: legs use fixed_kp/fixed_kd
// (or step_leg_kp/step_leg_kd when both are > 0), the wheels use a position loop
// (step_wheel_kp/kd) which acts as a parking brake when the target is held.
inline void WriteStepCommand(RL &rl, RobotCommand<float> *cmd, const std::vector<float> &target)
{
    auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
    auto kd = rl.params.Get<std::vector<float>>("fixed_kd");
    const float leg_kp = rl.params.Get<float>("step_leg_kp", 0.0f);
    const float leg_kd = rl.params.Get<float>("step_leg_kd", 0.0f);
    if (leg_kp > 0.0f && leg_kd > 0.0f)
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            kp[i] = leg_kp;
            kd[i] = leg_kd;
        }
    }
    const float wheel_kp = rl.params.Get<float>("step_wheel_kp", 15.0f);
    const float wheel_kd = rl.params.Get<float>("step_wheel_kd", 2.0f);
    for (int w : rl.params.Get<std::vector<int>>("wheel_indices"))
    {
        kp[w] = wheel_kp;
        kd[w] = wheel_kd;
    }
    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
    {
        cmd->motor_command.q[i] = target[i];
        cmd->motor_command.dq[i] = 0.0f;
        cmd->motor_command.kp[i] = kp[i];
        cmd->motor_command.kd[i] = kd[i];
        cmd->motor_command.tau[i] = 0.0f;
    }
}

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    // 为什么这里不是"完全松软"（kp = 0）
    // ------------------------------------
    // 原先这里对所有关节只给阻尼（kp=0, kd=8），腿撑不住机身，于是开机后机器
    // 人一路塌下去：1 秒就掉到 z≈0.29，最终停在前倾 17° 的怪姿态，且**再也站
    // 不起来** —— 实测从趴姿 GetUp，kp 从 80 提到 400 全都失败（最高只抬到
    // z≈0.43 且会翻车），因为后腿折叠到关节极限（-2.25 rad，range 下限 -2.286）
    // 后进入奇异位形，伸展时轮子打滑而不是把机身撑起来。也就是说 ancnW1 一旦
    // 趴下就无法自救，这跟 M20（趴姿是它的稳定态、且能自行起身）完全不同。
    //
    // 所以本状态改为"开机保持站立姿态"：用 fixed_kp / fixed_kd 把关节保持在
    // default_dof_pos。静态所需力矩实测只有 8 N·m（腿限 113），负担极小，仍
    // 是外力可推动的松软手感，但不会自己塌成怪姿态，随时可以按 0 进入 GetUp。
    //
    // 想恢复成原来的完全松软（仅阻尼），把下面 kp 那一行改回 0 即可。
    // 为什么这里改成"跟随当前姿态"（与 m20 完全一致）
    // ----------------------------------------------------
    // 之前这里曾是"开机硬撑站立"（fixed_kp 拉向 default_dof_pos），当时的原因是
    // 旧趴姿（后腿折叠到 -2.25 rad 关节极限附近）GetUp 起不来。现在 keyframe[0]
    // 已换成新的 crouch（thigh=-1.222/calf=+1.571，远离限位、远离奇异位形），
    // 用户要求与 m20 一致：出生趴着、按 0 起立、按 9 趴回。硬撑站立会让出生
    // keyframe 形同虚设 —— 一进 Passive 就被拉回站姿。
    // 本状态现在与 fsm_m20.hpp 的 Passive 逐行相同：目标位跟随当前位置（kp 项
    // 恒为 0，只输出 -kd*dq 的小刚度保持），姿态由 crouch keyframe 决定。
    // 若实测从新趴姿 GetUp 仍困难，优先调 GetUp 的时长/力度，而不是改回硬撑站立。
    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 20;
            fsm_command->motor_command.kd[i] = 10;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    // crouch pose before standing up, POLICY order:
    // 12 leg joints (fl_hip, fl_thigh, fl_calf, fr_..., hl_..., hr_...) + 4 wheels
    // 训练侧 assets/ancn.py 的 init_state.joint_pos 全 0 —— URDF 零位本身就是屈膝
    // 站姿，所以 GetUp 的目标只有 default_dof_pos（全 0），不再需要"先深蹲再起立"
    // 的中间姿态（那是旧站姿体系 0.6/-1.0 时代的产物，符号体系是 m20 的，对 ancnW1 反向）。
    std::vector<float> pre_running_pos = {
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00
    };
    bool stand_from_passive = true;

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        if (stand_from_passive)
        {
            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 1.0f, "Pre Getting up", true)) return;
            if (Interpolate(percent_getup, pre_running_pos, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true)) return;
        }
        else
        {
            if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 1.0f, "Getting up", true)) return;
        }
    }

    void Exit() override {}

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
            else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
            {
                return "RLFSMStateJitter";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
            {
                return "RLFSMStateStepInPlace";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
            {
                return "RLFSMStateStepTurn";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

// Shake-in-place ("原地抖动"): the standing pose is kept, but every leg joint
// is driven with a small, fast sinusoid on top of it, so the whole body trembles
// without travelling. This is a *different* mode from stepping: the wheels never
// leave the ground, the amplitude is ~1 cm and the frequency is several Hz.
//
// The oscillation is applied along the same direction the leg uses to lift the
// wheel (dthigh = -s, dcalf = +2s for the front legs, mirrored for the hind
// legs), so the visible motion is the body bouncing up and down.
class RLFSMStateJitter : public RLFSMState
{
public:
    RLFSMStateJitter(RL *rl) : RLFSMState(*rl, "RLFSMStateJitter") {}

    float percent_transition = 0.0f;
    float jitter_time = 0.0f;
    std::vector<float> wheel_locked_pos;

    void Enter() override
    {
        percent_transition = 0.0f;
        jitter_time = 0.0f;
        rl.now_state = *fsm_state;
        wheel_locked_pos = rl.now_state.motor_state.q;
        std::cout << LOGGER::NOTE << "Entered jitter mode (shake in place). '3' step, '4' step+turn, '1' RL, '0' GetUp, '9' GetDown, 'P' passive." << std::endl;
    }

    void Run() override
    {
        if (Interpolate(percent_transition, rl.now_state.motor_state.q,
                        rl.params.Get<std::vector<float>>("default_dof_pos"),
                        rl.params.Get<float>("jitter_transition", 0.5f),
                        "Jitter", true))
        {
            return;
        }

        const float amp = rl.params.Get<float>("jitter_amp", 0.05f);          // rad on the thigh
        const float freq = rl.params.Get<float>("jitter_freq", 5.0f);         // Hz
        const float spread = rl.params.Get<float>("jitter_phase_spread", 0.0f); // 0 = all legs in phase
        const float bias = rl.params.Get<float>("jitter_bias", 0.0f);         // >0: only lift, never press
        const float calf_ratio = rl.params.Get<float>("jitter_calf_ratio", 2.0f);

        jitter_time += rl.params.Get<float>("dt");
        const float omega = 2.0f * 3.14159265f * freq;

        auto target = rl.params.Get<std::vector<float>>("default_dof_pos");
        const int thigh_idx[4] = {1, 4, 7, 10};   // fl, fr, hl, hr (policy order)
        const int calf_idx[4] = {2, 5, 8, 11};
        for (int leg = 0; leg < 4; ++leg)
        {
            float phase = omega * jitter_time + 2.0f * 3.14159265f * spread * static_cast<float>(leg);
            float s = amp * (bias + (1.0f - bias) * std::sin(phase));
            float sign = (leg < 2) ? -1.0f : 1.0f;
            target[thigh_idx[leg]] += sign * s;
            target[calf_idx[leg]] -= sign * calf_ratio * s;
        }

        auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
        auto kd = rl.params.Get<std::vector<float>>("fixed_kd");
        const float leg_kp = rl.params.Get<float>("jitter_leg_kp", 0.0f);
        const float leg_kd = rl.params.Get<float>("jitter_leg_kd", 0.0f);
        if (leg_kp > 0.0f && leg_kd > 0.0f)
        {
            for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
            {
                kp[i] = leg_kp;
                kd[i] = leg_kd;
            }
        }

        const float wheel_kp = rl.params.Get<float>("step_wheel_kp", 15.0f);
        const float wheel_kd = rl.params.Get<float>("step_wheel_kd", 2.0f);
        for (int w : rl.params.Get<std::vector<int>>("wheel_indices"))
        {
            kp[w] = wheel_kp;
            kd[w] = wheel_kd;
            target[w] = wheel_locked_pos[w];
        }

        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.q[i] = target[i];
            fsm_command->motor_command.dq[i] = 0.0f;
            fsm_command->motor_command.kp[i] = kp[i];
            fsm_command->motor_command.kd[i] = kd[i];
            fsm_command->motor_command.tau[i] = 0.0f;
        }

        std::ostringstream msg;
        msg << "Jitter in place amp:" << std::fixed << std::setprecision(2) << amp << " rad ("
            << std::setprecision(1) << (amp * LIFT_M_PER_RAD * 100.0f) << " cm) freq:" << freq << " Hz";
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << msg.str() << std::flush;
    }

    void Exit() override {}

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
        else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        {
            return "RLFSMStateStepInPlace";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateStepTurn";
        }
        return state_name_;
    }
};

// Mark-time ("原地踏步"): the legs lift and plant one after another while the
// body stays put. The wheels are velocity joints, so they are braked
// (kp = 0, dq = 0) and the swing leg is the only contact that leaves the ground.
//
// The lift is a pure vertical wheel motion: solving the leg IK around the
// standing pose gives  dthigh = -s, dcalf = +2s  for the front legs and the
// mirrored values for the hind legs (0.203 m of clearance per rad of s).
// Keyboard '3' = step on the spot (wheels held, no drift),
// keyboard '4' = the same gait while skid-steering the body around its own axis.
class RLFSMStateStepInPlace : public RLFSMState
{
public:
    RLFSMStateStepInPlace(RL *rl) : RLFSMState(*rl, "RLFSMStateStepInPlace") {}

    float percent_transition = 0.0f;
    float gait_time = 0.0f;
    std::vector<float> wheel_locked_pos;

    void Enter() override
    {
        percent_transition = 0.0f;
        gait_time = 0.0f;
        rl.now_state = *fsm_state;
        // remember where the wheels are, so they can be held there (= parking brake)
        wheel_locked_pos = rl.now_state.motor_state.q;
        std::cout << LOGGER::NOTE << "Entered step-in-place mode (legs lift, body stays). '4' step+turn, '2' jitter, '1' RL, '0' GetUp, '9' GetDown, 'P' passive." << std::endl;
    }

    void Run() override
    {
        // first blend from the pose we entered with into the standing pose
        if (Interpolate(percent_transition, rl.now_state.motor_state.q,
                        rl.params.Get<std::vector<float>>("default_dof_pos"),
                        rl.params.Get<float>("step_transition", 0.5f),
                        "Step in place", true))
        {
            return;
        }

        gait_time += rl.params.Get<float>("dt");

        // wheels are held at the angle they had on entry: the body cannot drift
        const float hold[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        std::vector<float> target;
        int swing_leg = -1;
        float swing_peak = 0.0f;
        bool trot = true;
        BuildStepTargets(rl, gait_time, wheel_locked_pos, hold, 0.0f, target, swing_leg, swing_peak, trot);
        WriteStepCommand(rl, fsm_command, target);

        const float lift = rl.params.Get<float>("step_lift", 0.45f);
        const float stance_drop = rl.params.Get<float>("step_stance_drop", 0.10f);
        const float period = rl.params.Get<float>("step_period", 1.0f);

        static const char *leg_name[4] = {"fl", "fr", "hl", "hr"};
        std::string up_label = "-";
        if (swing_leg >= 0)
        {
            up_label = trot ? ((swing_leg == 0 || swing_leg == 3) ? "fl+hr" : "fr+hl")
                            : leg_name[swing_leg];
        }
        std::ostringstream msg;
        msg << "Step in place [" << (trot ? "trot" : "crawl") << "] clearance:" << std::fixed
            << std::setprecision(1) << ((lift + stance_drop) * LIFT_M_PER_RAD * 100.0f) << " cm  up:"
            << up_label << " (" << std::setprecision(1)
            << ((swing_peak + stance_drop) * LIFT_M_PER_RAD * 100.0f) << " cm now)  period:" << period << " s";
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << msg.str() << std::flush;
    }

    void Exit() override {}

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
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
        {
            return "RLFSMStateJitter";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateStepTurn";
        }
        return state_name_;
    }
};

// Mark-time while turning on the spot ("原地踏步转圈"), keyboard '4'.
// Identical gait to RLFSMStateStepInPlace, but instead of holding the wheels the
// two sides are driven against each other (skid steer): left wheels +w, right
// wheels -w. The net force cancels, the yaw torque does not, so the body spins
// about its own axis. With wheel radius 0.11 m and half track 0.22 m a body yaw
// rate w needs a wheel rate of w * 0.22 / 0.11 = 2w.
class RLFSMStateStepTurn : public RLFSMState
{
public:
    RLFSMStateStepTurn(RL *rl) : RLFSMState(*rl, "RLFSMStateStepTurn") {}

    float percent_transition = 0.0f;
    float gait_time = 0.0f;
    float wheel_spin = 0.0f;        // integrated wheel rotation, rad
    std::vector<float> wheel_ref_pos;

    void Enter() override
    {
        percent_transition = 0.0f;
        gait_time = 0.0f;
        wheel_spin = 0.0f;
        rl.now_state = *fsm_state;
        wheel_ref_pos = rl.now_state.motor_state.q;
        std::cout << LOGGER::NOTE << "Entered step-turn mode (step in place while spinning). '3' step only, '2' jitter, '1' RL, '0' GetUp, '9' GetDown, 'P' passive." << std::endl;
    }

    void Run() override
    {
        if (Interpolate(percent_transition, rl.now_state.motor_state.q,
                        rl.params.Get<std::vector<float>>("default_dof_pos"),
                        rl.params.Get<float>("step_transition", 0.5f),
                        "Step turn", true))
        {
            return;
        }

        const float dt = rl.params.Get<float>("dt");
        gait_time += dt;

        const float yaw_rate = rl.params.Get<float>("step_yaw_rate", 0.6f);          // rad/s, + = CCW
        const float half_track = rl.params.Get<float>("step_turn_half_track", HALF_TRACK);
        const float radius = rl.params.Get<float>("step_turn_wheel_radius", WHEEL_RADIUS);
        float wheel_omega = rl.params.Get<float>("step_turn_wheel_omega", 0.0f);     // >0 overrides the geometry
        if (wheel_omega == 0.0f)
        {
            wheel_omega = yaw_rate * half_track / radius;
        }
        wheel_spin += wheel_omega * dt;

        // a positive wheel angle drives the wheel backwards, so left + / right -
        // means left back, right forward = counter-clockwise (yaw +)
        const float dir[4] = {+1.0f, -1.0f, +1.0f, -1.0f};   // fl, fr, hl, hr
        std::vector<float> target;
        int swing_leg = -1;
        float swing_peak = 0.0f;
        bool trot = true;
        BuildStepTargets(rl, gait_time, wheel_ref_pos, dir, wheel_spin, target, swing_leg, swing_peak, trot);
        WriteStepCommand(rl, fsm_command, target);

        const float lift = rl.params.Get<float>("step_lift", 0.45f);
        const float stance_drop = rl.params.Get<float>("step_stance_drop", 0.10f);
        std::string up_label = "-";
        if (swing_leg >= 0)
        {
            up_label = trot ? ((swing_leg == 0 || swing_leg == 3) ? "fl+hr" : "fr+hl") : "-";
        }
        std::ostringstream msg;
        msg << "Step turn [" << (trot ? "trot" : "crawl") << "] clearance:" << std::fixed
            << std::setprecision(1) << ((lift + stance_drop) * LIFT_M_PER_RAD * 100.0f) << " cm  up:"
            << up_label << "  yaw:" << std::setprecision(2) << yaw_rate << " rad/s (wheel "
            << wheel_omega << " rad/s)";
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << msg.str() << std::flush;
    }

    void Exit() override {}

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
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
        {
            return "RLFSMStateJitter";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        {
            return "RLFSMStateStepInPlace";
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        // 2.5s：下蹲 0.29 m（0.537→0.250）需要足够慢，快了触地冲击会把机身弹歪。
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.5f, "Getting down", true);
    }

    void Exit() override {}

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

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

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
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
        {
            return "RLFSMStateJitter";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        {
            return "RLFSMStateStepInPlace";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateStepTurn";
        }
        return state_name_;
    }
};

} // namespace ancnW1_fsm

//状态工厂, 把状态名映射到具体的状态类
class AncnW1FSMFactory : public FSMFactory
{
public:
    AncnW1FSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<ancnW1_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<ancnW1_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<ancnW1_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateJitter")
            return std::make_shared<ancnW1_fsm::RLFSMStateJitter>(rl);
        else if (state_name == "RLFSMStateStepInPlace")
            return std::make_shared<ancnW1_fsm::RLFSMStateStepInPlace>(rl);
        else if (state_name == "RLFSMStateStepTurn")
            return std::make_shared<ancnW1_fsm::RLFSMStateStepTurn>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<ancnW1_fsm::RLFSMStateRLLocomotion>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "ancnW1"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateJitter",
            "RLFSMStateStepInPlace",
            "RLFSMStateStepTurn",
            "RLFSMStateRLLocomotion"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

//注册到全局 FSM 工厂表
REGISTER_FSM_FACTORY(AncnW1FSMFactory, "RLFSMStatePassive")

#endif // ANCNW1_FSM_HPP
