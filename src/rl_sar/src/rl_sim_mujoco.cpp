/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim_mujoco.hpp"

RL_Sim* RL_Sim::instance = nullptr;

RL_Sim::RL_Sim(int argc, char **argv)
{
    // Set static instance pointer early for signal handler
    instance = this;

    if (argc < 3)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " robot_name scene_name" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }
    else
    {
        this->robot_name = argv[1];
        this->scene_name = argv[2];
    }

    this->ang_vel_axis = "body";

    // now launch mujoco
    std::cout << LOGGER::INFO << "[MuJoCo] Launching..." << std::endl;

    // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
    if (rosetta_error_msg)
    {
        DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
        std::exit(1);
    }
#endif

    // print version, check compatibility
    std::cout << LOGGER::INFO << "[MuJoCo] Version: " << mj_versionString() << std::endl;
    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }

    // scan for libraries in the plugin directory to load additional plugins
    scanPluginLibraries();

    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    // simulate object encapsulates the UI
    sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

    std::string filename = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/../rl_sar_zoo/" + this->robot_name + "_description/mjcf/" + this->scene_name + ".xml";

    // start physics thread
    std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename.c_str());
    physicsthreadhandle.detach();

    while (1)
    {
        if (d)
        {
            std::cout << LOGGER::INFO << "[MuJoCo] Data prepared" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    this->mj_model = m;
    this->mj_data = d;

    // 若模型定义了 keyframe（如站立姿态），出生即复位到第一个 keyframe。
    // MuJoCo 默认初始姿态是"关节全零 + body 定义位置"，某些构型（如轮腿）
    // 零位落地会翻倒。没有 keyframe 的模型完全不受影响。
    if (this->mj_model && this->mj_data && this->mj_model->nkey > 0)
    {
        mj_resetDataKeyframe(this->mj_model, this->mj_data, 0);
        mj_forward(this->mj_model, this->mj_data);
        std::cout << LOGGER::INFO << "Reset to keyframe[0] \""
                  << (mj_id2name(this->mj_model, mjOBJ_KEY, 0) ? mj_id2name(this->mj_model, mjOBJ_KEY, 0) : "?")
                  << "\", base z = " << this->mj_data->qpos[2] << std::endl;
    }

    this->SetupSysJoystick("/dev/input/js0", 16); // 16 bits joystick

    // read params from yaml
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
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
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

    // joystick
    this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.01, std::bind(&RL_Sim::GetSysJoystick, this));
    this->loop_joystick->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;

    // start simulation UI loop (blocking call)
    sim->RenderLoop();
}

RL_Sim::~RL_Sim()
{
    // Clear static instance pointer
    instance = nullptr;

    this->loop_keyboard->shutdown();
    this->loop_joystick->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::GetState(RobotState<float> *state)
{
    if (mj_data)
    {
        state->imu.quaternion[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 0];
        state->imu.quaternion[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 1];
        state->imu.quaternion[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 2];
        state->imu.quaternion[3] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 3];

        state->imu.gyroscope[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 4];
        state->imu.gyroscope[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 5];
        state->imu.gyroscope[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 6];

        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            state->motor_state.q[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i]];
            state->motor_state.dq[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + this->params.Get<int>("num_of_dofs")];
            state->motor_state.tau_est[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + 2 * this->params.Get<int>("num_of_dofs")];
        }
    }
}

void RL_Sim::SetCommand(const RobotCommand<float> *command)
{
    if (mj_data)
    {
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            mj_data->ctrl[this->params.Get<std::vector<int>>("joint_mapping")[i]] =
                command->motor_command.tau[i] +
                command->motor_command.kp[i] * (command->motor_command.q[i] - mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i]]) +
                command->motor_command.kd[i] * (command->motor_command.dq[i] - mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + this->params.Get<int>("num_of_dofs")]);
        }
    }
}

void RL_Sim::RobotControl()
{
    // Lock the sim mutex once for the entire control cycle to prevent race conditions
    const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

    // 地形几何体位于 group 3（专供 height_scan 射线过滤）。渲染线程的
    // RenderLoop() 启动时会调用 mjv_defaultOption() 把渲染选项整个重置
    // （group 3 默认不可见），构造前对 mjvOption 所做的修改会被覆盖。
    // 因此必须在并行运行的控制周期里持续打开该组，地形才能在窗口中显示。
    sim->opt.geomgroup[3] = 1;

    // --------------------------------------- 第三人称跟随视角（一直保持 Free 相机，按 Z 开关）
    //
    // 【为什么不再用 cam.type = Tracking】：simulate 的 UI 会【根据 cam.type 反向把 Camera
    // 下拉框刷回去】—— 即 mjui 那边写着 "type==TRACKING ⇒ camera=1"。于是只要代码侧每个
    // 控制周期写一次 type，就等于和用户点下拉框互抢，用户点 Free 后立刻被顶回 Tracking，
    // 表现为"面板切 Free 切不过来"。
    //
    // 【现在的做法】：相机始终保持 Free 类型，"跟拍"效果完全由本文件自己更新 lookat /
    // azimuth 来模拟。面板上的 Camera 下拉框因此永远是 Free，没有任何代码去改写它，用户
    // 随时可以用鼠标自由旋转/平移/缩放；按 Z 关闭跟拍，镜头就在原地悬停不再追。
    if (this->mj_model && this->mj_data)
    {
        static const char *kFollowBody = "base_link";   // M20 机身刚体名
        static int follow_body_id = -2;
        static bool follow_enabled = true;              // 默认开启跟拍
        static bool follow_ready = false;               // 首帧是否已摆好机位
        static int last_key_code = -1;

        if (follow_body_id == -2)
        {
            follow_body_id = mj_name2id(this->mj_model, mjOBJ_BODY, kFollowBody);
            if (follow_body_id < 0)
            {
                follow_enabled = false;
                std::cout << LOGGER::WARNING << "Camera: 未找到刚体 \"" << kFollowBody
                          << "\"，跟随视角不可用" << std::endl;
            }
        }

        // 用户若在面板里显式选了 Tracking，就把镜头控制权完整让给 mjui，不再插手
        if (follow_enabled && sim->camera == 1)
        {
            follow_enabled = false;
            follow_ready = false;
        }

        // 按 Z 开关跟拍（边沿触发：按一次只翻一次，长按不会连续抖动）
        int key_code = static_cast<int>(this->control.current_keyboard);
        if (key_code == static_cast<int>(Input::Keyboard::Z) && key_code != last_key_code)
        {
            follow_enabled = !follow_enabled;
            follow_ready = false;                       // 重新摆一次机位
            std::cout << LOGGER::INFO << "Camera follow: "
                      << (follow_enabled ? "ON（再按 Z 关闭）"
                                         : "OFF（镜头已悬停，交还给你：左键旋转 / 右键平移 / 滚轮缩放；面板 Camera 保持 Free）")
                      << std::endl;
        }
        last_key_code = key_code;

        if (follow_enabled && follow_body_id >= 0)
        {
            const mjtNum *p = this->mj_data->xpos + 3 * follow_body_id;   // 机身世界坐标
            const mjtNum *q = this->mj_data->xquat + 4 * follow_body_id;  // 机身姿态四元数
            double yaw = atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                               1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));

            sim->cam.type = mjCAMERA_FREE;   // 关键：永远是 Free，下拉框不会被抢
            sim->camera = 0;

            if (!follow_ready)               // distance / elevation 只在这里摆一次，
            {                                // 之后交给用户滚轮调整，不再强行覆盖
                sim->cam.distance = 1.9;     // 机位离机身约 1.9 m
                sim->cam.elevation = -14.0;  // 略高的俯视角，能看见地形起伏
                sim->cam.lookat[0] = p[0];
                sim->cam.lookat[1] = p[1];
                sim->cam.lookat[2] = p[2] + 0.15;
                follow_ready = true;
                std::cout << LOGGER::INFO
                          << "Camera: 第三人称跟拍已开启（面板 Camera 始终为 Free，可自由拖动）；按 Z 关闭"
                          << std::endl;
            }

            // lookat 做一阶低通后再跟随，避免机身的高频抖动直接传导到画面
            double tx[3] = {p[0], p[1], p[2] + 0.15};
            for (int i = 0; i < 3; ++i)
            {
                sim->cam.lookat[i] += 0.25 * (tx[i] - sim->cam.lookat[i]);
            }
            sim->cam.azimuth = yaw * 180.0 / 3.14159265358979 + 170.0;    // 170°≈机身正后方
        }
    }

    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        if (this->mj_model && this->mj_data)
        {
            if (this->mj_model->nkey > 0)
            {
                mj_resetDataKeyframe(this->mj_model, this->mj_data, 0);
            }
            else
            {
                mj_resetData(this->mj_model, this->mj_data);
            }
            mj_forward(this->mj_model, this->mj_data);
        }
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
            sim->run = 0;
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            sim->run = 1;
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

void RL_Sim::SetupSysJoystick(const std::string& device, int bits)
{
    this->sys_js = std::make_unique<Joystick>(device);
    if (!this->sys_js->isFound())
    {
        std::cout << LOGGER::ERROR << "Joystick [" << device << "] open failed." << std::endl;
        // exit(1);
    }

    this->sys_js_max_value = (1 << (bits - 1));
}

void RL_Sim::GetSysJoystick()
{
    // Clear all button event states
    for (int i = 0; i < 20; ++i)
    {
        this->sys_js_button[i].on_press = false;
        this->sys_js_button[i].on_release = false;
    }

    // Check if joystick is valid before using
    if (!this->sys_js)
    {
        return;
    }

    while (this->sys_js->sample(&this->sys_js_event))
    {
        if (this->sys_js_event.isButton())
        {
            this->sys_js_button[this->sys_js_event.number].update(this->sys_js_event.value);
        }
        else if (this->sys_js_event.isAxis())
        {
            double normalized = double(this->sys_js_event.value) / this->sys_js_max_value;
            if (std::abs(normalized) < this->axis_deadzone)
            {
                this->sys_js_axis[this->sys_js_event.number] = 0;
            }
            else
            {
                this->sys_js_axis[this->sys_js_event.number] = this->sys_js_event.value;
            }
        }
    }

    if (this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::A);
    if (this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::B);
    if (this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::X);
    if (this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->sys_js_button[4].on_press) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->sys_js_button[4].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->sys_js_button[4].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->sys_js_button[4].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->sys_js_button[4].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->sys_js_button[4].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->sys_js_button[5].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->sys_js_button[5].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->sys_js_button[5].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->sys_js_button[5].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->sys_js_button[5].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->sys_js_button[5].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->sys_js_button[4].pressed && this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::LB_RB);

    float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    if (has_input)
    {
        this->control.x = ly;
        this->control.y = lx;
        this->control.yaw = rx;
        this->sys_js_active = true;
    }
    else if (this->sys_js_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->sys_js_active = false;
    }
}

void RL_Sim::UpdateHeightScan()
{
    if (this->obs.height_scan.empty() || !this->mj_model || !this->mj_data)
    {
        return;
    }

    const int grid_x = this->params.Get<int>("height_scan_grid_x", 17);
    const int grid_y = this->params.Get<int>("height_scan_grid_y", 11);
    const float step = this->params.Get<float>("height_scan_resolution", 0.1f);
    const float range = this->params.Get<float>("height_scan_range", 2.0f);
    const int group = this->params.Get<int>("height_scan_geom_group", 3);
    // 命中点世界高度的额外偏移（米）。默认 0 = 直接输出世界高度（平地约 0）。
    // 若日后确认策略是以基座为基准训练的，把它设成「基座站立高度」即可切换回去。
    const float height_scan_ref = this->params.Get<float>("height_scan_ref_offset", 0.0f);

    // 与 IsaacLab 的 RayCaster(attach_yaw_only=True) 对齐：
    // 网格只跟随基座的偏航角，射线永远铅垂向下，不受俯仰/滚转的影响。
    int base_id = mj_name2id(this->mj_model, mjOBJ_BODY, "base_link");
    if (base_id < 0)
    {
        if (!height_scan_warned)
        {
            height_scan_warned = true;
            std::cout << LOGGER::WARNING << "[HeightScan] 模型中找不到 body \"base_link\"，已跳过地形扫描" << std::endl;
        }
        return;
    }

    const mjtNum *pos = this->mj_data->xpos + 3 * base_id;
    const mjtNum *xmat = this->mj_data->xmat + 9 * base_id;
    // xmat 行优先，第一列即机体系 x 轴在世界系下的方向，据此取偏航角
    float yaw = atan2f((float)xmat[3], (float)xmat[0]);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    // 只对地形所在的 group 发射射线，避免打到机器人自己的腿脚
    mjtByte geomgroup[mjNGROUP] = {0};
    int group_idx = (group >= 0 && group < mjNGROUP) ? group : 0;
    geomgroup[group_idx] = 1;

    int missed = 0;
    size_t k = 0;
    for (int ix = 0; ix < grid_x; ++ix)
    {
        float bx = (ix - (grid_x - 1) * 0.5f) * step;
        for (int iy = 0; iy < grid_y; ++iy)
        {
            float by = (iy - (grid_y - 1) * 0.5f) * step;
            mjtNum pnt[3] = {
                pos[0] + (mjtNum)(cy * bx - sy * by),
                pos[1] + (mjtNum)(sy * bx + cy * by),
                pos[2]
            };
            mjtNum vec[3] = {0.0, 0.0, -(mjtNum)range};
            int hit_geomid = -1;
            mjtNum dist = mj_ray(this->mj_model, this->mj_data, pnt, vec, geomgroup, 1, -1, &hit_geomid);
            if (dist >= 0.0)
            {
                // ① mj_ray 的返回值是按「射线向量长度」归一化的，不是米数。
                //    实测：vec 长 1.0 返回 0.5900，长 2.0 返回 0.2950，长 4.0 返回 0.1475，
                //    即 返回值 = 真实距离 / |vec|。这里 |vec| = range，故须乘回 range。
                const double real_dist = (double)dist * (double)range;
                // ② 参考基准对齐 IsaacLab：mdp.height_scan 默认 offset=0，返回的是
                //    命中点的世界高度（平地约 0）。若改用「基座到地面的距离」，
                //    平地会变成 0.59，等于给 187 维输入整体加了一个训练时从未出现的
                //    常量偏置，策略会持续输出错误动作。
                const double hit_z = (double)pos[2] - real_dist;
                this->obs.height_scan[k] = (float)(hit_z - height_scan_ref);
            }
            else
            {
                // 未命中（地形不在该 group，或该点悬空）：按「地形极高」处理，clip 后到上限
                this->obs.height_scan[k] = (float)((double)range - height_scan_ref);
                ++missed;
            }
            ++k;
        }
    }

    // 首次采样后打印一次统计，用于快速核对数值是否落在合理范围：
    // 平地上应接近 0（世界高度基准）；若全是射程上限说明射线打空了。
    if (!height_scan_warned)
    {
        height_scan_warned = true;
        float hs_min = this->obs.height_scan[0];
        float hs_max = this->obs.height_scan[0];
        float hs_sum = 0.0f;
        for (float v : this->obs.height_scan)
        {
            if (v < hs_min)
            {
                hs_min = v;
            }
            if (v > hs_max)
            {
                hs_max = v;
            }
            hs_sum += v;
        }
        std::cout << LOGGER::INFO << "[HeightScan] 地形扫描已启用：点数 " << this->obs.height_scan.size()
                  << "，最小 " << hs_min << " m / 最大 " << hs_max << " m / 均值 "
                  << (hs_sum / this->obs.height_scan.size()) << " m，未命中 " << missed << " 个" << std::endl;
        if (missed == (int)this->obs.height_scan.size())
        {
            std::cout << LOGGER::WARNING << "[HeightScan] 全部采样点都未命中地形（geom group = " << group
                      << "）。请确认场景中地形 geom 的 group 与 height_scan_geom_group 一致，"
                      << "否则喂给模型的一律是射程上限。" << std::endl;
        }
    }
}

void RL_Sim::RunModel()
{
    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.commands[0] = this->control.x;
        this->obs.commands[1] = this->control.y;
        this->obs.commands[2] = this->control.yaw;
        //not currently available for non-ros mujoco version
        // if (this->control.navigation_mode)
        // {
        //     this->obs.commands[0] = (float)this->cmd_vel.linear.x;
        //     this->obs.commands[1] = (float)this->cmd_vel.linear.y;
        //     this->obs.commands[2] = (float)this->cmd_vel.angular.z;
        // }
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;
        this->UpdateHeightScan();

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est(this->params.Get<int>("num_of_dofs"), 0.0f);
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]];
        }
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Sim::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(mj_data->sensordata[i]);
        // this->plot_target_joint_pos[i].push_back();  // TODO
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}

// Signal handler for Ctrl+C
void signalHandler(int signum)
{
    std::cout << LOGGER::INFO << "Received signal " << signum << ", exiting..." << std::endl;
    if (RL_Sim::instance && RL_Sim::instance->sim)
    {
        RL_Sim::instance->sim->exitrequest.store(1);
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);
    RL_Sim rl_sar(argc, argv);
    return 0;
}
