# ancnW1 真机 CAN-FD 电机驱动

基于**智身新创关节模组使用手册（CANFD 版）V1.2** 实现的 ancnW1 真机驱动。

> ⚠️ **当前状态：代码已编译通过，但从未在真实硬件上运行过。**
> 协议框架、字节序、映射关系均已按手册实现；**电机型号阈值、CANID 分配、反馈寻址等必须先跟厂商确认**（见 [§6](#6-待厂商确认的阻塞项)），否则通电有风险。

---

## 1. 文件清单

**① 充当厂商 SDK 的三层**（零 rl_sar 依赖，可整包抽走 —— 详见 [§9](#9-sdk-归属哪几个文件在充当厂商-sdk)）

| 文件 | 行数 | 职责 |
|---|:--:|---|
| `include/ancnW1_can_interface.hpp` | 175 | CAN-FD socketcan 接口定义 |
| `src/ancnW1_can_interface.cpp` | 427 | socketcan 收发、异步接收线程、反馈缓存 |
| `include/ancnW1_motor_driver.hpp` | 182 | 电机协议封装（映射/组帧/解析） |
| `src/ancnW1_motor_driver.cpp` | 304 | `float↔uint16` 映射、64 字节帧组装、12 字节反馈解析 |
| `include/ancnW1_motor_fsm.hpp` | 175 | 电机初始化状态机接口 |
| `src/ancnW1_motor_fsm.cpp` | 485 | 使能/失能/设零位序列、故障检测 |
| **小计** | **1748** | |

**② rl_sar 适配层**（对标 `rl_real_go2.cpp`）

| 文件 | 行数 | 职责 |
|---|:--:|---|
| `include/rl_real_ancnW1.hpp` | 120 | 真机节点接口 |
| `src/rl_real_ancnW1.cpp` | 594 | 装配层：`GetState` / `SetCommand` / `RobotControl` |
| `fsm_robot/fsm_ancnW1.hpp` | 482 | RLFSM 4 个状态（Passive/GetUp/GetDown/RL） |
| **小计** | **1196** | |

**③ 已搁置**（未参与编译，见 [§10](#10-tcp-桥接-ancnw1_tcp_bridge上位机对接)）

| 文件 | 行数 | 职责 |
|---|:--:|---|
| `include/ancnW1_tcp_bridge_protocol.hpp` | 498 | 上位机 TCP 通信协议（孤儿文件） |

**④ 构建改动**

| 文件 | 改动 | 说明 |
|---|:--:|---|
| `CMakeLists.txt` | +27 | 新增 `rl_real_ancnW1` 构建目标 |
| `fsm_robot/fsm_all.hpp` | +1 | 注册 ancnW1 状态机 |

**合计新增约 3442 行**（含详细中文注释，各文件注释密度 38%~52%）。

---

## 2. 分层架构

```
┌──────────────────────────────────────────────────────────┐
│  RL 框架（已有，不改）                                      │
│  RLFSM → StateController → ComputeOutput → robot_command  │
│    base.yaml / robot_lab/config.yaml                      │
└────────────────────────┬─────────────────────────────────┘
                         │ GetState() ↑        SetCommand() ↓
┌────────────────────────┴─────────────────────────────────┐
│  RL_Real_AncnW1  （装配层）                                │
│   · 建立三个模块的依赖                                     │
│   · joint_mapping 重排：policy 序 ⇄ physical 序            │
│   · IMU 数据注入 RobotState                                │
└──────┬─────────────────┬──────────────────┬──────────────┘
       │                 │                  │
┌──────┴──────┐  ┌───────┴────────┐  ┌──────┴──────────────┐
│MotorFsm     │  │MotorDriver     │  │CanInterface         │
│状态机        │  │协议封装         │  │socketcan 收发       │
│             │→ │                │→ │                     │
│Disabled     │  │· 映射函数       │  │· 打开/配置 5Mbps FD│
│Enabling     │  │· 组 64B 帧     │  │· Send(64B)         │
│Enabled      │  │· 解 12B 反馈   │  │· 后台接收线程       │
│Disabling    │  │· 型号阈值表     │  │· per-motor 缓存    │
│Zeroing      │  │                │  │                     │
│Fault        │  │                │  │                     │
└─────────────┘  └────────────────┘  └─────────────────────┘
```

**线程模型**

| 线程 | 频率 | 做什么 |
|:--:|---|
| 主线程 `loop_keyboard` | 20 Hz | 键盘输入 |
| 主线程 `loop_control` | 200 Hz (`dt=0.005`) | `Tick` → `GetState` → `StateController` → `SetCommand` |
| 主线程 `loop_rl` | 50 Hz (`dt×decimation=4`) | 观测 → 推理 → `ComputeOutput` |
| CanInterface 后台线程 | 持续 | 收 12B 反馈 → 写入 per-motor 缓存 |

---

## 3. 协议要点（手册 → 代码实现）

| 手册项 | 数值 | 代码位置 |
|---|---|---|
| 仲裁段波特率 | 1 Mbps | `CanInterface::Open` 注释（需用户态 `ip link` 配置） |
| 数据段波特率 | 5 Mbps | 同上，`CANFD_BRS` 置位 |
| 帧格式 | 标准帧 | `can_id & CAN_SFF_MASK` |
| 下发帧长 | 64 字节（6 电机槽） | `CanInterface::kMaxFrameLen` |
| 反馈帧长 | 12 字节（**单电机**） | `CanInterface::kFeedbackLen` |
| 主站 CANID | `0x00` | `can_->Send(0x00, ...)` |
| 从站 CANID | `0x01`~`0x06`（**每帧槽位**） | ⚠️ 见 §6 Q2 |
| 字段字节序 | 字段内**小端** | 所有 `*(元素 & 0xFF)` → `(元素 >> 8) & 0xFF` |

**运行指令槽位布局**（手册 §3.2.2，`BuildRunFrame`）

```
Byte0-1    MSGID（低字节在前）
Byte2-11   第1槽： 位置 | 速度 | Kp | Kd | τ    ← 每个字段 uint16 小端
Byte12-21  第2槽
Byte22-31  第3槽
Byte32-41  第4槽
Byte42-51  第5槽
Byte52-61  第6槽
Byte62-63  保留
```

**特殊指令**（`BuildEnableFrame` / `Disable` / `Zero`）

| 指令 | Byte2-10 | Byte11 |
|---|---|---|
| 使能 | `0xFF × 9` | `0xFC` |
| 失能 | `0xFF × 9` | `0xFD` |
| 设机械零位 | `0xFF × 9` | `0xEF` |

---

## 4. 关键设计决策

### 4.1 ⚠️ 绝对不要把电机 Kp 清零

这是最容易踩的坑，`config.yaml` 的注释已经点明：

> rl_sar applies the position term twice (`output_dof_tau` + `SetCommand`'s `kp*(cmd_q-q)`),
> so the effective stiffness/damping are **2x** the values below:
> legs `40/1.0` here == **Isaac stiffness 80 / damping 2.0**

按手册 §3.1 阻抗模型展开：

```
电机力矩 = Kp*(q_cmd - q_fb) + Kd*(dq_cmd - dq_fb) + τ_ff
```

| 方案 | 代入 | 等效刚度 | 结果 |
|---|---|---|---|
| **透传** `Kp=40,Kd=1.0,τ_ff=40Δp-1.0·v` | `40Δp + 1.0(0-v) + (40Δp - 1.0v)` = **80Δp - 2.0v** | **80 / 2.0** | ✅ 与训练一致 |
| 清零 `Kp=0,Kd=0` | `0 + 0 + (40Δp - 1.0v)` | 40 / 1.0 | ❌ 刚度腰斩，站不起来 |

所以 `AssembleRunFrames()` 里 **`motor_command.kp/kd` 原样透传**，代码中有醒目注释标注勿改。

### 4.2 轮子自动满足"禁止位置控制"

手册 §2.6.1.1 注：**轮毂电机不支持位置控制**。

好消息是无需额外处理——`config.yaml` 里 `rl_kp` 的最后 4 项本来就是 `0`：

```yaml
rl_kp: [40×12, 0.0, 0.0, 0.0, 0.0]   # 轮 = 0 ✓
rl_kd: [1.0×12, 0.6, 0.6, 0.6, 0.6]  # 轮 = 0.6，速度伺服
```

透传后轮子自动进入纯速度模式（`Kp=0, dq=target, Kd=0.6`），符合手册 §1.1.2.2。

> ⚠️ **唯一例外**：`base.yaml` 的 `step_wheel_kp: 15.0` 会在原地踏步模式（按键 `3`）给轮子加位置刚度，**违反协议**。
> 真机部署前必须改成 `0.0`，改用纯阻尼 `step_wheel_kd: 2.0`。

### 4.3 policy ⇄ physical ⇄ motor_id 三者关系

```yaml
joint_mapping: [0,1,2,4,5,6,8,9,10,12,13,14,3,7,11,15]
wheel_indices: [12,13,14,15]     # 注意是 policy index
```

含义与推算规则：

```
motor_id = joint_mapping[policy_idx] + 1
```

| 语义 | policy idx | physical idx | motor_id |
|---|---:|---:|---:|
| `fl_hip` | 0 | 0 | 1 |
| `fl_wheel` | 12 | 3 | **4** |
| `fr_hip` | 3 | 4 | 5 |
| `hr_wheel` | 15 | 15 | 16 |

注意 **轮子不是最后的 motor 13~16**，而是 **4 / 8 / 12 / 16**（交错分布）。`GetState` 和 `SetCommand` 都按这个表做双向重排。

### 4.4 反馈解析的阈值（TODO）

`MotorDriver::ParseFeedback()` 目前用**硬编码的通用阈值**（±12.5 / ±65 / ±30）。
原因：12 字节反馈帧里**不含电机 ID**，而速度/力矩阈值因型号而异（如腿 P85MAX-S 力矩 ±300、轮 W190-S 速度 ±200）。

**BOM 确认后必须按 `motor_id` 取对应 `MotorTypeConfig` 重新解析**，否则轮子的速度反馈会被严重截断。

### 4.5 MSGID 字节序有歧义

手册示例写 `MSGID 00 01`，但同时在别处声明"低字节在前"。

- 若 `00 01` 是字面字节序 → MSGID = `0x0100`
- 若 MSGID = `0x0001` 按小端传输 → 字节应为 `01 00`

当前代码按 **MSGID=0x0001 → 字节 `[0x01, 0x00]`** 实现。
**MSGID 只用于帧回显匹配，不影响控制功能**，厂商确认后一行改动即可。

---

## 5. 编译与运行

```bash
# 编译（CMake 模式，仅新增 ancnW1 目标）
./build.sh -m rl_real_ancnW1
# 产物：cmake_build/bin/rl_real_ancnW1
```

运行前配置 socketcan（需要 root / `CAP_NET_RAW`）：

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 txqueuelen 1000
```

确认链路：

```bash
ip -details link show can0      # 应看到 fd、dbitrate 5000000
candump -fd can0                # 应能看到 12 字节反馈帧刷屏
```

启动：

```bash
sudo ./cmake_build/bin/rl_real_ancnW1
# 可选指定接口：CAN_IFACE=can1 sudo ./cmake_build/bin/rl_real_ancnW1
```

终端按键沿用 rl_sar 统一约定：`0` 起立 / `1` RL 行走 / `9` 趴下 / `P` 急停。

---

## 6. 待厂商确认的阻塞项

### Q1. 关节型号 BOM

手册阈值表（§3.3.4）里**没有 ±113 的档位**，而 `config.yaml` 的 `torque_limits` 腿写的是 `113`（从 Isaac URDF 抄的）。

| 型号 | T_MAX | KD_MAX | V_MAX |
|---|---:|---:|---:|
| P55 / P65 / P65MAX-S | 30 / 50 / 100 | 5 | ±65 |
| P85MAX-S | 300 | **50** | ±65 |
| P100MAX-S | 650 | **50** | ±65 |
| W110-S / W150-S | 30 | 5 | ±300 |
| W190-S | 50 | 5 | ±200 |

**必须问**：12 个腿关节 + 4 个轮子分别什么型号？
拿到后马上改两处：`ConfigureMotorTypes()` 的阈值表、`ParseFeedback()` 的解析阈值。

> 在 BOM 确认前**不要通电**：限幅大于电机实际能力会触发过流故障（手册 bit2），有损坏风险。

### Q2. 🔴 16 个电机如何寻址（最高优先级）

**这是当前最大的架构不确定性。**

手册 §3.2.2 规定单帧最多 **6 个槽位**，且帧内**没有任何字段标识"这是第几组"**。但 ancnW1 有 **16 个电机**。

`MotorFsm::SendCommandFrames()` 已实现"每 6 个一组 + MSGID 递增"的发送逻辑，
**但若 16 个电机挂在同一条 CAN 总线上，第 2、3 帧会因为字节布局完全相同而被同一批电机重复接收**——电机 7~16 收不到使能/失能指令。

需要厂商答复二选一：

1. **拆多条 CAN 总线**（如 6+6+4，每条独立 socket）→ 需要改 `CanInterface` 支持多接口
2. **提供 CANID 重映射机制**，让 16 个电机分时占用 1~6 槽位 → 需要额外指令

### Q3. CANID 重设方式

出厂默认全部是 `0x03`，手册**没给出重设指令**。需确认走上位机配置工具还是别的协议。

### Q4. 反馈帧的 CAN ID

手册只说反馈是 12 字节，没说用什么 CAN ID：

- 若是**电机自身 ID** → 16 个不同 ID 的反馈帧
- 若复用下发帧 ID → 负载小得多

当前 `CanInterface::ReceiveLoop()` 按 **"CAN ID 即电机 ID"** 解析，待确认。

### Q5. 反馈频率与总线带宽

```
下发：3 帧/周期 × 64B × 500Hz      ≈ 1.0 Mbps
反馈：16 电机 × 1kHz × ~30B/帧     ≈ 3.8 Mbps
合计                               ≈ 4.8 Mbps  ← 5 Mbps 几乎打满
```

建议把反馈频率降到 **500 Hz**（降到 ~2.4 Mbps，留一半裕量给重传/突发）。

### Q6. 通信超时保护

手册未提及：指令中断多久后电机自动停止？
**这决定要不要额外加 MCU 看门狗。** 若电机无内置超时保护，主机卡死时电机会保持最后的力矩指令——有失控风险。

---

## 7. 已知限制 / TODO

| 项 | 位置 | 说明 |
|---|---|---|
| **IMU 未接线** | `InitImu()` / `UpdateImuData()` | 桩实现返回单位四元数 + 零角速度。**观测里的 `ang_vel` 和 `gravity_vec` 全为 0，策略会立刻跑飞**。必须先把 IMU 接到 RK3588（SPI/I2C）并替换这两个函数 |
| 反馈阈值未按型号 | `ParseFeedback()` | 见 §4.4 |
| `torque_limits` 待改 | `policy/ancnW1/robot_lab/config.yaml` | 113 → 按 BOM 的真实 T_MAX |
| `step_wheel_kp` 待改 | `policy/ancnW1/base.yaml` | 15.0 → 0.0 |
| FSM 缺 3 个状态 | `fsm_ancnW1.hpp` | base.yaml 有 `jitter` / `step_in_place` / `step_turn` 配置，但 FSM 只实现了 4 个核心状态 |
| 轮子位置反馈跳变 | `GetState()` | 手册 §2.2：轮毂电机不支持绝对位置记忆，上电时 `q` 会跳变。当前策略不观测轮子位置（`action_scale` 轮予 fù 项已置 0），暂不影响，但字段仍填了 0 |
| 无硬件闭环验证 | 全部 | 从未在真电机上运行过，首次上电建议先**卸掉轮子、架空机身、限幅降到 10%** |

---

## 8. 参考

- 手册：`关节模组使用手册（CANFD版）V1.2`，智身新创（苏州）智能科技有限公司，2026.4.22
- 协议章节：§3.1 力矩模型 / §3.2 帧格式 / §3.3 指令集 / §3.3.4 型号阈值表
- 仿真对照：`policy/ancnW1/base.yaml`（关节映射注释详尽）、`policy/ancnW1/robot_lab/config.yaml`（RL 侧详细的 sim2real 说明）

---

## 9. SDK 归属：哪几个文件在充当「厂商 SDK」

### 9.1 背景

rl_sar 已适配的真机都依赖厂商 SDK，统一放在 `library/thirdparty/robot_sdk/`：

| 机器人 | SDK | 形态 | 适配层行数 |
|---|---|---|:--:|
| go2 / g1 | `unitree_sdk2` | 源码子目录 | 478 / 414 |
| a1 | `unitree_legged_sdk` | `.so` + include | 321 |
| l4w4 | `l4w4_sdk` | 纯头文件（INTERFACE） | 341 |
| Lite3 | `Lite3_MotionSDK` | `.so` | — |
| agibot D1 | `libmc_sdk_zsl_1` | `.so` | — |

它们的 `rl_real_*.cpp` **只做胶水**：拿 SDK 的类型收发，再转成 rl_sar 的 `RobotCommand` / `RobotState`。典型如 go2：

```cpp
this->lowcmd_publisher.reset(
    new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
```

ancnW1 **没有厂商 SDK**，所以 §1 分组①那 1748 行是我们自己写的替代品 —— 它们站在厂商 SDK 的位置上。

### 9.2 分层证据：单向依赖

```
═══════ rl_sar 侧（留下来） ═══════
  src/rl_real_ancnW1.cpp     594    ← 对标 rl_real_go2.cpp (478)
  fsm_robot/fsm_ancnW1.hpp   482    ← 对标 fsm_go2.hpp
        │  #include "rl_sdk.hpp" / "fsm.hpp"
        ↓ 单向调用
═══════ 充当 SDK（可整包抽走） ═══════
  ancnW1_motor_fsm           485    ↓ include driver
  ancnW1_motor_driver        304    ↓ include can_interface
  ancnW1_can_interface       427    ← 只有 <array> <cstdint> <thread>…
```

**关键点**：这三个 `.hpp` 里**没有一个 `#include "rl_sdk.hpp"`**，全部依赖只是 C++ 标准库。这是能抽离的硬依据。

### 9.3 逐层对应到厂商 SDK

| ancnW1 文件 | 行数 | 相当于厂商 SDK 的什么 | 厂商大概率会提供吗 |
|---|:--:|---|---|
| `ancnW1_can_interface` | 427 | 通信层：对应 unitree 的 `ChannelPublisher/Subscriber`、`InitChannel()`、异步接收线程 | 较可能仍需自写（要跑在 RK3588 的 socketcan 上） |
| `ancnW1_motor_driver` | 304 | 协议层：对应 `LowCmd_/LowState_` 结构体定义 + `Crc32Core()` + 单位换算 + **型号阈值表** | **这层最可能由厂商提供** |
| `ancnW1_motor_fsm` | 485 | 设备管理：enable / disable / setZero 流程 + 故障检测 | 宇树无此层（Go2 直发 cmd 即可）；工业关节 SDK 一般会有 |

对照用的数字：

> 有 SDK 的机器人，适配层是 **321~478 行**；
> ancnW1 的适配层 **594 行**，量级相当 —— 说明胶水层没臃肿；
> 多出来的 **1748 行**，正是本该厂商提供的部分。

### 9.4 ⚠️ 注意事项：维护 SDK 层的四条铁律

守住这几条，将来抽离不需重构；一旦破戒，这一层永久沦为 rl_sar 专属代码。

1. **不许 include rl_sar 的任何头文件**
   禁止清单：`rl_sdk.hpp`、`loop.hpp`、`fsm.hpp`、`fsm_ancnW1.hpp`、`rl_real_ancnW1.hpp`、`observation_buffer.hpp`。当前三层都干净，改动时请保持。

2. **不许读 yaml 配置**
   配置（如 `torque_limits`、`fixed_kp`）必须由适配层读好后，**以结构体或函数参数传进来**。SDK 层一旦出现 `params.Get<...>()`，就绑死 rl_sar 的配置体系。

3. **不许出现 RL 概念**
   SDK 层只认 `motor_id`（1~16 物理编号），不知道 policy 顺序、不知道 action / observation。`policy ⇄ physical` 的重排属于适配层（见 §4.3）。

4. **日志只用 `std::cout` / `std::cerr`**
   不要用 rl_sar 的 `LOGGER::*`。统一用 `[CanInterface]` / `[MotorDriver]` / `[MotorFsm]` 前缀。

补充：**线程安全由 SDK 层自己保证**。can 的 per-motor 反馈缓存、MotorFsm 的 `last_fault_` 都已加锁，适配层调用时无需额外加锁；但也**不要在 200Hz 控制线程里直接调用 SDK 的阻塞式 API**。

### 9.5 何时抽离、怎么抽

**现在不要抽**：Q2 未答复，若需拆多 CAN 总线则 `motor_driver` 要重写；且从未上电验证，此刻抽离只是换个目录。

**触发条件**：真机上电跑通 + Q1/Q2 明确答复之后。

**届时操作**（约 1 小时，不动逻辑只挪位置）：

```bash
mkdir -p library/thirdparty/robot_sdk/ancnw1/{include,src}
git mv include/ancnW1_{can_interface,motor_driver,motor_fsm}.hpp \
       library/thirdparty/robot_sdk/ancnw1/include/
git mv src/ancnW1_{can_interface,motor_driver,motor_fsm}.cpp \
       library/thirdparty/robot_sdk/ancnw1/src/
```

```cmake
add_library(ancnw1_sdk STATIC
    library/thirdparty/robot_sdk/ancnw1/src/ancnW1_can_interface.cpp
    library/thirdparty/robot_sdk/ancnw1/src/ancnW1_motor_driver.cpp
    library/thirdparty/robot_sdk/ancnw1/src/ancnW1_motor_fsm.cpp)
target_include_directories(ancnw1_sdk PUBLIC library/thirdparty/robot_sdk/ancnw1/include)
target_link_libraries(ancnw1_sdk PUBLIC Threads::Threads)

# rl_real_ancnW1 原先列出的这三个 src/*.cpp 删除，改为：
target_link_libraries(rl_real_ancnW1 ancnw1_sdk)
```

**若厂商后来真给了 SDK**：优先用厂商的替换 `motor_driver`（协议层），保留自写的 `can_interface`（要上 RK3588）与 `motor_fsm`（若厂商没给）。只要新 driver 保持原有公开接口，上面两层不用动 —— 这就是当初分层的收益。

---

## 10. TCP 桥接 ancnW1_tcp_bridge（上位机对接）

### 10.1 现状：已搁置

| 项 | 说明 |
|---|---|
| 文件 | `include/ancnW1_tcp_bridge_protocol.hpp`（498 行） |
| 编译状态 | **孤儿文件** —— 无任何 `.cpp` include 它，不参与编译链接 |
| 影响 | 对机器人行为**零影响**，不存在半成品依赖 |
| 验证 | 已用临时 round-trip 程序验证通过（见 §10.4，验证程序已删除） |

**搁置原因**：三个硬件阻塞项未解，此刻做完也只能在假数据上调。等真机跑通、确有上位机需求时再实现本体（收发线程 + 集成）。

### 10.2 架构位置与命名

```
Windows PC                              RK3588
┌──────────────┐        TCP           ┌───────────────────────────┐
│  C# 上位机    │ ←── 遥测 @200Hz ──── │  rl_real_ancnW1           │
│  （connect）  │ ── FSM/速度指令 ───→ │   └ TcpBridge（成员模块）  │
└──────────────┘                      └───────────────────────────┘
```

> ⚠️ **命名上踩过的坑：不要叫 `Server`**。`listen/connect` 只是 TCP 连接角色；**架构上它不是独立层，而是嵌在 `rl_real_ancnW1` 内的成员模块**。文件名也因此定为 `tcp_bridge` 而非 `tcp_server`。

### 10.3 为什么是二进制而非 JSON

> ⚠️ 早期注释里写的理由是"省带宽"，**这个理由是错的**：368B × 200Hz = 73.6 KB/s，千兆网毫无压力。

真正理由是两条：

1. **CPU 开销** —— 16 关节浮点打成 JSON 要逐个转字符串（每个约 10~12 字符），加 key 名后整帧膨胀到 2~3 KB，是二进制的 5~8 倍。200Hz 下 RK3588 的 ARM 核持续序列化、对端持续字符串解析，这部分开销实打实抢控制线程 CPU。
2. **浮点精度** —— `float32 → 十进制文本 → 解析回 float32` 会丢精度，除非用 `%.9g`（又把空间代价拉回来）。关节角度、力矩进了 RL 观测会被放大。

没用 protobuf 是因为要引入第三方库与 `.proto` 编译步骤，对这个固定布局偏重；没用 ROS topic 是因为上位机是 Windows C# 不跑 ROS。

> **注意区分两个概念**：「二进制」是选择，「流式粘包处理」是 TCP 的固有属性 —— 用 JSON 照样要处理粘包，只是把 `magic+CRC` 换成分隔符。

### 10.4 帧格式与已验证内容

```
┌────────┬─────┬──────┬─────┬─────┬──────────┬────────┐
│ magic  │ ver │ type │ len │ seq │ payload  │ crc16  │
│  2B    │ 1B  │  1B  │ 2B  │ 2B  │  len B   │   2B   │
└────────┴─────┴──────┴─────┴─────┴──────────┴────────┘

magic = 0xA55A（小端上线字节：5A A5）
len   = payload 字节数（不含头部与 CRC）
crc16 = CRC16-Modbus，覆盖 [magic .. payload] 全部字节
字节序 = 统一小端（与电机 CAN 协议一致；C# BinaryWriter/BinaryReader 默认即小端，无需反转）
```

命令字**按方向分区**：`0x01~0x0F` = 桥接→上位机，`0x10~0x1F` = 上位机→桥接。好处是对端收到非本方向区间的命令字可直接判为异常帧，不必等到业务层才发现"方向搞反"。

| 命令字 | 方向 | payload | 说明 |
|---|---|:--:|---|
| `kMsgRobotState` `0x01` | → 上位机 | 368 B | 关节 + IMU + 电机健康 + FSM，200Hz |
| `kMsgAck` `0x02` | → 上位机 | 2 B | 命令应答（`acked_type` 回显，便于并发时对应） |
| `kMsgCmdFsm` `0x10` | → 桥接 | 2 B | 切状态 / 急停（`stop` 优先级**高于** `target_state`） |
| `kMsgCmdVelocity` `0x11` | → 桥接 | 12 B | `vx, vy, omega` |
| `kMsgCmdPing` `0x12` | → 桥接 | 0 B | 心跳 |

遥测帧 368 B 的完整偏移表见源文件注释（与 C# `BinaryReader` 读取顺序必须逐字段一致）。

已验证（`g++ -Wall -Wextra -Wpedantic` 下头文件零警告）：

```
[OK] round-trip 正常帧
[OK] CRC 损坏被拒绝并滑窗
[OK] 半包不消耗字节        ← 关键：consumed 必须为 0，否则半包会被吃掉一截
[OK] 补齐后半包可解出
[OK] 垃圾前缀后成功重新同步
[OK] 序列化/反序列化 + 越界保护
```

### 10.5 ⚠️ 实现本体时的三条失效保护（必须做到）

C# 不调用时应当完全无感，真正要防的是"连上之后上位机崩溃 / 断网"。

**1. 网络 IO 绝不能进控制线程**
所有 `accept/recv/send` 只在独立线程里做，socket 设非阻塞。
> 违反后果：上位机断网后不再读，发送缓冲区满 → `send` 阻塞 → **200Hz 控制环卡死 → 机器人当场摔倒**。这是整个模块最致命的失败模式。

**2. 上位机指令必须带超时**
超过 500ms 没收到新指令就自动把 `commands` 清零。因此 C# 端要**持续发送**（≥50Hz，哪怕值不变），不能发一次就停。
> 违反后果：上位机发"前进 1 m/s"后掉线，机器人带着这条指令一直跑到撞墙。

**3. 本地安全永远压过远端**
键盘 / 物理急停优先级最高；TCP 下发的任何指令都不能绕过 `motor_fsm` 的故障检测。建议在 yaml 给桥接加 `enable` 开关，现场出问题能一键关掉排查。

做到这三条，桥接就是**纯旁路观察者**：连不连都不改变机器人行为，最坏只是上位机黑屏。

---

## 11. 当前进度总表

| 模块 | 状态 | 说明 |
|---|---|---|
| CAN-FD 通信 `can_interface` | ✅ 完成 | 已修 busy-loop 隐患（错误分支加退避休眠） |
| 协议编解码 `motor_driver` | ✅ 完成 | 组帧 / 解析 / 单位换算已验证 |
| 使能状态机 `motor_fsm` | ✅ 完成 | 已修 `AllMotorsHaveBit` 空集误判 |
| RL 适配层 `rl_real_ancnW1` | ✅ 完成 | 594 行，含 Kp 透传修正（见 §4.1） |
| RLFSM 四状态 `fsm_ancnW1` | ✅ 完成 | Passive / GetUp / GetDown / RL |
| 构建集成 | ✅ 完成 | `./build.sh -m rl_real_ancnW1` 通过 |
| 注释密度 | ✅ 达标 | 9 个文件均 38%~52% |
| TCP 协议文件 | ⏸ 搁置 | 孤儿文件，见 §10 |
| SDK 抽离 | ⏸ 待触发 | 见 §9.5 |
| **硬件闭环验证** | ❌ **未做** | **从未上电** |
| IMU | ❌ 桩实现 | 观测 `ang_vel` / `gravity_vec` 全零，策略必崩 |
| `torque_limits` = 113 | ⚠️ 存疑 | 手册无此档位，需按 BOM 改 |
| 16 电机寻址 | ⏸ 阻塞 | Q2 未答复 |

**结论**：链路约 90%，闭环验证 0%。严格来说 sim2real 适配**尚未完成** —— 直到有一帧真实反馈进来、且 policy 输出能驱动电机为止。

---

## 12. 后续启用清单（按依赖顺序）

1. **拿到厂商答复**（Q1/Q2/Q3/Q4，见 §6）
   → Q2 若要求拆多 CAN 总线，需重写 `motor_driver` 分帧逻辑并增加 socket
2. **接 IMU**（SPI/I2C → RK3588），替换 `InitImu()` / `UpdateImuData()`
   → 这是比电机协议更早的硬阻塞，不解决则策略无法运行
3. **按 BOM 改 `torque_limits`**；`base.yaml` 的 `step_wheel_kp: 15.0 → 0.0`
4. **台架试验**：卸轮子、架空机身、限幅降到 10%
   → 先验单关节使能 / 阻抗，再整机上电
5. **sim2real 调参**：`config.yaml` 的 `rl_kp` / `rl_kd` / `action_scale`
6. （可选）**启用 TCP 上位机**：补 `ancnW1_tcp_bridge.{hpp,cpp}` 本体 + CMake，实现 §10.5 三条保护
7. （可选）**SDK 抽离**：按 §9.5 挪到 `library/thirdparty/robot_sdk/ancnw1/`
