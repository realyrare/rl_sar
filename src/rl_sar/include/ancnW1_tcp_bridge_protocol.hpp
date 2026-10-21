/*
 * Copyright (c) 2024-2025 ancn
 * SPDX-License-Identifier: Apache-2.0
 *
 * ancnW1 TCP 桥接协议定义（RK3588 桥接模块 ↔ Windows C# 上位机）
 * ------------------------------------------------------------------
 * 本文件是**链路契约**，C# 端必须严格镜像实现，任何字段改动都需双端同步。
 *
 * ==================== 术语澄清 ====================
 * 下文出现的 "服务端 / 客户端" **仅描述 TCP 连接角色**（listen 端 / connect 端），
 * 不代表架构分层。架构上是这样：
 *   · RK3588 侧 = 嵌在 rl_real_ancnW1 内的**桥接模块**（非独立进程）
 *   · Windows 侧 = C# 上位机，负责 UI 与状态曲线显示
 * 强调这一点，是为了避免日后误以为存在一个名为 "server" 的架构层。
 *
 * ==================== 为什么用二进制而不是 JSON ====================
 *   - 遥测 @200Hz，JSON 序列化/解析开销大，且浮点精度有损
 *     （"打印成十进制文本再解析回来"会丢失二进制精度）
 *   - 368 字节定长帧 @200Hz = 73.6 KB/s，TCP 毫无压力，二进制更省带宽
 *
 * 字节序：**统一小端**（与电机 CAN 协议的手册约定保持一致，避免项目里出现两套规则）
 *         幸运的是 C# 的 BinaryWriter / BinaryReader 默认即小端，**无需反转**
 *
 * ┌────────┬─────┬──────┬─────┬─────┬──────────┬────────┐
 * │ magic  │ ver │ type │ len │ seq │ payload  │ crc16  │
 * │  2B    │ 1B  │  1B  │ 2B  │ 2B  │  len B   │   2B   │
 * └────────┴─────┴──────┴─────┴─────┴──────────┴────────┘
 *   magic = 0xA55A（小端 → 线上字节 5A A5）
 *   len   = payload 字节数（不含头部与 CRC）
 *   crc16 = CRC16-Modbus，覆盖 [magic .. payload] 全部字节
 *
 * ==================== 为什么这里是 header-only ====================
 * 本文件含有实现代码，看似违背"头文件只放声明"的惯例，
 * 但**不会产生 ODR（多重定义）链接错误**，依据是：
 *   · 所有自由函数均已标记 inline
 *   · 所有常量均标记 static constexpr → 内部链接，各 TU 各自持有一份副本
 *   · BufWriter / BufReader 的成员函数为类内定义 → 隐式 inline
 *
 * 仍然选择 header-only 的理由：
 *   · 本文件是**纯计算逻辑**：CRC、序列化、组帧；
 *     无全局可变状态、无 I/O、无系统调用 —— 正是适合 header-only 的形态
 *   · C# 端需逐字节镜像本实现，逻辑与声明同处一文件便于双端对照维护
 *   · 本 TU 数量少（当前 4 个），拆分带来的编译收益可忽略
 *
 * ⚠ 拆分准则：一旦需要引入非 inline 的静态数据，
 *   或出现模板以外的复杂实现，应立即把实现移出到 ancnW1_tcp_bridge_protocol.cpp。
 *
 * 对应 C# 端：TcpProtocol.cs（字段顺序、类型、字节序必须完全一致）
 */

#ifndef ANCNW1_TCP_BRIDGE_PROTOCOL_HPP
#define ANCNW1_TCP_BRIDGE_PROTOCOL_HPP

#include <cstdint>
#include <cstring>
#include <vector>

namespace ancn_w1
{
namespace tcp
{

// ============================================================================
// 链路层常量
// ============================================================================

/// 帧同步头（magic 值 0xA55A，小端下发字节为 0x5A, 0xA5）
static constexpr uint16_t kMagic = 0xA55A;

/// 协议版本（双端不一致应拒绝通信）
static constexpr uint8_t kVersion = 1;

/// 帧头长度 = magic(2) + ver(1) + type(1) + len(2) + seq(2)
static constexpr size_t kHeaderLen = 8;

/// CRC16 长度
static constexpr size_t kCrcLen = 2;

/// 单帧 payload 上限（防御畸形包，防止 OOM）
static constexpr size_t kMaxPayloadLen = 1024;

/// 接收缓冲区建议大小 = 头 + 最大 payload + CRC
static constexpr size_t kRxBufferLen = kHeaderLen + kMaxPayloadLen + kCrcLen;

/// 默认 TCP 监听端口（可用命令行 --tcp-port 覆盖）
static constexpr uint16_t kDefaultPort = 8080;

// ============================================================================
// 命令字（type 字段）
// ============================================================================

/**
 * @brief 帧类型——既是命令字，也隐含传输方向
 *
 * 方向标记**仅表示 TCP 字节流方向**（再次强调：不代表架构角色）：
 *   S2C = 桥接模块 → 上位机（遥测上送 @200Hz）
 *   C2S = 上位机 → 桥接模块（指令下发，按需）
 *
 * @note 命令字按方向**分区**：0x01~0x0F 归 S2C，0x10~0x1F 归 C2S。
 *       好处有二：
 *         ① 看命令字高半字节就能判断方向，不必查表
 *         ② 对端收到非本方向区间的命令字可直接判为异常帧，
 *            无需等到上层业务才发现"方向搞反了"
 */
enum MsgType : uint8_t
{
    // ---------- S2C：桥接模块 → 上位机 ----------
    kMsgRobotState = 0x01,   ///< 周期遥测（关节 + IMU + 电机健康 + FSM），200Hz
    kMsgAck        = 0x02,   ///< 命令应答（result 见 AckResult）

    // ---------- C2S：上位机 → 桥接模块 ----------
    kMsgCmdFsm      = 0x10,  ///< 切换机器人状态 / 急停
    kMsgCmdVelocity = 0x11,  ///< 下发速度指令 vx, vy, omega
    kMsgCmdPing     = 0x12,  ///< 心跳，桥接模块回 kMsgAck
};

/// kMsgAck 的 result 取值
enum AckResult : uint8_t
{
    kAckOk       = 0,   ///< 命令已受理
    kAckRejected = 1,   ///< 已解析但被拒绝（如当前状态不允许该操作）
    kAckUnknown  = 2,   ///< 未知命令字
    kAckBadCrc   = 3,   ///< CRC 校验失败的包
};

/// 机器人 FSM 状态（与 fsm_ancnW1.hpp 的 FSMState 保持一致）
enum RobotFsmId : uint8_t
{
    kFsmPassive = 0,   ///< 趴下 / 关节阻尼
    kFsmGetUp   = 1,   ///< 起立
    kFsmGetDown = 2,   ///< 趴下过渡
    kFsmRL      = 3,   ///< RL 行走
};

// ============================================================================
// payload 布局
// ============================================================================

/// kMsgRobotState 的 payload 长度（字节），见下方偏移表
static constexpr size_t kRobotStateLen = 368;

/* kMsgRobotState payload 偏移表（C# BinaryReader 必须按同一顺序读）
 *
 * 偏移  类型              字段                  说明
 * ----  ----------------  --------------------  --------------------------------
 * 0     uint32            tick_ms               桥接模块启动后经过的毫秒数
 * 4     uint8             fsm_state             见 RobotFsmId
 * 5     uint8             motor_fsm_state       MotorFsmState：0失能1使能中2使能3失能中4设零位5故障
 * 6     uint8             motors_online         在线（有反馈）电机数 0~16
 * 7     uint8             emergency_stop        1=急停已触发
 * 8     float32[4]        imu_quat              姿态四元数 w,x,y,z
 * 24    float32[3]        imu_ang_vel           机体角速度 rad/s（IMU 未接线时为 0）
 * 36    float32[3]        projected_gravity     机体系重力向量（IMU 未接线时为 0）
 * 48    float32[16]       joint_q               关节位置 rad（policy 顺序）
 * 112   float32[16]       joint_dq              关节速度 rad/s
 * 176   float32[16]       joint_tau             关节力矩 N·m
 * 240   float32[3]        commands              vx, vy, omega（当前速度指令）
 * 252   float32[16]       actions               上一拍策略输出（调试观测用）
 * 316   uint8[16]         motor_temp            电机温度 raw，换算：T(℃) = val - 40
 * 332   uint8[16]         motor_status          状态码 bit0使能 bit1过压 bit2过流 bit3过温
 *                                               bit4过速 bit5编码器 bit6欠压
 * 348   uint8[16]         motor_volt            母线电压 raw，0~255 即 0~255V
 * 364   float32           ctrl_period_ms        实测控制周期（健康度指标，理想 5.0ms）
 *                                              总长 368 字节
 */

/**
 * @name C2S 命令的 payload 长度常量
 * @note C# 端组帧时应**引用这些常量的镜像副本**，不要硬编码数字。
 *       将来新增字段导致长度变化，硬编码的上位机会静默造成解析错位。
 * @{
 */

/**
 * @brief kMsgCmdFsm 的 payload 长度
 *
 * payload（2 字节）布局：
 *   偏移  类型     字段          说明
 *   0     uint8    target_state  RobotFsmId：要切换到的目标状态
 *   1     uint8    stop          1=急停（立刻转 Passive 并关电机使能）
 *
 * @warning **stop 字段优先级高于 target_state**：
 *          stop=1 时应忽略 target_state，直接执行急停。
 *          这样即使上位机出错同时发了"切到行走"+stop，也不会绕过安全。
 */
static constexpr size_t kCmdFsmLen = 2;

/**
 * @brief kMsgCmdVelocity 的 payload 长度
 *
 * payload（12 字节）布局：
 *   偏移  类型      字段    说明
 *   0     float32   vx      前后速度 m/s（正=前进）
 *   4     float32   vy      左右速度 m/s（正=左移）
 *   8     float32   omega   偏航角速度 rad/s（正=逆时针）
 *
 * @warning **上位机必须持续发送本命令**（即便数值不变，建议 ≥50Hz）。
 *          桥接模块侧会做超时保护：超过一定时间没收到新指令就自动清零，
 *          防止上位机崩溃/断网后机器人带着最后一条速度指令一直跑下去。
 *          这条约束是整个链路最重要的失效保护，C# 端必须实现。
 */
static constexpr size_t kCmdVelocityLen = 12;

/**
 * @brief kMsgAck 的 payload 长度
 *
 * payload（2 字节）布局：
 *   偏移  类型     字段         说明
 *   0     uint8    acked_type   应答的是哪个命令字
 *   1     uint8    result       AckResult
 *
 * @note acked_type 原样回显被应答的命令字，
 *       使上位机能把应答与请求一一对应（避免并发下发时串味）。
 */
static constexpr size_t kAckLen = 2;

/// @}

// ============================================================================
// CRC16-Modbus（poly 0xA001，初值 0xFFFF，结果低字节在前）
// ============================================================================

/**
 * @brief 计算 CRC16-Modbus
 * @param data 数据起始
 * @param len  长度
 * @return 16 位 CRC（传输时低字节在前）
 *
 * C# 端镜像：
 *   ushort crc = 0xFFFF;
 *   foreach (byte b in data) {
 *       crc ^= b;
 *       for (int i = 0; i < 8; i++)
 *           crc = (crc & 1) != 0 ? (ushort)((crc >> 1) ^ 0xA001) : (ushort)(crc >> 1);
 *   }
 */
inline uint16_t Crc16Modbus(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int b = 0; b < 8; ++b)
        {
            crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001)
                                 : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

// ============================================================================
// 缓冲区读写工具（显式小端序列化，避免结构体对齐带来的跨平台风险）
// C# 端对应 BinaryWriter / BinaryReader，默认即为小端，**无需反转**
// ============================================================================

/**
 * @brief payload 序列化器：把各字段按**显式小端**逐个追加到缓冲区
 *
 * ==================== 为什么手写小端，而不是 memcpy 一个 struct ====================
 * 看起来把整个 payload 定义成一个 struct 再 memcpy 更省事，但那样会踩两个大坑：
 *   ① **对齐填充**：编译器会在 struct 成员之间插入空隙（padding），
 *      而插入规则随平台和编译器而异。一旦两端 padding 不一致，
 *      memcpy 出来的字节流就是错位的 —— 这是典型的 struct packing 事故。
 *   ② **字节序**：struct 里的多字节字段直接落到大端平台上就反了。
 *
 * 逐字节显式写入则完全由代码决定布局，与编译器、平台均无关，
 * 且便于在 C# 端用 BinaryWriter 一行一行对照着实现。
 *
 * @note 本类**不持有缓冲区**，只保存外部 vector 的指针。
 *       生命周期由调用方负责保证长于本对象 ——
 *       与 MotorFsm 使用裸指针是同样的取舍：所有权关系清晰且在栈上短命。
 */
class BufWriter
{
public:
    /// @param buf 目标缓冲区指针（**不接管所有权**）
    explicit BufWriter(std::vector<uint8_t> *buf) : buf_(buf) {}

    /// 写入 1 字节（单字节不涉及字节序）
    void U8(uint8_t v)   { buf_->push_back(v); }

    void U16(uint16_t v)
    {
        buf_->push_back(static_cast<uint8_t>(v & 0xFF));
        buf_->push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    }

    void U32(uint32_t v)
    {
        for (int i = 0; i < 4; ++i)
        {
            buf_->push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
        }
    }

    /// float32 按 IEEE754 位模式写入（C# BinaryWriter.Write(float) 完全等价）
    void F32(float v)
    {
        uint32_t bits;
        std::memcpy(&bits, &v, sizeof(bits));
        U32(bits);
    }

    void Raw(const uint8_t *src, size_t n)
    {
        buf_->insert(buf_->end(), src, src + n);
    }

private:
    std::vector<uint8_t> *buf_;
};

/**
 * @brief payload 反序列化器：带边界检查的显式小端读取
 *
 * @warning **安全第一**：所有 Read 方法在越界时返回 false，
 *          并且**不移动读指针、不写输出参数**。
 *          这样上层遇到畸形包可以安全地中止解析，
 *          而不会读到缓冲区外的内存（这类越界读正是远程可利用漏洞的常见来源）。
 *
 * @note data_ 是**只读外部指针**：本类不拷贝 payload，也不接管所有权，
 *       适用于"缓冲区已确定、只在此作用域内解析一遍"的场景。
 */
class BufReader
{
public:
    /// @param data 待解析数据起始
    /// @param len  数据长度（用于边界检查）
    BufReader(const uint8_t *data, size_t len) : data_(data), len_(len) {}

    bool U8(uint8_t *out)
    {
        if (pos_ + 1 > len_) return false;
        *out = data_[pos_];
        pos_ += 1;
        return true;
    }

    bool U16(uint16_t *out)
    {
        if (pos_ + 2 > len_) return false;
        *out = static_cast<uint16_t>(data_[pos_]) |
               static_cast<uint16_t>(data_[pos_ + 1] << 8);
        pos_ += 2;
        return true;
    }

    bool U32(uint32_t *out)
    {
        if (pos_ + 4 > len_) return false;
        *out = 0;
        for (int i = 0; i < 4; ++i)
        {
            *out |= static_cast<uint32_t>(data_[pos_ + i]) << (8 * i);
        }
        pos_ += 4;
        return true;
    }

    bool F32(float *out)
    {
        uint32_t bits;
        if (!U32(&bits)) return false;
        std::memcpy(out, &bits, sizeof(*out));
        return true;
    }

    bool Raw(uint8_t *dst, size_t n)
    {
        if (pos_ + n > len_) return false;
        std::memcpy(dst, data_ + pos_, n);
        pos_ += n;
        return true;
    }

    size_t Remaining() const { return len_ - pos_; }
    size_t Pos() const { return pos_; }

private:
    const uint8_t *data_;
    size_t len_;
    size_t pos_ = 0;
};

// ============================================================================
// 组帧 / 解帧
// ============================================================================

/**
 * @brief 组装一个完整帧（头 + payload + CRC）
 * @param type    命令字
 * @param seq     帧序号（用于检测丢包）
 * @param payload 有效载荷
 * @param out     输出的完整帧字节流
 */
inline void BuildFrame(MsgType type, uint16_t seq,
                       const std::vector<uint8_t> &payload,
                       std::vector<uint8_t> *out)
{
    out->clear();
    out->reserve(kHeaderLen + payload.size() + kCrcLen);

    out->push_back(static_cast<uint8_t>(kMagic & 0xFF));
    out->push_back(static_cast<uint8_t>((kMagic >> 8) & 0xFF));
    out->push_back(kVersion);
    out->push_back(static_cast<uint8_t>(type));

    const uint16_t len = static_cast<uint16_t>(payload.size());
    out->push_back(static_cast<uint8_t>(len & 0xFF));
    out->push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
    out->push_back(static_cast<uint8_t>(seq & 0xFF));
    out->push_back(static_cast<uint8_t>((seq >> 8) & 0xFF));

    out->insert(out->end(), payload.begin(), payload.end());

    // CRC 覆盖 magic 起到 payload 结束
    const uint16_t crc = Crc16Modbus(out->data(), out->size());
    out->push_back(static_cast<uint8_t>(crc & 0xFF));
    out->push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
}

/**
 * @brief 尝试从流式缓冲区里解出一个完整帧
 *
 * TCP 是字节流，存在粘包/半包，需循环调用本函数：
 *   - 返回 true 且 consumed>0：成功解出一帧，从 buf 头部裁掉 consumed 字节
 *   - 返回 false 且 consumed=0：数据不足，保留缓冲等下次
 *   - 返回 false 且 consumed>0：帧头非法，丢弃 consumed 字节重新同步（滑窗找 magic）
 *
 * @param buf      累积的接收缓冲（会被就地裁剪）
 * @param out_type 解出的命令字
 * @param out_seq  解出的帧序号
 * @param out_pl   解出的 payload
 * @param consumed 本次消耗/丢弃的字节数
 */
inline bool TryParseFrame(std::vector<uint8_t> *buf, MsgType *out_type,
                          uint16_t *out_seq, std::vector<uint8_t> *out_pl,
                          size_t *consumed)
{
    *consumed = 0;
    const uint8_t *d = buf->data();
    const size_t n = buf->size();

    // 至少要有完整帧头
    if (n < kHeaderLen) return false;

    // 同步：magic 必须对齐在 buf[0]
    if (d[0] != static_cast<uint8_t>(kMagic & 0xFF) ||
        d[1] != static_cast<uint8_t>((kMagic >> 8) & 0xFF))
    {
        // 从 buf[1] 起寻找下一个 magic 起点，最多丢 1 字节以保留重叠可能
        *consumed = 1;
        return false;
    }

    const uint8_t ver = d[2];
    const size_t len = static_cast<size_t>(d[4]) | (static_cast<size_t>(d[5]) << 8);
    const size_t total = kHeaderLen + len + kCrcLen;

    if (ver != kVersion)
    {
        // 版本不符：整帧丢弃
        if (n >= total) *consumed = total;
        else            *consumed = 1;
        return false;
    }

    if (len > kMaxPayloadLen)
    {
        // payload 超长，视为乱码，滑一字节重新同步
        *consumed = 1;
        return false;
    }

    if (n < total) return false;   // 半包，等更多数据

    // CRC 校验（覆盖 magic..payload）
    const uint16_t recv_crc = static_cast<uint16_t>(d[total - 2]) |
                              static_cast<uint16_t>(d[total - 1] << 8);
    const uint16_t calc_crc = Crc16Modbus(d, kHeaderLen + len);
    if (recv_crc != calc_crc)
    {
        *consumed = 1;   // 校验失败，滑窗重同步
        return false;
    }

    *out_type = static_cast<MsgType>(d[3]);
    *out_seq  = static_cast<uint16_t>(d[6]) | static_cast<uint16_t>(d[7] << 8);
    out_pl->assign(d + kHeaderLen, d + kHeaderLen + len);
    *consumed = total;
    return true;
}

}  // namespace tcp
}  // namespace ancn_w1

#endif  // ANCNW1_TCP_PROTOCOL_HPP
