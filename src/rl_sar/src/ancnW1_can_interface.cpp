/*
 * Copyright (c) 2024-2025 ancn
 * SPDX-License-Identifier: Apache-2.0
 *
 * ancnW1 CAN-FD 接口实现（Linux socketcan）
 * ----------------------------------------
 * socketcan API 关键点：
 *   - SOCK_RAW | CAN_RAW + CAN_RAW_FD_FRAMES 启用 CAN-FD
 *   - struct canfd_frame { canid_t can_id; uint8_t len; uint8_t flags; uint8_t data[64]; __attribute__((aligned(8))); }
 *   - flags = CANFD_BRS（数据段加速位速率）= 0x01，我们始终置 1 以启用 5Mbps 数据段
 *   - 发送超时 1ms（避免阻塞控制循环）；接收超时 100ms（让线程可定期 wakeup 检查 running_）
 */

#include "ancnW1_can_interface.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>

#include <cerrno>      // errno 与各错误码（EINTR / EAGAIN 等）
#include <thread>      // std::this_thread::sleep_for（错误退避用）
#include <fcntl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <chrono>

namespace ancn_w1
{

CanInterface::CanInterface() = default;

CanInterface::~CanInterface()
{
    StopReceiveThread();
    Close();
}

/**
 * @brief 打开 CAN-FD 接口并完成 socketcan 初始化
 *
 * 完整流程 5 步，缺一不可，任何一步失败都会 Close() 并返回 false：
 *
 *   ① socket()            创建 PF_CAN / SOCK_RAW / CAN_RAW 套接字
 *   ② setsockopt()        启用 CAN_RAW_FD_FRAMES —— 不开这个发不了 64 字节
 *   ③ ioctl(SIOCGIFINDEX) 把接口名 "can0" 解析成内核接口索引号
 *   ④ bind()              把套接字绑定到该接口
 *   ⑤ setsockopt()        设置收发超时
 *
 * @param ifname socketcan 接口名，如 "can0"（RK3588 上按实际网卡名调整）
 * @return true=打开成功；false=失败（权限不足 / 接口不存在 / 内核不支持 FD）
 *
 * @warning 前置条件（用户态先执行一次，需要 root）：
 *   sudo ip link set can0 down
 *   sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
 *
 * @note 为什么必须先在 `ip link` 里配波特率？
 *   Linux 的 CAN 波特率在网络设备层配置，socket 层无法设置。
 *   所以代码只能要求已在用户态配好。手册 §3.2.1 规定
 *   仲裁段 1Mbps、数据段 5Mbps、标准帧。
 *
 * @note 两个超时值这样选的理由：
 *   发送 1ms   —— 单个 CAN-FD 帧实际发送耗时远小于此，一旦超时说明总线异常。
 *                 宁可本周期发送失败（下一周期重试），
 *                 也绝不能阻塞住 200Hz 的控制环。
 *   接收 100ms —— 让 ReceiveLoop() 每 100ms 至少被唤醒一次，
 *                 从而有机会检查 running_ 标志并及时退出；
 *                 否则 StopReceiveThread() 的 join() 会一直卡住。
 */
bool CanInterface::Open(const std::string &ifname)
{
    // 重复打开保护：同一个对象不允许持有两个 fd
    if (sock_ >= 0)
    {
        std::cerr << "[CanInterface] already opened on " << ifname_ << std::endl;
        return false;
    }

    ifname_ = ifname;

    // ① 创建 CAN_RAW 套接字
    //    PF_CAN   = CAN 协议族（不是 AF_INET）
    //    SOCK_RAW = 收发原始 CAN 帧（不做流式解析）
    //    CAN_RAW  = CAN 协议本身（区别于 CAN_BCM 等上层协议）
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0)
    {
        std::cerr << "[CanInterface] socket() failed: " << strerror(errno)
                  << " (need CAP_NET_RAW or root)" << std::endl;
        sock_ = -1;
        return false;
    }

    // ② 启用 CAN-FD 帧支持
    //    这是最关键的一步：不启用时内核限定为经典 CAN 的 8 字节，
    //    下发 64 字节会被拒绝。手册 §3.2.1 要求下发 64 字节，因此必须开。
    int enable_fd = 1;
    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0)
    {
        std::cerr << "[CanInterface] setsockopt(CAN_RAW_FD_FRAMES) failed: "
                  << strerror(errno) << " (kernel/module may not support CAN-FD)" << std::endl;
        Close();
        return false;
    }

    // ③ 接口名 → 内核索引
    //    内核用数字索引标识网卡，用户习惯用名字，这一步做转换。
    //    strncpy 后手动补 '\0'：IFNAMSIZ 是缓冲区长度，必须保证结尾。
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
    {
        std::cerr << "[CanInterface] ioctl(SIOCGIFINDEX) for " << ifname
                  << " failed: " << strerror(errno) << std::endl;
        Close();
        return false;
    }

    // ④ 绑定到接口（不 bind 的话内核不知道这个 socket 对应哪个网卡）
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        std::cerr << "[CanInterface] bind() to " << ifname
                  << " failed: " << strerror(errno) << std::endl;
        Close();
        return false;
    }

    // ⑤ 设置收发超时（理由见 @note）
    struct timeval send_tv = {0, 1000};    // 1 ms
    struct timeval recv_tv = {0, 100000};  // 100 ms
    setsockopt(sock_, SOL_SOCKET, SO_SNDTIMEO, &send_tv, sizeof(send_tv));
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &recv_tv, sizeof(recv_tv));

    std::cout << "[CanInterface] opened " << ifname
              << " (CAN-FD, 1Mbps arb / 5Mbps data)" << std::endl;
    return true;
}

/**
 * @brief 关闭接口并释放资源
 *
 * 先停接收线程再关 fd —— 顺序不能反：
 * 若先关 fd，ReceiveLoop 会对已关闭的描述符调用 read()，行为未定义。
 *
 * @note 可重复调用（幂等）：sock_ < 0 时直接跳过，
 *       因此析构函数里再调一次是安全的。
 */
void CanInterface::Close()
{
    StopReceiveThread();   // 保证线程先退出，避免对已关闭 fd 操作
    if (sock_ >= 0)
    {
        ::close(sock_);
        sock_ = -1;
    }
}

/**
 * @brief 发送一帧 CAN-FD
 *
 * @param can_id 11 位 CAN ID（标准帧）。
 *               · 下发主机帧统一用 0x00（手册 §3.2.1 主站 CANID）
 *               · 超过 11 位的部分会被 CAN_SFF_MASK 截掉
 * @param data   要发送的数据
 * @param len    数据字节数，1~64。手册 §3.2.2 约定：6 个电机槽位时为 64，
 *               只控部分电机时可截短为 12/24/32/48 以节省带宽
 * @return true=已交给内核；false=参数非法或发送失败（总线忙/超时）
 *
 * @note canfd_frame 的填法：
 *   · can_id  — 帧 ID，标准帧只用低 11 位
 *   · len     — **实际要发的字节数**，内核据此决定线上帧长
 *   · flags   — CANFD_BRS 表示"数据段切换高速率"，
 *               手册 §3.2.1 要求数据段 5Mbps，所以常量置 1
 *   · data[64]— 数据负载；多余字节清零，保证报文确定性问题排查更容易
 *
 * @note 这里用 write() 而不是 sendto()：因为已经在 Open() 里 bind 了接口，
 *       目标地址已确定，write 更简洁。且始终写 sizeof(frame) 个字节，
 *       由内核根据 frame.len 决定实际帧长 —— 这是 socketcan 的标准用法。
 *
 * @note 失败只累加 error_count_ 而不抛异常：
 *       200Hz 控制环里任何阻塞/异常都会让整条控制链停摆，
 *       丢一帧的代价远小于卡死。
 */
bool CanInterface::Send(uint32_t can_id, const uint8_t *data, size_t len)
{
    // 参数防御：fd 有效、数据非空、长度在 1~64
    if (sock_ < 0 || data == nullptr || len == 0 || len > kMaxFrameLen)
    {
        error_count_.fetch_add(1);
        return false;
    }

    // canfd_frame 必须 8 字节对齐（sockcan 头文件已用 __attribute__((aligned(8))) 标注，
    // 这里声明成局部变量即自动满足对齐要求）
    struct canfd_frame frame {};   // 整体清零，未使用的 data 字节自动为 0
    frame.can_id = can_id & CAN_SFF_MASK;  // 取低 11 位，确保是标准帧
    frame.len = static_cast<__u8>(len);    // 实际负载长度（非 sizeof(frame)）
    frame.flags = CANFD_BRS;               // 数据段提速到 5Mbps
    std::memcpy(frame.data, data, len);

    // 手册 §3.2.2：保留字节可填 0。frame 初始化已清零，此处再显式处理一次，
    // 防止未来改为复用 frame 后出现脏数据。
    if (len < kMaxFrameLen)
    {
        std::memset(frame.data + len, 0, kMaxFrameLen - len);
    }

    // 写入完整 canfd_frame 结构，内核按 frame.len 截取真实负载
    ssize_t n = ::write(sock_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        error_count_.fetch_add(1);
        return false;
    }

    tx_count_.fetch_add(1);
    return true;
}

/**
 * @brief 同步接收一帧（用于诊断 / 单元测试）
 *
 * @param[out] can_id 收到的 CAN ID（低 11 位）
 * @param[out] data   数据缓冲区，建议 ≥ 12 字节
 * @param[in,out] len 传入=缓冲区容量；返回=实际收到的字节数
 * @param timeout_ms  0=非阻塞轮询；>0=最长阻塞毫秒数
 * @return true=收到一帧；false=超时、出错或缓冲不足
 *
 * @warning 本函数会修改 socket 上全局的 SO_RCVTIMEO，
 *          这会一并影响 ReceiveLoop() 的阻塞时长。
 *          正常运行应走 StartReceiveThread() + GetLatestFeedback()，
 *          两者不要混用；本接口仅供调试。
 *
 * @note *len 既做入参又做出参：
 *       入参是"能装多少"，出参是"实际装了多少"，
 *       min(frame.len, *len) 保证不会越界写入调用方缓冲区。
 */
bool CanInterface::Recv(uint32_t *can_id, uint8_t *data, size_t *len, int timeout_ms)
{
    if (sock_ < 0 || can_id == nullptr || data == nullptr || len == nullptr)
    {
        return false;
    }

    // 临时调整超时（仅本次调用，但注意会全局生效，见 @warning）
    struct timeval tv;
    if (timeout_ms == 0)
    {
        tv = {0, 0};   // 非阻塞：read 立即返回
    }
    else
    {
        tv.tv_sec  =  timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
    }
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct canfd_frame frame {};
    ssize_t n = ::read(sock_, &frame, sizeof(frame));
    if (n <= 0)
    {
        return false;  // 超时或错误（不区分，调用方只看成败）
    }

    *can_id = frame.can_id & CAN_SFF_MASK;

    // 防止 memcpy 越界：实际长度可能大于调用方缓冲区
    size_t copy_len = std::min(static_cast<size_t>(frame.len), *len);
    std::memcpy(data, frame.data, copy_len);
    *len = copy_len;
    rx_count_.fetch_add(1);
    return true;
}

/**
 * @brief 启动后台接收线程
 *
 * @return true=已启动；false=线程已在运行（重复调用被拒绝）
 *
 * @note running_.exchange(true) 是原子的"测试并置位"：
 *       返回 true 表示此前已经是 true（说明线程已在跑），
 *       借此避免多线程同时 Start 创建出两个线程。
 */
bool CanInterface::StartReceiveThread()
{
    if (running_.exchange(true))
    {
        return false;  // 已在运行
    }
    recv_thread_ = std::thread(&CanInterface::ReceiveLoop, this);
    return true;
}

/**
 * @brief 停止后台接收线程并等待其退出（阻塞）
 *
 * @note running_.exchange(false) 同样是原子的"读取并清零"：
 *       返回 false 说明此前就是 false（已停止），直接返回保证幂等。
 *       之后 join() 等待 ReceiveLoop 真正退出（最多等一个 SO_RCVTIMEO=100ms）。
 *
 * @note 为什么 Close() 和析构都要调它？
 *       保证"线程先于 socket 关闭"，否则线程会对已失效的 fd 调用 read()。
 */
void CanInterface::StopReceiveThread()
{
    if (!running_.exchange(false))
    {
        return;  // 已停止
    }
    if (recv_thread_.joinable())
    {
        recv_thread_.join();
    }
}

/**
 * @brief 后台接收线程主循环：持续读帧并按电机 ID 分派到本地缓存
 *
 * 设计要点：
 *   · 采用"覆盖式"缓存：每个电机只保留**最新**一帧，
 *     旧数据被无条件覆盖。这对控制是合理的——永远只需要最新状态。
 *   · FeedbackSlot 各带一把锁（per-slot mutex）而不是全局一把，
 *     这样 16 个电机的读写基本不互斥，降低与控制线程的竞争。
 *
 * 错误处理：
 *   read() 返回 <=0 有两类截然不同的情况，必须区分对待（详见循环内注释）。
 *   早期版本在此一律 continue，一旦 socket 出错（例如网口被 down），
 *   read 会立即返回 -1 而不阻塞，线程随即退化成忙循环、
 *   吃满一个 CPU 核，进而拖慢同进程的 200Hz 控制线程。
 *   现已通过"错误退避休眠"修复。
 */
void CanInterface::ReceiveLoop()
{
    while (running_.load())
    {
        struct canfd_frame frame {};
        ssize_t n = ::read(sock_, &frame, sizeof(frame));
        if (n <= 0)
        {
            // read() 返回 <=0 必须区分两种情况，否则可能陷入忙循环：
            //
            //   ① EINTR / EAGAIN
            //      被信号打断，或超过 SO_RCVTIMEO(100ms) 而超时 —— 均属正常，
            //      直接 continue 重新检查 running_ 即可，不做额外处理。
            //
            //   ② 其他错误（ENETDOWN / ENODEV / EBADF 等）
            //      表示 socket 已不可用，此时 read() 会**立即**返回而不阻塞。
            //      若放任 continue，本循环将以前所未有的频率空转，
            //      吃满一个 CPU 核，直接拖垮同进程的 200Hz 控制线程。
            //      → 退避休眠 1ms：既杜绝空转，也给总线恢复留出窗口。
            const int err = errno;
            if (err != EINTR && err != EAGAIN)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            continue;
        }

        // 反馈帧固定 12 字节（手册 §3.2.1）。长度不符说明是别的设备发的或是异常帧
        if (frame.len != kFeedbackLen)
        {
            error_count_.fetch_add(1);
            continue;
        }

        // 假设：反馈帧的 CAN ID 即电机 ID（手册未明示，待厂商确认）。
        // 取低 8 位后必须在 1~16 范围内才是我们要的电机。
        uint8_t motor_id = static_cast<uint8_t>(frame.can_id & 0xFF);
        if (motor_id < 1 || motor_id > kMaxMotorId)
        {
            error_count_.fetch_add(1);
            continue;
        }

        // 写入该电机专属槽位。锁的作用域刻意做得很小：
        // 只包住 memcpy 与标志更新，避免长时间占用而阻塞控制线程读取。
        FeedbackSlot &slot = feedbacks_[motor_id];
        {
            std::lock_guard<std::mutex> lock(slot.mtx);
            std::memcpy(slot.data.data(), frame.data, kFeedbackLen);
            slot.fresh = true;                                    // 标记"收到过"
            slot.ts = std::chrono::steady_clock::now();           // 供上层判断新鲜度
        }
        rx_count_.fetch_add(1);
    }
}

/**
 * @brief 取指定电机的最新反馈（线程安全）
 *
 * @param motor_id 电机编号 1~16
 * @param[out] out 12 字节原始反馈（未经解析，解析由 MotorDriver::ParseFeedback 负责）
 * @return true=取到了数据；false=参数非法，或**该电机至今没来过任何反馈**
 *
 * @note 本函数只保证"曾经收到过"，不判断数据是否新鲜。
 *       若要判断是否掉线（例如超过 N 毫秒没更新），需在 MotorFsm 里
 *       结合 FeedbackSlot 的时间戳来做——那是 kCommTimeout 的职责。
 *
 * @note 这里的设计取舍：宁可返回旧数据也不清空。清空会让上层误以为
 *       "关节位置变成 0"，而保留旧值至少不会出现伪跳变。
 */
bool CanInterface::GetLatestFeedback(uint8_t motor_id, std::array<uint8_t, kFeedbackLen> *out)
{
    if (motor_id < 1 || motor_id > kMaxMotorId || out == nullptr)
    {
        return false;
    }

    FeedbackSlot &slot = feedbacks_[motor_id];
    std::lock_guard<std::mutex> lock(slot.mtx);
    if (!slot.fresh)
    {
        return false;   // 该电机从未回过帧（可能没上电 / ID 不对）
    }
    *out = slot.data;
    return true;
}

}  // namespace ancn_w1
