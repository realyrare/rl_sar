/*
 * Copyright (c) 2024-2025 ancn
 * SPDX-License-Identifier: Apache-2.0
 *
 * ancnW1 CAN-FD 底层接口
 * ----------------------------------------
 * 负责 Linux socketcan 通信：打开/关闭接口、配置 CAN-FD 波特率、收发 64/12 字节帧。
 *
 * 通信模型（手册 §3.2.1）：
 *   - 仲裁段 1 Mbps，数据段 5 Mbps，标准帧
 *   - 主站 → 从站：64 字节（最多 6 个电机槽位）
 *   - 从站 → 主站：12 字节（单电机反馈）
 *   - 主站 CANID = 0x00；从站 CANID = 0x01~0x06（一帧内）
 *
 * 线程模型：
 *   - 后台线程持续接收，把每个电机 ID 的最新反馈写入槽位（覆盖式）
 *   - 主线程通过 GetLatestFeedback() 读取最新反馈（非阻塞）
 *   - 主线程调用 Send() 阻塞发送（CAN-FD 单帧 <1ms）
 */

#ifndef ANCNW1_CAN_INTERFACE_HPP
#define ANCNW1_CAN_INTERFACE_HPP

#include <atomic>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace ancn_w1
{

/**
 * @brief CAN-FD 接口封装（Linux socketcan）
 *
 * 用法：
 *   1. 构造 → Open("can0") → StartReceiveThread()
 *   2. 主线程周期性 Send(can_id, data, 64)
 *   3. 主线程 GetLatestFeedback(motor_id, &out) 读最新反馈
 *   4. 析构前 StopReceiveThread() + Close()
 *
 * 注意：
 *   - Open() 失败时需检查 `ip link set can0 up type can fd` 已配置
 *   - 内核需加载 can_raw 模块（默认有）
 */
class CanInterface
{
public:
    /// 电机 ID 范围（手册 §3.2.1：从站 CANID = 0x01~0x06，但 ancnW1 用 1~16 物理编号）
    static constexpr int kMaxMotorId = 16;

    /// CAN-FD 帧最大数据长度（手册 §3.2.1：主机下发 64 字节）
    static constexpr size_t kMaxFrameLen = 64;

    /// 反馈帧长度（手册 §3.2.1：从站返回 12 字节）
    static constexpr size_t kFeedbackLen = 12;

    /// 默认接口名（RK3588 上根据实际 socketcan 网桥名称调整）
    static constexpr const char *kDefaultIfname = "can0";

    CanInterface();
    ~CanInterface();

    // 禁止拷贝：socket fd 不可共享
    CanInterface(const CanInterface &) = delete;
    CanInterface &operator=(const CanInterface &) = delete;

    /**
     * @brief 打开 CAN-FD 接口并绑定
     * @param ifname socketcan 接口名（如 "can0"）
     * @return true=成功，false=失败（权限不足/接口不存在/未配置为 FD）
     *
     * 前置条件（用户态执行一次即可）：
     *   sudo ip link set can0 down
     *   sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
     *
     * 注意：
     *   - 需要 CAP_NET_RAW 权限；通常以 root 启动或给可执行文件 capability
     *   - 接口未配置为 FD 时 send() 会返回 EINVAL
     */
    bool Open(const std::string &ifname = kDefaultIfname);

    /// 关闭接口并停止接收线程
    void Close();

    /// 是否已打开
    bool IsOpen() const { return sock_ >= 0; }

    /**
     * @brief 发送一帧 CAN-FD
     * @param can_id 11 位 CAN ID（标准帧）
     * @param data 数据指针
     * @param len 数据字节数（≤64）；CAN-FD 自动启用当 len>8
     * @return true=成功进入内核队列，false=失败
     *
     * 注意：
     *   - 非阻塞模式（setsockopt(SNDTIMEO) 短超时）；失败时应重试
     *   - 主站帧一般用 CANID=0x00，从站帧 CANID=0x01~0x06（手册 §3.2.1）
     */
    bool Send(uint32_t can_id, const uint8_t *data, size_t len);

    /**
     * @brief 同步接收一帧（带超时）
     * @param[out] can_id 收到的 CAN ID
     * @param[out] data 数据缓冲（≥12 字节）
     * @param[in,out] len 输入=缓冲容量，输出=实际字节数
     * @param timeout_ms 0=非阻塞，>0=阻塞最长时长
     * @return true=收到一帧，false=超时或错误
     *
     * 注意：
     *   - 主要用于诊断/测试；正常运行时使用 StartReceiveThread + GetLatestFeedback
     */
    bool Recv(uint32_t *can_id, uint8_t *data, size_t *len, int timeout_ms);

    /// 启动后台接收线程（持续更新 feedbacks_ 槽位）
    bool StartReceiveThread();

    /// 停止后台接收线程（join）
    void StopReceiveThread();

    /**
     * @brief 获取电机最新反馈（线程安全，无锁轮询版）
     * @param motor_id 电机 ID（1~16）
     * @param[out] out 12 字节反馈（手册 §3.3.2）
     * @return true=收到过该电机反馈，false=从未收到
     *
     * 注意：
     *   - 不带时间戳检查；调用方应自行判断反馈是否新鲜
     *   - 多线程安全：使用 per-slot mutex 保护
     */
    bool GetLatestFeedback(uint8_t motor_id, std::array<uint8_t, kFeedbackLen> *out);

    /// 发送帧计数（调试用）
    uint64_t TxCount() const { return tx_count_.load(); }
    /// 接收帧计数
    uint64_t RxCount() const { return rx_count_.load(); }
    /// 错误计数（socket 错误/超时）
    uint64_t ErrorCount() const { return error_count_.load(); }

private:
    /// 后台接收线程入口
    void ReceiveLoop();

    /// socket fd（-1=未打开）
    int sock_ = -1;

    /// 接口名（用于日志）
    std::string ifname_;

    /// 接收线程运行标志
    std::atomic<bool> running_{false};

    /// 后台接收线程
    std::thread recv_thread_;

    /// 每电机反馈槽（索引 = motor_id，索引 0 不用）
    struct FeedbackSlot
    {
        std::array<uint8_t, kFeedbackLen> data{};  /// 反馈内容
        std::mutex mtx;                            /// 保护 data 与 fresh
        bool fresh = false;                        /// 是否收到过
        std::chrono::steady_clock::time_point ts;  /// 最后更新时间（用于超时检测）
    };
    FeedbackSlot feedbacks_[kMaxMotorId + 1];

    /// 计数器（原子，便于后台线程递增 + 主线程读取）
    std::atomic<uint64_t> tx_count_{0};
    std::atomic<uint64_t> rx_count_{0};
    std::atomic<uint64_t> error_count_{0};
};

}  // namespace ancn_w1

#endif  // ANCNW1_CAN_INTERFACE_HPP
