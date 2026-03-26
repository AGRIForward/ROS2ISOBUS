/*
 *  This file is part of ROS2ISOBUS
 *
 *  Copyright 2026 Juha Backman / Natural Resources Institute Finland
 *
 *  ROS2ISOBUS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  ROS2ISOBUS is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with ROS2ISOBUS.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#include "CanBridge.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <functional>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "ros2_isobus/topics.hpp"

using namespace std::chrono_literals;

namespace ros2_isobus
{
namespace
{

std::uint32_t make_can_id(const msg::IsobusFrame & frame)
{
    const std::uint8_t pf = frame.pf ? frame.pf : static_cast<std::uint8_t>((frame.pgn >> 8) & 0xFF);
    const std::uint8_t ps = frame.ps ? frame.ps : static_cast<std::uint8_t>(frame.pgn & 0xFF);
    const std::uint8_t page = static_cast<std::uint8_t>(frame.page ? 1 : 0);

    return (static_cast<std::uint32_t>(frame.priority) << 26) |
           (static_cast<std::uint32_t>(page) << 24) |
           (static_cast<std::uint32_t>(pf) << 16) |
           (static_cast<std::uint32_t>(ps) << 8) |
           (frame.sa);
}

msg::IsobusFrame make_isobus_frame_from_can(std::uint32_t id, const std::array<std::uint8_t, 8> & payload)
{
    msg::IsobusFrame out;
    out.sa = static_cast<std::uint8_t>((id >> 0) & 0xFF);
    out.pgn = static_cast<std::uint32_t>((id >> 8) & 0xFFFF);
    out.page = static_cast<bool>((id >> 24) & 0x01);
    out.priority = static_cast<std::uint8_t>((id >> 26) & 0x07);
    out.pf = static_cast<std::uint8_t>((out.pgn >> 8) & 0xFF);
    out.ps = static_cast<std::uint8_t>((out.pgn >> 0) & 0xFF);
    out.data = payload;
    return out;
}

}  // namespace

// ISO 11783-3 SocketCAN bridge: relays between CAN bus and ROS topics.
CanBridge::CanBridge()
    : rclcpp::Node("ISOBUS_can_bridge")
{
    interface_ = declare_parameter<std::string>("interface", "can0");
    disableLoopback_ = declare_parameter<bool>("disable_loopback", true);
    const bool enable_tp = declare_parameter<bool>("enable_tp", true);
    const bool enable_etp = declare_parameter<bool>("enable_etp", true);
    const bool tp_strict_cts_timing = declare_parameter<bool>("tp_strict_cts_timing", false);
    const auto tp_rx_timeout_ms = static_cast<std::uint32_t>(declare_parameter<int>("tp_rx_timeout_ms", 1000));
    const auto tp_tx_timeout_ms = static_cast<std::uint32_t>(declare_parameter<int>("tp_tx_timeout_ms", 1000));
    const auto tp_tx_window_default = static_cast<std::uint16_t>(declare_parameter<int>("tp_tx_window_size_default", 8));
    const auto tp_local_sa = static_cast<std::uint8_t>(declare_parameter<int>("tp_local_sa", 0xFF));

    rxPublisher_ = create_publisher<msg::IsobusFrame>(kBusRxTopic, 100);
    rxTpPublisher_ = create_publisher<msg::IsobusTpFrame>(kBusRxTpTopic, 10);
    txSubscriber_ = create_subscription<msg::IsobusFrame>(
        kBusTxTopic, 200,
        std::bind(&CanBridge::handleTx, this, std::placeholders::_1));
    txTpSubscriber_ = create_subscription<msg::IsobusTpFrame>(
        kBusTxTpTopic, 10,
        std::bind(&CanBridge::handleTpTx, this, std::placeholders::_1));

    TransportProtocol::Params tp_params;
    tp_params.enable_tp = enable_tp;
    tp_params.enable_etp = enable_etp;
    tp_params.strict_cts_timing = tp_strict_cts_timing;
    tp_params.rx_timeout_ms = tp_rx_timeout_ms;
    tp_params.tx_timeout_ms = tp_tx_timeout_ms;
    tp_params.tx_window_default = tp_tx_window_default;
    tp_params.local_sa = tp_local_sa;

    tp_ = std::make_unique<TransportProtocol>(
        tp_params,
        [this](const msg::IsobusFrame & f) { sendFrame(f); },
        [this](const msg::IsobusTpFrame & f) { if (rxTpPublisher_) rxTpPublisher_->publish(f); },
        get_logger(),
        get_clock());

    const auto latched_reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    addressStatusSubscriber_ = create_subscription<msg::IsobusAddressStatus>(
        kAddressManagerStatus,
        latched_reliable_qos,
        [this](const msg::IsobusAddressStatus::SharedPtr msg)
        {
            if (!msg || !tp_)
                return;
            if (msg->sa > 0xFDU)
            {
                RCLCPP_WARN(get_logger(), "Ignoring invalid SA from address manager status: 0x%02X", msg->sa);
                return;
            }
            tp_->setSA(msg->sa);
        });

    tpTimer_ = create_wall_timer(
        100ms,
        [this]()
        {
            if (tp_)
                tp_->tick();
        });

    openSocket();
    readerThread_ = std::thread([this]() { readLoop(); });
}

CanBridge::~CanBridge()
{
    running_.store(false);
    if (socketFd_ >= 0)
    {
        close(socketFd_);
        socketFd_ = -1;
    }
    if (readerThread_.joinable())
    {
        readerThread_.join();
    }
}

// Open and bind a SocketCAN raw socket for the configured interface.
void CanBridge::openSocket()
{
    socketFd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketFd_ < 0)
    {
        RCLCPP_FATAL(get_logger(), "Failed to open CAN socket: %s", strerror(errno));
        return;
    }

    if (disableLoopback_)
    {
        int loopback = 0;
        setsockopt(socketFd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socketFd_, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_FATAL(get_logger(), "Interface %s not found: %s", interface_.c_str(), strerror(errno));
        close(socketFd_);
        socketFd_ = -1;
        return;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socketFd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_FATAL(get_logger(), "Failed to bind CAN socket on %s: %s", interface_.c_str(), strerror(errno));
        close(socketFd_);
        socketFd_ = -1;
        return;
    }

    RCLCPP_INFO(get_logger(), "SocketCAN bridge bound to %s", interface_.c_str());
}

// Send an IsobusFrame out on SocketCAN.
void CanBridge::handleTx(const msg::IsobusFrame &msg)
{
    sendFrame(msg);
}

void CanBridge::handleTpTx(const msg::IsobusTpFrame &msg)
{
    if (tp_)
        tp_->handleTxRequest(msg);
}

void CanBridge::sendFrame(const msg::IsobusFrame &msg)
{
    if (socketFd_ < 0)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "CAN socket unavailable");
        return;
    }

    std::lock_guard<std::mutex> lk(socketMutex_);

    struct can_frame canMsg;
    std::memset(&canMsg, 0, sizeof(canMsg));
    // Always send as 29-bit extended CAN ID
    canMsg.can_id = make_can_id(msg) | CAN_EFF_FLAG;
    canMsg.can_dlc = 8;
    for (std::size_t i = 0; i < msg.data.size(); ++i)
    {
        canMsg.data[i] = msg.data[i];
    }

    /*
    RCLCPP_INFO(get_logger(),
                "TX CAN ID 0x%08X pri:%u pf:0x%02X ps:0x%02X sa:0x%02X data:%02X %02X %02X %02X %02X %02X %02X %02X",
                canMsg.can_id,
                static_cast<unsigned>(msg.priority),
                static_cast<unsigned>(msg.pf ? msg.pf : (msg.pgn >> 8) & 0xFF),
                static_cast<unsigned>(msg.ps ? msg.ps : msg.pgn & 0xFF),
                static_cast<unsigned>(msg.sa),
                canMsg.data[0], canMsg.data[1], canMsg.data[2], canMsg.data[3],
                canMsg.data[4], canMsg.data[5], canMsg.data[6], canMsg.data[7]);
    */

    const auto written = write(socketFd_, &canMsg, sizeof(canMsg));
    ++tx_attempts_;
    if (written != static_cast<int>(sizeof(canMsg)))
    {
        ++tx_fail_;
        if (written >= 0) {
            ++tx_short_write_;
        }
        tx_last_errno_ = errno;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "CAN TX failed: if=%s can_id=0x%08X pgn=0x%06X sa=0x%02X pf=0x%02X ps=0x%02X "
            "written=%d errno=%d(%s) attempts=%llu ok=%llu fail=%llu short=%llu",
            interface_.c_str(),
            static_cast<unsigned>(canMsg.can_id),
            static_cast<unsigned>(msg.pgn & 0x3FFFFU),
            static_cast<unsigned>(msg.sa),
            static_cast<unsigned>(msg.pf ? msg.pf : ((msg.pgn >> 8) & 0xFF)),
            static_cast<unsigned>(msg.ps ? msg.ps : (msg.pgn & 0xFF)),
            static_cast<int>(written),
            tx_last_errno_,
            strerror(tx_last_errno_),
            static_cast<unsigned long long>(tx_attempts_),
            static_cast<unsigned long long>(tx_ok_),
            static_cast<unsigned long long>(tx_fail_),
            static_cast<unsigned long long>(tx_short_write_));
        return;
    }
    ++tx_ok_;
}

// Blocking read loop to receive CAN frames and publish as IsobusFrame.
void CanBridge::readLoop()
{
    const auto backoff = 500ms;
    while (rclcpp::ok() && running_.load())
    {
        if (socketFd_ < 0)
        {
            std::this_thread::sleep_for(backoff);
            continue;
        }

        struct pollfd pfd;
        std::memset(&pfd, 0, sizeof(pfd));
        pfd.fd = socketFd_;
        pfd.events = POLLIN;

        const int pollResult = poll(&pfd, 1, 200);
        if (pollResult < 0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "CAN poll error: %s", strerror(errno));
            continue;
        }
        else if (pollResult == 0)
        {
            continue;
        }

        if (pfd.revents & POLLIN)
        {
            struct can_frame frame;
            const auto bytes = read(socketFd_, &frame, sizeof(frame));
            if (bytes == static_cast<int>(sizeof(frame)))
            {
                std::array<std::uint8_t, 8> payload;
                std::copy(std::begin(frame.data), std::end(frame.data), payload.begin());
                auto isobus_msg = make_isobus_frame_from_can(frame.can_id, payload);
                rxPublisher_->publish(isobus_msg);
                if (tp_)
                    tp_->handleRx(isobus_msg);
            }
        }
    }
}

}  // namespace ros2_isobus
