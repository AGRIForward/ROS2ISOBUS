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

#include "ros2_isobus/ISOBUSframe.hpp"
#include "ros2_isobus/topics.hpp"

using namespace std::chrono_literals;

namespace ros2_isobus
{

// ISO 11783-3 SocketCAN bridge: relays between CAN bus and ROS topics.
CanBridge::CanBridge()
    : rclcpp::Node("ISOBUS_can_bridge")
{
    interface_ = declare_parameter<std::string>("interface", "can0");
    disableLoopback_ = declare_parameter<bool>("disable_loopback", true);

    rxPublisher_ = create_publisher<msg::IsobusFrame>(kBusRxTopic, 100);
    txSubscriber_ = create_subscription<msg::IsobusFrame>(
        kBusTxTopic, 200,
        std::bind(&CanBridge::handleTx, this, std::placeholders::_1));

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
    if (socketFd_ < 0)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "CAN socket unavailable");
        return;
    }

    struct can_frame canMsg;
    std::memset(&canMsg, 0, sizeof(canMsg));
    // Always send as 29-bit extended CAN ID
    canMsg.can_id = get_can_id(msg) | CAN_EFF_FLAG;
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
    if (written != static_cast<int>(sizeof(canMsg)))
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to send CAN frame: %s", strerror(errno));
    }
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
                auto isobus_msg = ros2_isobus::make_isobus_frame_from_can(frame.can_id, payload);
                rxPublisher_->publish(isobus_msg);
           }
        }
    }
}

}  // namespace ros2_isobus
