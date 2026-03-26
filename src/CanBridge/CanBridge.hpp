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

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <linux/can.h>

#include "rclcpp/rclcpp.hpp"

#include "ros2_isobus/msg/isobus_address_status.hpp"
#include "ros2_isobus/msg/isobus_frame.hpp"
#include "ros2_isobus/msg/isobus_tp_frame.hpp"
#include "TransportProtocol.hpp"

namespace ros2_isobus
{

/*
 *
 * CanBridge maps between SocketCAN and ROS IsobusFrame topic streams.
 * Implements ISO 11783-3 physical layer access on Linux SocketCAN:
 *  - Subscribes to TX topic and writes CAN frames out
 *  - Listens on SocketCAN and publishes received frames
 *
 */
class CanBridge : public rclcpp::Node
{
public:
    CanBridge();
    ~CanBridge() override;

private:
    void openSocket();                     // Initialize and bind SocketCAN interface.
    void handleTx(const msg::IsobusFrame &msg); // Send outgoing frames to the CAN bus.
    void handleTpTx(const msg::IsobusTpFrame &msg); // Send TP/ETP payloads.
    void sendFrame(const msg::IsobusFrame & msg); // Thread-safe CAN write.
    void readLoop();                       // Blocking read/publish loop.

    std::string interface_;
    bool disableLoopback_ = true;
    std::atomic<bool> running_{true};
    int socketFd_ = -1;
    std::thread readerThread_;
    std::mutex socketMutex_;

    rclcpp::Publisher<msg::IsobusFrame>::SharedPtr rxPublisher_;
    rclcpp::Publisher<msg::IsobusTpFrame>::SharedPtr rxTpPublisher_;
    rclcpp::Subscription<msg::IsobusFrame>::SharedPtr txSubscriber_;
    rclcpp::Subscription<msg::IsobusTpFrame>::SharedPtr txTpSubscriber_;
    rclcpp::Subscription<msg::IsobusAddressStatus>::SharedPtr addressStatusSubscriber_;
    rclcpp::TimerBase::SharedPtr tpTimer_;

    std::unique_ptr<TransportProtocol> tp_;
    // TX diagnostics.
    std::uint64_t tx_attempts_ = 0;
    std::uint64_t tx_ok_ = 0;
    std::uint64_t tx_fail_ = 0;
    std::uint64_t tx_short_write_ = 0;
    int tx_last_errno_ = 0;
};

}  // namespace ros2_isobus
