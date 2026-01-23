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
#include <string>
#include <thread>

#include <linux/can.h>

#include "rclcpp/rclcpp.hpp"

#include "ros2_isobus/ISOBUSframe.hpp"
#include "ros2_isobus/msg/isobus_frame.hpp"

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
    void readLoop();                       // Blocking read/publish loop.

    std::string interface_;
    bool disableLoopback_ = true;
    std::atomic<bool> running_{true};
    int socketFd_ = -1;
    std::thread readerThread_;

    rclcpp::Publisher<msg::IsobusFrame>::SharedPtr rxPublisher_;
    rclcpp::Subscription<msg::IsobusFrame>::SharedPtr txSubscriber_;
};

}  // namespace ros2_isobus
