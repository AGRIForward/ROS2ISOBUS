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

#include "AddressManager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_isobus/msg/isobus_address_book.hpp"
#include "ros2_isobus/msg/isobus_address_entry.hpp"
#include "ros2_isobus/msg/isobus_address_status.hpp"
#include "ros2_isobus/topics.hpp"

namespace ros2_isobus
{

/*
 *
 * ROS2 wrapper for AddressManager:
 *  - Declares NAME/SA parameters
 *  - Subscribes/publishes ISOBUS frames and address book/status topics
 *  - Drives ISO 11783-5 address-claim state machine via timer
 *
 */
class AddressManagerROS2 : public AddressManager, public rclcpp::Node
{
public:

  explicit AddressManagerROS2(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // Transport hooks
  void sendFrame(const msg::IsobusFrame &frame) override;
  void emitStatus(NameState state, std::uint8_t sa) override;
  void publishAddressBook(const std::map<ByteArray8, std::uint8_t> & book) override;
  void printInfo(const std::string & msg) override;
  void printWarn(const std::string & msg) override;
  bool busReadyToSend() const override;

  rclcpp::Publisher<ros2_isobus::msg::IsobusAddressStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<ros2_isobus::msg::IsobusAddressBook>::SharedPtr address_book_publisher_;
  rclcpp::Publisher<ros2_isobus::msg::IsobusFrame>::SharedPtr bus_publisher_;
  rclcpp::Subscription<ros2_isobus::msg::IsobusFrame>::SharedPtr bus_subscriber_;
  rclcpp::TimerBase::SharedPtr run_timer_;
};

}  // namespace ros2_isobus
