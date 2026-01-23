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

// Entry point for ISO 11783-5 address claim node.
#include "rclcpp/rclcpp.hpp"
#include "AddressManagerNode.hpp"

namespace ros2_isobus
{

AddressManagerROS2::AddressManagerROS2(const rclcpp::NodeOptions &options)
: AddressManager(), rclcpp::Node("ISOBUS_address_manager_node", options)
{
  this->declare_parameter<std::string>("ecu_name_hex", "0100E0FF00820180");
  this->declare_parameter<int>("preferred_address", 0x1C);
  const auto ecu_hex = this->get_parameter("ecu_name_hex").as_string();
  isobusName_ = ros2_isobus::hex_to_byte_array(ecu_hex);
  address_ = static_cast<std::uint8_t>(this->get_parameter("preferred_address").as_int());

  const auto latchedReliableQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  status_publisher_ = create_publisher<ros2_isobus::msg::IsobusAddressStatus>(
    ros2_isobus::kAddressManagerStatus, latchedReliableQos);
  address_book_publisher_ = create_publisher<ros2_isobus::msg::IsobusAddressBook>(
    ros2_isobus::kAddressManagerAddressBook, latchedReliableQos);
  bus_publisher_ = create_publisher<ros2_isobus::msg::IsobusFrame>(ros2_isobus::kBusTxTopic, 50);
  bus_subscriber_ = create_subscription<ros2_isobus::msg::IsobusFrame>(
    ros2_isobus::kBusRxTopic, 100,
    std::bind(&AddressManager::handleMsg, this, std::placeholders::_1));

  run_timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&AddressManager::run, this));

  ensureSelfEntry();
  printInfo("AddressManager initialised (ECU " + ecu_hex + " SA 0x" + to_hex(address_) + ")");
}

void AddressManagerROS2::sendFrame(const msg::IsobusFrame &frame)
{
  if (bus_publisher_) {
    bus_publisher_->publish(frame);
  }
}

void AddressManagerROS2::emitStatus(NameState state, std::uint8_t sa)
{
  if (!status_publisher_) {
    return;
  }
  ros2_isobus::msg::IsobusAddressStatus status_msg;
  status_msg.sa = sa;
  status_msg.state = static_cast<std::uint8_t>(state);
  status_publisher_->publish(status_msg);
}

void AddressManagerROS2::publishAddressBook(const std::map<ByteArray8, std::uint8_t> & book)
{
  if (!address_book_publisher_) {
    return;
  }
  ros2_isobus::msg::IsobusAddressBook msg;
  msg.entries.reserve(book.size());
  for (const auto &entry : book) {
    ros2_isobus::msg::IsobusAddressEntry entry_msg;
    entry_msg.name = entry.first;
    entry_msg.sa = entry.second;
    msg.entries.push_back(entry_msg);
  }
  address_book_publisher_->publish(msg);
}

void AddressManagerROS2::printInfo(const std::string & msg)
{
  RCLCPP_INFO(get_logger(), "%s", msg.c_str());
}

void AddressManagerROS2::printWarn(const std::string & msg)
{
  RCLCPP_WARN(get_logger(), "%s", msg.c_str());
}

bool AddressManagerROS2::busReadyToSend() const
{
  return bus_publisher_ && bus_publisher_->get_subscription_count() > 0;
}

}  // namespace ros2_isobus

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_isobus::AddressManagerROS2>());
  rclcpp::shutdown();
  return 0;
}
