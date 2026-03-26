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

#include "rclcpp/rclcpp.hpp"

#include "Diagnostics.hpp"
#include "ros2_isobus/msg/isobus_address_status.hpp"

namespace ros2_isobus
{

/*
 *
 * ROS2 wrapper for Diagnostics:
 *  - Declares diagnostics configuration parameters
 *  - Subscribes/publishes ISOBUS frame and TP topics
 *  - Drives ISO 11783-12 diagnostics processing via timer
 *
 */
class DiagnosticsROS2 : public Diagnostics, public rclcpp::Node
{
public:
  explicit DiagnosticsROS2(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void sendFrame(const msg::IsobusFrame & frame) override;
  void sendTpFrame(const msg::IsobusTpFrame & frame) override;
  void printInfo(const std::string & msg) override;
  void printWarn(const std::string & msg) override;

private:
  static std::uint8_t clamp_u8(int v);
  static std::vector<std::uint8_t> parse_byte_list(const std::string & s);
  void load_config();

  bool use_address_manager_sa_{true};
  rclcpp::Publisher<msg::IsobusFrame>::SharedPtr tx_pub_;
  rclcpp::Publisher<msg::IsobusTpFrame>::SharedPtr tx_tp_pub_;
  rclcpp::Subscription<msg::IsobusFrame>::SharedPtr rx_sub_;
  rclcpp::Subscription<msg::IsobusAddressStatus>::SharedPtr addr_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ros2_isobus
