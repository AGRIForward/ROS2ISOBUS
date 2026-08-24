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

#include "TECUClass3Server.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_isobus/msg/isobus_address_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace ros2_isobus
{

/*
 *
 * TECUClass3ServerROS2 binds the configurable T-ECU server to ROS 2:
 *  - Receives the claimed source address from AddressManager
 *  - Publishes and subscribes ISOBUS frames through CanBridge topics
 *  - Maps tractor input status and decoded implement commands to ROS topics
 *
 */
class TECUClass3ServerROS2 : public TECUClass3Server, public rclcpp::Node
{
public:
  explicit TECUClass3ServerROS2(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  // Transport and command-publication hooks used by TECUClass3Server.
  void sendFrame(const msg::IsobusFrame & frame) override;
  void publishCruiseCommand(const msg::TecuCruiseCommand & command) override;
  void publishGuidanceCommand(const msg::TecuGuidanceCommand & command) override;
  void publishRearHitchCommand(const msg::TecuRearHitchCommand & command) override;
  void publishRearPtoCommand(const msg::TecuRearPtoCommand & command) override;
  void publishAuxValveCommand(const msg::AuxValveCommand & command) override;
  void publishTwistCommand(double speed_m_s, double yaw_rate_rad_s) override;
  void printWarn(const std::string & text) override;

private:
  // ROS input callbacks.
  void onAddressStatus(const msg::IsobusAddressStatus & status);
  void onTwistMeasurement(const geometry_msgs::msg::TwistStamped & twist);
  void updateGuidanceAvailability();

  // ISOBUS and decoded command publishers.
  rclcpp::Publisher<msg::IsobusFrame>::SharedPtr bus_pub_;
  rclcpp::Publisher<msg::TecuCruiseCommand>::SharedPtr cruise_pub_;
  rclcpp::Publisher<msg::TecuGuidanceCommand>::SharedPtr guidance_pub_;
  rclcpp::Publisher<msg::TecuRearHitchCommand>::SharedPtr hitch_pub_;
  rclcpp::Publisher<msg::TecuRearPtoCommand>::SharedPtr pto_pub_;
  rclcpp::Publisher<msg::AuxValveCommand>::SharedPtr valve_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  // Address, bus and physical tractor input subscriptions.
  rclcpp::Subscription<msg::IsobusFrame>::SharedPtr bus_sub_;
  rclcpp::Subscription<msg::IsobusAddressStatus>::SharedPtr address_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<msg::TecuRearHitchStatus>::SharedPtr hitch_sub_;
  rclcpp::Subscription<msg::TecuRearPtoStatus>::SharedPtr pto_sub_;
  rclcpp::Subscription<msg::AuxValveStatus>::SharedPtr valve_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr engine_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr maximum_power_time_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr key_switch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr guidance_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mechanical_lockout_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_twist_stamp_;
  bool guidance_ready_{false};
  bool mechanical_lockout_{false};
  bool guidance_ready_received_{false};
  bool mechanical_lockout_received_{false};
};

}  // namespace ros2_isobus
