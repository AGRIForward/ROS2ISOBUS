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

#include "TECUClass3ServerNode.hpp"

#include "ros2_isobus/topics.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace ros2_isobus
{
namespace
{
constexpr std::uint8_t kAddressClaimedState = 4U;
}

TECUClass3ServerROS2::TECUClass3ServerROS2(const rclcpp::NodeOptions & options)
: TECUClass3Server(), Node("tecu_server", options), last_twist_stamp_(now())
{
  Configuration configuration;
  configuration.tecu_class = static_cast<std::uint8_t>(std::clamp<std::int64_t>(
    declare_parameter<int>("tecu_class", 3), 1, 3));
  configuration.enable_guidance = declare_parameter<bool>("enable_guidance", true);
  configuration.enable_cruise = declare_parameter<bool>("enable_cruise", true);
  configuration.enable_rear_hitch = declare_parameter<bool>("enable_rear_hitch", true);
  configuration.enable_rear_pto = declare_parameter<bool>("enable_rear_pto", true);
  configuration.enable_lighting = declare_parameter<bool>("enable_lighting", true);
  configuration.enable_power_management =
    declare_parameter<bool>("enable_power_management", true);
  configuration.aux_valve_count = static_cast<std::uint8_t>(std::clamp<std::int64_t>(
    declare_parameter<int>("aux_valve_count", 8), 0, 16));
  setConfiguration(configuration);
  const auto input_timeout_ms = std::max<std::int64_t>(
    100, declare_parameter<int>("input_timeout_ms", 1000));
  setInputTimeoutTicks(static_cast<unsigned>((input_timeout_ms + 99) / 100));

  bus_pub_ = create_publisher<msg::IsobusFrame>(kBusTxTopic, 100);
  cruise_pub_ = create_publisher<msg::TecuCruiseCommand>(kTECUServerCruiseCommandTopic, 10);
  guidance_pub_ = create_publisher<msg::TecuGuidanceCommand>(kTECUServerCurvatureCommandTopic, 10);
  hitch_pub_ = create_publisher<msg::TecuRearHitchCommand>(kTECUServerRearHitchCommandTopic, 10);
  pto_pub_ = create_publisher<msg::TecuRearPtoCommand>(kTECUServerRearPtoCommandTopic, 10);
  valve_pub_ = create_publisher<msg::AuxValveCommand>(kTECUServerAuxValveCommandTopic, 10);
  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(kTECUServerTwistCommandTopic, 10);

  bus_sub_ = create_subscription<msg::IsobusFrame>(
    kBusRxTopic, 100, [this](const msg::IsobusFrame::SharedPtr frame) {
      if (frame) HandleMsg(*frame);
    });
  address_sub_ = create_subscription<msg::IsobusAddressStatus>(
    kAddressManagerStatus, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&TECUClass3ServerROS2::onAddressStatus, this, std::placeholders::_1));
  twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    kTECUServerTwistMeasurementTopic, rclcpp::SensorDataQoS(),
    std::bind(&TECUClass3ServerROS2::onTwistMeasurement, this, std::placeholders::_1));
  hitch_sub_ = create_subscription<msg::TecuRearHitchStatus>(
    kTECUServerRearHitchStatusTopic, 10,
    [this](const msg::TecuRearHitchStatus::SharedPtr status) {
      if (status) setRearHitchStatus(*status);
    });
  pto_sub_ = create_subscription<msg::TecuRearPtoStatus>(
    kTECUServerRearPtoStatusTopic, 10,
    [this](const msg::TecuRearPtoStatus::SharedPtr status) {
      if (status) setRearPtoStatus(*status);
    });
  valve_sub_ = create_subscription<msg::AuxValveStatus>(
    kTECUServerAuxValveStatusTopic, 10,
    [this](const msg::AuxValveStatus::SharedPtr status) {
      if (status) setAuxValveStatus(*status);
    });
  engine_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
    kTECUServerEngineSpeedTopic, rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float64::SharedPtr value) {
      if (value) setEngineSpeed(value->data);
    });
  maximum_power_time_sub_ = create_subscription<std_msgs::msg::UInt8>(
    kTECUServerMaximumPowerTimeTopic, rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::UInt8::SharedPtr value) {
      if (value) setMaximumPowerTime(value->data);
    });
  key_switch_sub_ = create_subscription<std_msgs::msg::Bool>(
    kTECUServerKeySwitchTopic, rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Bool::SharedPtr value) {
      if (value) setKeySwitch(value->data);
    });
  guidance_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
    kTECUServerGuidanceReadyTopic, rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Bool::SharedPtr value) {
      if (!value) return;
      guidance_ready_ = value->data;
      guidance_ready_received_ = true;
      updateGuidanceAvailability();
    });
  mechanical_lockout_sub_ = create_subscription<std_msgs::msg::Bool>(
    kTECUServerMechanicalLockoutTopic, rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Bool::SharedPtr value) {
      if (!value) return;
      mechanical_lockout_ = value->data;
      mechanical_lockout_received_ = true;
      updateGuidanceAvailability();
    });
  timer_ = create_wall_timer(100ms, [this]() {run();});
  RCLCPP_INFO(
    get_logger(), "TECU Class %u server ready; waiting for AddressManager",
    configuration.tecu_class);
}

void TECUClass3ServerROS2::onAddressStatus(const msg::IsobusAddressStatus & status)
{
  setSourceAddress(status.sa);
  setAddressClaimed(status.state == kAddressClaimedState && status.sa < 0xFEU);
}

void TECUClass3ServerROS2::onTwistMeasurement(const geometry_msgs::msg::TwistStamped & twist)
{
  const auto stamp = now();
  const double elapsed = std::clamp((stamp - last_twist_stamp_).seconds(), 0.0, 1.0);
  last_twist_stamp_ = stamp;
  setTwist(twist.twist.linear.x, twist.twist.angular.z, elapsed);
}

void TECUClass3ServerROS2::updateGuidanceAvailability()
{
  if (guidance_ready_received_ && mechanical_lockout_received_) {
    setGuidanceAvailable(guidance_ready_, mechanical_lockout_);
    guidance_ready_received_ = false;
    mechanical_lockout_received_ = false;
  }
}

void TECUClass3ServerROS2::sendFrame(const msg::IsobusFrame & input)
{
  auto frame = input;
  frame.timestamp = now();
  bus_pub_->publish(frame);
}

void TECUClass3ServerROS2::publishCruiseCommand(const msg::TecuCruiseCommand & value) {cruise_pub_->publish(value);}
void TECUClass3ServerROS2::publishGuidanceCommand(const msg::TecuGuidanceCommand & value) {guidance_pub_->publish(value);}
void TECUClass3ServerROS2::publishRearHitchCommand(const msg::TecuRearHitchCommand & value) {hitch_pub_->publish(value);}
void TECUClass3ServerROS2::publishRearPtoCommand(const msg::TecuRearPtoCommand & value) {pto_pub_->publish(value);}
void TECUClass3ServerROS2::publishAuxValveCommand(const msg::AuxValveCommand & value) {valve_pub_->publish(value);}

void TECUClass3ServerROS2::publishTwistCommand(double speed, double yaw_rate)
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now();
  command.header.frame_id = "base_link";
  command.twist.linear.x = speed;
  command.twist.angular.z = yaw_rate;
  twist_pub_->publish(command);
}

void TECUClass3ServerROS2::printWarn(const std::string & text)
{
  RCLCPP_WARN(get_logger(), "%s", text.c_str());
}

}  // namespace ros2_isobus

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_isobus::TECUClass3ServerROS2>());
  rclcpp::shutdown();
  return 0;
}
