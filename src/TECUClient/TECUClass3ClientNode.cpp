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

#include <algorithm>
#include <cmath>

#include "TECUClass3ClientNode.hpp"

using namespace std::chrono_literals;

namespace ros2_isobus
{
namespace
{

ByteArray8 hex_to_byte_array(const std::string & hex)
{
    ByteArray8 result{};
    std::size_t idx = 0;
    for (std::size_t i = 0; i < result.size() && (idx + 1) < hex.size(); ++i, idx += 2) {
        const auto byte = hex.substr(idx, 2);
        result[i] = static_cast<std::uint8_t>(std::stoul(byte, nullptr, 16));
    }
    return result;
}

}  // namespace

TECUClass3ClientROS2::TECUClass3ClientROS2()
: TECUBaseROS2<TECUClass3Client>("ISOBUS_tecu_node")
{
    const int valve_count = declare_parameter<int>("valve_count", 8);
    set_valve_count(valve_count);

    const int periodic_timeout = declare_parameter<int>("periodic_timeout_ticks", kPeriodicTimeoutTicks);
    setPeriodicTimeoutTicks(periodic_timeout);

    const auto cmd_mode_str = declare_parameter<std::string>("command_mode", "periodic");
    auto mode_lower = cmd_mode_str;
    std::transform(mode_lower.begin(), mode_lower.end(), mode_lower.begin(), ::tolower);
    CommandMode mode = CommandMode::PERIODIC;
    if (mode_lower == "direct") mode = CommandMode::DIRECT;
    else if (mode_lower == "both") mode = CommandMode::BOTH;
    set_command_mode(mode);

    const auto esp_hex = declare_parameter<std::string>("esp_name_hex", "A0001900AAA00006");
    set_esp_name(hex_to_byte_array(esp_hex));
    set_esp_sa(static_cast<std::uint8_t>(declare_parameter<int>("esp_sa", 0x13)));

    steering_wheel_pub_ = create_publisher<msg::TecuSteeringWheel>(kTECUSteeringWheelTopic, 10);
    steering_valve_status_pub_ = create_publisher<msg::TecuSteeringValveStatus>(kTECUSteeringValveStatusTopic, 10);
    guidance_status_pub_ = create_publisher<msg::TecuGuidanceStatus>(kTECUGuidanceStatusTopic, 10);
    cruise_status_pub_ = create_publisher<msg::TecuCruiseStatus>(kTECUCruiseStatusTopic, 10);
    valve_status_pub_ = create_publisher<msg::AuxValveStatus>(kAuxValveStatusTopic, 10);
    bus_pub_ = create_publisher<msg::IsobusFrame>(kBusTxTopic, 50);

    const auto latchedReliableQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    address_book_sub_ = create_subscription<msg::IsobusAddressBook>(
        kAddressManagerAddressBook, latchedReliableQos,
        [this](const msg::IsobusAddressBook::SharedPtr msg)
        {
            if (!msg || !my_sa_known_) return;
            for (const auto &entry : msg->entries)
            {
                if (entry.name == guidance_name() && guidance_sa() != entry.sa && entry.sa != my_sa())
                {
                    set_esp_sa(entry.sa);
                    RCLCPP_INFO(this->get_logger(), "Updated ESP SA from address book: 0x%02X", entry.sa);
                }
            }
        });

    address_status_sub_ = create_subscription<msg::IsobusAddressStatus>(
        kAddressManagerStatus, latchedReliableQos,
        [this](const msg::IsobusAddressStatus::SharedPtr msg)
        {
            if (!msg) return;
            if (!my_sa_known_ || my_sa() != msg->sa)
            {
                set_my_sa(msg->sa);
                my_sa_known_ = true;
                RCLCPP_INFO(this->get_logger(), "Updated my SA from address manager status: 0x%02X", msg->sa);
            }
        });

    cruise_command_sub_ = create_subscription<msg::TecuCruiseCommand>(
        kTECUCruiseCommandTopic, 10,
        [this](const msg::TecuCruiseCommand::SharedPtr msg)
        {
            if (msg)
            {
                setSpeed(msg->speed);
                setMaxSpeed(msg->max_speed);
            }
        });
    curvature_command_sub_ = create_subscription<msg::TecuGuidanceCommand>(
        kTECUCurvatureCommandTopic, 10,
        [this](const msg::TecuGuidanceCommand::SharedPtr msg)
        {
            if (msg)
                setCurvature(msg->curvature);
        });
    rear_hitch_command_sub_ = create_subscription<msg::TecuRearHitchCommand>(
        kTECURearHitchCommandTopic, 10,
        [this](const msg::TecuRearHitchCommand::SharedPtr msg)
        {
            if (msg)
                setRearHitch(msg->position_percent);
        });
    rear_pto_command_sub_ = create_subscription<msg::TecuRearPtoCommand>(
        kTECURearPtoCommandTopic, 10,
        [this](const msg::TecuRearPtoCommand::SharedPtr msg)
        {
            if (msg)
                setPTO(msg->rpm, msg->engagement);
        });
    twist_command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        kTECUTwistCommandTopic, 10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
            if (!msg) return;
            const double v = msg->twist.linear.x;
            const double omega = msg->twist.angular.z;
            setSpeed(v);
            if (std::abs(v) > 1e-6)
                setCurvature(omega / v);
            else
                setCurvature(guidance_status().measured_curvature);
        });
    valve_command_sub_ = create_subscription<msg::AuxValveCommand>(
        kAuxValveCommandTopic, 10,
        [this](const msg::AuxValveCommand::SharedPtr msg)
        {
            if (!msg) return;
            setValve(msg->valve_number, msg->flow_percent, msg->floating, msg->failsafe);
        });

    bus_sub_ = create_subscription<msg::IsobusFrame>(
        kBusRxTopic, 100,
        [this](const msg::IsobusFrame::SharedPtr m)
        {
            if (m) HandleMsg(*m);
        });

    run_timer_ = create_wall_timer(100ms, [this](){ run(); });

    RCLCPP_INFO(this->get_logger(), "TECU Class3 node ready");
}

}  // namespace ros2_isobus
