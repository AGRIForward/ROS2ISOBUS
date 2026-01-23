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

#include "NMEA2000Client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_isobus/topics.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace ros2_isobus
{

/*
 *
 * NMEA2000ClientROS2 binds the protocol parser to ROS 2 (rclcpp::Node):
 *  - Receives ISOBUS frames from kBusRxTopic
 *  - Publishes NavSatFix, TwistStamped, Vector3Stamped, DiagnosticArray
 *  - Exposes rclcpp time/logging for the transport-agnostic base class
 *
 */
class NMEA2000ClientROS2 : public NMEA2000Client, public rclcpp::Node
{
public:
    explicit NMEA2000ClientROS2(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    // Transport hooks
    builtin_interfaces::msg::Time nowStamp() const override { return this->now(); }
    void printInfo(const std::string & msg) override { RCLCPP_INFO(get_logger(), "%s", msg.c_str()); }
    void printWarn(const std::string & msg) override { RCLCPP_WARN(get_logger(), "%s", msg.c_str()); }
    void publishNavSatFix() override;
    void publishCogSog() override;
    void publishAttitude() override;
    void publishPseudoNoise() override;
    void publishRapidPosition() override;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cog_sog_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pseudo_noise_pub_;
    rclcpp::Subscription<msg::IsobusFrame>::SharedPtr bus_sub_;
};

}  // namespace ros2_isobus
