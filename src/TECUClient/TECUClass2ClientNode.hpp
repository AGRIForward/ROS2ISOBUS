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

#include "TECUClass2Client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_isobus/topics.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ros2_isobus/msg/isobus_frame.hpp"

namespace ros2_isobus
{

/*
 *
 * TECUBaseROS2 binds the Class2-level TECU logic to ROS 2:
 *  - Subscribes bus_rx and feeds frames to the logic HandleMsg
 *  - Publishes wheel/ground speed, hitch, PTO and twist measurements
 *  - Provides time/error logging for the transport-agnostic base
 *
 */
template <typename LogicT>
class TECUBaseROS2 : public LogicT, public rclcpp::Node
{
public:
    explicit TECUBaseROS2(const std::string & node_name)
    : LogicT(), rclcpp::Node(node_name)
    {
        wheel_speed_pub_ = this->create_publisher<msg::TecuWheelSpeed>(
            kTECUWheelSpeedTopic, 10);
        ground_speed_pub_ = this->create_publisher<msg::TecuGroundSpeed>(
            kTECUGroundSpeedTopic, 10);
        rear_hitch_pub_ = this->create_publisher<msg::TecuRearHitchStatus>(
            kTECURearHitchTopic, 10);
        rear_pto_pub_ = this->create_publisher<msg::TecuRearPtoStatus>(
            kTECURearPtoTopic, 10);
        twist_measured_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            kTECUTwistMeasurementTopic, 10);
        bus_sub_ = this->create_subscription<msg::IsobusFrame>(
            kBusRxTopic, 100,
            [this](const msg::IsobusFrame::SharedPtr m)
            {
                if (m) this->HandleMsg(*m);
            });
    }

protected:
    builtin_interfaces::msg::Time nowStamp() const override { return this->now(); }
    void printError(const std::string & msg) override { RCLCPP_FATAL(this->get_logger(), "%s", msg.c_str()); }
    void publishWheelSpeed() override
    {
        if (wheel_speed_pub_) wheel_speed_pub_->publish(this->wheel_speed_data_);
    }
    void publishGroundSpeed() override
    {
        if (ground_speed_pub_) ground_speed_pub_->publish(this->ground_speed_data_);
    }
    void publishRearHitchStatus() override
    {
        if (rear_hitch_pub_) rear_hitch_pub_->publish(this->rear_hitch_data_);
    }
    void publishRearPtoStatus() override
    {
        if (rear_pto_pub_) rear_pto_pub_->publish(this->rear_pto_data_);
    }
    void publishTwistMeasurement() override
    {
        if (!twist_measured_pub_) return;
        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = this->now();
        twist.header.frame_id = "base_link";
        twist.twist.linear.x = this->wheel_speed_data_.speed_ms;
        twist.twist.angular.z = this->measured_curvature() * this->ground_speed_data_.speed_ms;
        twist_measured_pub_->publish(twist);
    }

    rclcpp::Publisher<msg::TecuWheelSpeed>::SharedPtr wheel_speed_pub_;
    rclcpp::Publisher<msg::TecuGroundSpeed>::SharedPtr ground_speed_pub_;
    rclcpp::Publisher<msg::TecuRearHitchStatus>::SharedPtr rear_hitch_pub_;
    rclcpp::Publisher<msg::TecuRearPtoStatus>::SharedPtr rear_pto_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_measured_pub_;
    rclcpp::Subscription<msg::IsobusFrame>::SharedPtr bus_sub_;
};

/*
 *
 * TECUClass2ClientROS2: ROS 2 node wrapper for Class2 TECU (ISO 11783-7/9)
 *  - Uses TECUBaseROS2 for common pub/sub wiring
 *  - Publishes Class2 statuses and twist measurement
 *
 */
class TECUClass2ClientROS2 : public TECUBaseROS2<TECUClass2Client>
{
public:
    TECUClass2ClientROS2();
};

}  // namespace ros2_isobus
