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

#include "TECUClass3Client.hpp"
#include "TECUClass2ClientNode.hpp"
#include "ros2_isobus/msg/isobus_address_book.hpp"
#include "ros2_isobus/msg/isobus_address_status.hpp"
#include "ros2_isobus/msg/tecu_cruise_command.hpp"
#include "ros2_isobus/msg/tecu_guidance_command.hpp"
#include "ros2_isobus/msg/tecu_rear_hitch_command.hpp"
#include "ros2_isobus/msg/tecu_rear_pto_command.hpp"
#include "ros2_isobus/msg/aux_valve_command.hpp"
#include "ros2_isobus/msg/aux_valve_status.hpp"

namespace ros2_isobus
{

/*
 *
 * TECUClass3ClientROS2: ROS 2 node for legacy Class3 guidance/cruise (ISO 11783-7:2015)
 *  - Inherits Class2 wiring from TECUBaseROS2
 *  - Adds guidance/cruise/hydraulic pub/subs and address manager integration
 *
 */
class TECUClass3ClientROS2 : public TECUBaseROS2<TECUClass3Client>
{
public:
    TECUClass3ClientROS2();

private:
    void publishSteeringWheel() override
    {
        if (steering_wheel_pub_) steering_wheel_pub_->publish(this->steering_wheel_data_);
    }
    void publishValveStatus() override
    {
        if (steering_valve_status_pub_) steering_valve_status_pub_->publish(this->steering_valve_status_data_);
    }
    void publishGuidanceStatus() override
    {
        if (guidance_status_pub_) guidance_status_pub_->publish(this->guidance_status_data_);
    }
    void publishCruiseStatus() override
    {
        if (cruise_status_pub_) cruise_status_pub_->publish(this->cruise_status_data_);
    }
    void publishAuxValveStatus(std::uint8_t valve_id, const ValveStatus & status) override
    {
        if (!valve_status_pub_) return;
        msg::AuxValveStatus msg;
        msg.valve_number = valve_id;
        msg.extend_flow_percent = status.extend_flow_percent;
        msg.retract_flow_percent = status.retract_flow_percent;
        msg.state = status.state;
        msg.failsafe = status.failsafe;
        valve_status_pub_->publish(msg);
    }
    void sendFrame(const msg::IsobusFrame & f) override
    {
        if (bus_pub_) bus_pub_->publish(f);
    }
    void printInfo(const std::string & s) override
    {
        RCLCPP_INFO(get_logger(), "%s", s.c_str());
    }
    void printWarn(const std::string & s) override
    {
        RCLCPP_WARN(get_logger(), "%s", s.c_str());
    }

    rclcpp::Publisher<msg::TecuSteeringWheel>::SharedPtr steering_wheel_pub_;
    rclcpp::Publisher<msg::TecuSteeringValveStatus>::SharedPtr steering_valve_status_pub_;
    rclcpp::Publisher<msg::TecuGuidanceStatus>::SharedPtr guidance_status_pub_;
    rclcpp::Publisher<msg::TecuCruiseStatus>::SharedPtr cruise_status_pub_;
    rclcpp::Publisher<msg::AuxValveStatus>::SharedPtr valve_status_pub_;
    rclcpp::Publisher<msg::IsobusFrame>::SharedPtr bus_pub_;

    rclcpp::Subscription<msg::IsobusAddressBook>::SharedPtr address_book_sub_;
    rclcpp::Subscription<msg::IsobusAddressStatus>::SharedPtr address_status_sub_;
    rclcpp::Subscription<msg::TecuCruiseCommand>::SharedPtr cruise_command_sub_;
    rclcpp::Subscription<msg::TecuGuidanceCommand>::SharedPtr curvature_command_sub_;
    rclcpp::Subscription<msg::TecuRearHitchCommand>::SharedPtr rear_hitch_command_sub_;
    rclcpp::Subscription<msg::TecuRearPtoCommand>::SharedPtr rear_pto_command_sub_;
    rclcpp::Subscription<msg::AuxValveCommand>::SharedPtr valve_command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_sub_;
    rclcpp::Subscription<msg::IsobusFrame>::SharedPtr bus_sub_;
    rclcpp::TimerBase::SharedPtr run_timer_;
    bool my_sa_known_{false};
};

}  // namespace ros2_isobus
