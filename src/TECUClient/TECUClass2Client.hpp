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

#ifndef ROS2_ISOBUS_TECU_CLASS2CLIENT_HPP
#define ROS2_ISOBUS_TECU_CLASS2CLIENT_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "ros2_isobus/msg/isobus_frame.hpp"
#include "ros2_isobus/msg/tecu_ground_speed.hpp"
#include "ros2_isobus/msg/tecu_rear_hitch_status.hpp"
#include "ros2_isobus/msg/tecu_rear_pto_status.hpp"
#include "ros2_isobus/msg/tecu_wheel_speed.hpp"
#include "ros2_isobus/topics.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace ros2_isobus
{

using ByteArray8 = std::array<std::uint8_t, 8>;

/*
 *
 * TECUClass2Client parses ISO 11783-7/9 Class 2 TECU PGNs:
 *  - Wheel/Ground speed, hitch and PTO status to custom ROS messages
 *  - Publishes a TwistStamped measurement (speed + derived curvature)
 *
 */
class TECUClass2Client
{
public:
    TECUClass2Client();
    virtual ~TECUClass2Client() = default;

    // Public aliases and accessors for parsed status
    using WheelSpeedData = msg::TecuWheelSpeed;
    using GroundSpeedData = msg::TecuGroundSpeed;
    using RearHitchStatus = msg::TecuRearHitchStatus;
    using RearPTOStatus = msg::TecuRearPtoStatus;

    const WheelSpeedData & wheel_speed() const { return wheel_speed_data_; }
    const GroundSpeedData & ground_speed() const { return ground_speed_data_; }
    const RearHitchStatus & rear_hitch() const { return rear_hitch_data_; }
    const RearPTOStatus & rear_pto() const { return rear_pto_data_; }

    void HandleMsg(const msg::IsobusFrame & imsg);

protected:
    // Transport hooks implemented by ROS2 wrapper
    virtual void publishWheelSpeed() = 0;
    virtual void publishGroundSpeed() = 0;
    virtual void publishRearHitchStatus() = 0;
    virtual void publishRearPtoStatus() = 0;
    virtual void publishTwistMeasurement() = 0;
    virtual builtin_interfaces::msg::Time nowStamp() const = 0;
    virtual void printError(const std::string &) = 0;
    virtual double measured_curvature() const { return 0.0; }

    // Parsed status state
    WheelSpeedData wheel_speed_data_;
    GroundSpeedData ground_speed_data_;
    RearHitchStatus rear_hitch_data_;
    RearPTOStatus rear_pto_data_;

private:
    // PGN parsers (Class2)
    void parseWheelSpeed(const msg::IsobusFrame & imsg);
    void parseGroundSpeed(const msg::IsobusFrame & imsg);
    void parseRearHitchStatus(const msg::IsobusFrame & imsg);
    void parseRearPtoStatus(const msg::IsobusFrame & imsg);
};

}  // namespace ros2_isobus

#endif // ROS2_ISOBUS_TECU_CLASS2CLIENT_HPP
