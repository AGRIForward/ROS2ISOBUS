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

#ifndef ROS2_ISOBUS_TECU_CLASS3CLIENT_HPP
#define ROS2_ISOBUS_TECU_CLASS3CLIENT_HPP

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "TECUClass2Client.hpp"
#include "ros2_isobus/msg/isobus_frame.hpp"
#include "ros2_isobus/msg/tecu_cruise_command.hpp"
#include "ros2_isobus/msg/tecu_guidance_command.hpp"
#include "ros2_isobus/msg/tecu_rear_hitch_command.hpp"
#include "ros2_isobus/msg/tecu_rear_pto_command.hpp"
#include "ros2_isobus/msg/tecu_cruise_status.hpp"
#include "ros2_isobus/msg/tecu_guidance_status.hpp"
#include "ros2_isobus/msg/tecu_steering_wheel.hpp"
#include "ros2_isobus/msg/tecu_steering_valve_status.hpp"
#include "ros2_isobus/msg/isobus_address_book.hpp"
#include "ros2_isobus/msg/isobus_address_entry.hpp"
#include "ros2_isobus/msg/isobus_address_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ros2_isobus/msg/aux_valve_command.hpp"
#include "ros2_isobus/msg/aux_valve_status.hpp"

namespace ros2_isobus
{

/*
 *
 * TECUClass3Client extends Class2 with guidance/cruise (legacy ISO 11783-7:2015 Class 3):
 *  - Publishes steering valve, guidance and cruise status
 *  - Accepts cruise/curvature commands and optional Twist command input
 *  - Sends ISOBUS guidance/cruise/hitch/PTO commands over the bus
 *
 */
class TECUClass3Client : public TECUClass2Client
{
public:
    TECUClass3Client();
    virtual ~TECUClass3Client() = default;

    // Command configuration (guidance/cruise/hydraulics)
    enum class CommandMode { DIRECT, PERIODIC, BOTH };
    void setSpeed(double v);
    void setMaxSpeed(double v);
    void setCurvature(double curvature);
    void setRearHitch(double pos);
    void setPTO(int RPM, bool engagement);
    void setValve(std::uint8_t valve_id, float flow_percent, bool floating, bool failsafe);
    void set_valve_count(int count);
    void set_command_mode(CommandMode mode);
    void set_esp_name(const ByteArray8 & name);
    void set_esp_sa(std::uint8_t sa) { esp_sa_ = sa; }
    void set_my_sa(std::uint8_t sa) { my_sa_ = sa; }

    // Parsed status types and accessors
    using SteeringWheelData = msg::TecuSteeringWheel;
    using SteeringValveStatusData = msg::TecuSteeringValveStatus;
    using GuidanceStatusData = msg::TecuGuidanceStatus;
    using CruiseStatusData = msg::TecuCruiseStatus;

    const SteeringWheelData & steering_wheel() const { return steering_wheel_data_; }
    const SteeringValveStatusData & valve_status() const { return steering_valve_status_data_; }
    const GuidanceStatusData & guidance_status() const { return guidance_status_data_; }
    const CruiseStatusData & cruise_status() const { return cruise_status_data_; }
    const ByteArray8 & guidance_name() const { return esp_name_; }
    std::uint8_t guidance_sa() const { return esp_sa_; }
    std::uint8_t my_sa() const { return my_sa_; }

    // Core processing
    void HandleMsg(const msg::IsobusFrame & imsg);
    void run();

private:
    // Configuration / addressing
    ByteArray8 esp_name_;
    std::uint8_t my_sa_ = 0;
    std::uint8_t esp_sa_ = 0;
    int valve_count_ = 8;
    CommandMode command_mode_ = CommandMode::PERIODIC;

protected:
    static constexpr int kPeriodicTimeoutTicks = 20;  // Stop periodic sends after N ticks without update.
    void setPeriodicTimeoutTicks(int ticks) { periodic_timeout_ticks_ = ticks; }
    int periodicTimeoutTicks() const { return periodic_timeout_ticks_; }

    // State data (accessible to ROS2 wrapper)
    SteeringWheelData steering_wheel_data_;
    SteeringValveStatusData steering_valve_status_data_;
    GuidanceStatusData guidance_status_data_;
    CruiseStatusData cruise_status_data_;
    struct ValveCommand
    {
        float flow_percent = 0.0f;
        bool floating = false;
        bool failsafe = false;
        bool pending = false;
        int timeout_ticks = 0;
    };
    std::vector<ValveCommand> valve_commands_;
    struct ValveStatus
    {
        float extend_flow_percent = 0.0f;
        float retract_flow_percent = 0.0f;
        std::uint8_t state = 0;
        bool failsafe = false;
    };
    std::vector<ValveStatus> valve_status_;

    // Command/state flags
    int start_cruise = 1;
    int start_guidance = 2;
    double setspeed = 0;
    double maxspeed = 10000;
    double setcurvature = 0;
    double setRearHitchPos = 100;           // 0 - 100 [%]
    double setRearPTORPM = 0;               // [1/min]
    char   setRearPTOengagement = 0;        // 0 = off, 1 = on, 2 = error
    bool pendingCurvatureCommand = false;
    bool pendingCruiseCommand = false;
    bool pendingHitchPTOCommand = false;
    int periodic_timeout_ticks_ = kPeriodicTimeoutTicks;
    int curvature_timeout_ = 0;
    int cruise_timeout_ = 0;
    int hitchpto_timeout_ = 0;

    // Virtual hooks
    virtual void publishSteeringWheel() = 0;
    virtual void publishValveStatus() = 0;
    virtual void publishGuidanceStatus() = 0;
    virtual void publishCruiseStatus() = 0;
    virtual void publishAuxValveStatus(std::uint8_t valve_id, const ValveStatus &) = 0;
    virtual void sendFrame(const msg::IsobusFrame &) = 0;
    virtual void printInfo(const std::string &) = 0;
    virtual void printWarn(const std::string &) = 0;
    double measured_curvature() const override { return guidance_status_data_.measured_curvature; }

private:
    // Command builders for periodic (~100 ms) sending
    msg::IsobusFrame curvatureCommand();    
    msg::IsobusFrame cruiseCommand();       
    msg::IsobusFrame hitchPTOCommand();
    msg::IsobusFrame valveCommand(std::uint8_t valve_id, const ValveCommand & cmd);
    msg::IsobusFrame getCurrentModeValve();
    msg::IsobusFrame startStopStatus(char set);  // 1 = set, 0 = stop
    void parseSteeringWheel(const msg::IsobusFrame & imsg);
    void parseAuxValveStatus(const msg::IsobusFrame & imsg);
    void parseValveStatusResponse(const msg::IsobusFrame & imsg);
    void parseValveCommandDevice(const msg::IsobusFrame & imsg);
    void parseGuidanceStatus(const msg::IsobusFrame & imsg);
    void parseCruiseStatus(const msg::IsobusFrame & imsg);

    // Mode helpers
    bool useDirect() const { return command_mode_ == CommandMode::DIRECT || command_mode_ == CommandMode::BOTH; }
    bool usePeriodic() const { return command_mode_ == CommandMode::PERIODIC || command_mode_ == CommandMode::BOTH; }
};

}  // namespace ros2_isobus

#endif // ROS2_ISOBUS_TECU_CLASS3CLIENT_HPP
