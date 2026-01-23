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

#include <cmath>
#include <chrono>
#include <string>
#include <algorithm>

#include "TECUClass3Client.hpp"

namespace ros2_isobus
{

using namespace std::chrono_literals;

// Class3 (deprecated) TECU client: guidance/cruise command and status handling.
TECUClass3Client::TECUClass3Client()
    : TECUClass2Client()
{
    valve_count_ = 8;
    valve_commands_.assign(static_cast<std::size_t>(std::max(0, valve_count_)), ValveCommand{});
    valve_status_.assign(static_cast<std::size_t>(std::max(0, valve_count_)), ValveStatus{});
    esp_name_ = hex_to_byte_array("A0001900AAA00006");
    esp_sa_ = 0x13;
    command_mode_ = CommandMode::PERIODIC;
    periodic_timeout_ticks_ = kPeriodicTimeoutTicks;
}

void TECUClass3Client::set_valve_count(int count)
{
    valve_count_ = count;
    valve_commands_.assign(static_cast<std::size_t>(std::max(0, valve_count_)), ValveCommand{});
    valve_status_.assign(static_cast<std::size_t>(std::max(0, valve_count_)), ValveStatus{});
}

void TECUClass3Client::set_command_mode(CommandMode mode)
{
    command_mode_ = mode;
}

void TECUClass3Client::set_esp_name(const ByteArray8 & name)
{
    esp_name_ = name;
}

// Decode Class3-specific PGNs (steering wheel, valve status, guidance/cruise).
void TECUClass3Client::HandleMsg(const msg::IsobusFrame & imsg)
{
    TECUClass2Client::HandleMsg(imsg);

    if ( imsg.pgn == 0x03 && imsg.sa == 0x01) // Steering wheel
    {
        parseSteeringWheel(imsg);
    }
    else if ((imsg.pgn & 0xFFF0) == 0xFE10) // AUX valve estimated flow/status
    {
        parseAuxValveStatus(imsg);
    }
    if (imsg.pgn == 0xFF00 && imsg.sa == esp_sa_) // VALVE: StatusResponse, 40ms interval
    {
        parseValveStatusResponse(imsg);
    }
    else if(imsg.pgn == 0xFF01 && imsg.sa == esp_sa_) // VALVE: ??
    {
        parseValveCommandDevice(imsg);
    }
    else if (imsg.pf == 0xAC && (imsg.ps == my_sa_ || imsg.ps == 0xFF) &&  imsg.sa == esp_sa_)	// VALVE: Guidance command status, 80ms interval
    {
        parseGuidanceStatus(imsg);
    }

    if (imsg.pgn == 0xFE0A)
    {
        parseCruiseStatus(imsg);
    }
}

// Parse PGN 0x0003 Steering Wheel (vendor / legacy; not defined in ISO 11783-7).
void TECUClass3Client::parseSteeringWheel(const msg::IsobusFrame & imsg)
{
    steering_wheel_data_.angle = ((imsg.data[0] << 0) | (imsg.data[1] << 8)) * 0.0015341;
    steering_wheel_data_.change = (((imsg.data[3] << 0) | (imsg.data[4] << 8)) - 4095) * 0.0015341;
    publishSteeringWheel();
}

// Parse PGN FE10..FE1F AUX Valve Estimated Flow/Status (ISO 11783-7:2015, Annex B.25.3).
void TECUClass3Client::parseAuxValveStatus(const msg::IsobusFrame & imsg)
{
    std::uint8_t valve_id = static_cast<std::uint8_t>(imsg.pgn & 0x0F);
    if (valve_id >= 1 && valve_id <= valve_count_)
    {
        const float extend_percent = static_cast<int>(imsg.data[0]) - 125.0f;
        const float retract_percent = static_cast<int>(imsg.data[1]) - 125.0f;
        const std::uint8_t state = imsg.data[2] & 0x0F;
        const bool failsafe = (imsg.data[2] >> 6) != 0;

        // Cache latest status
        auto & vs = valve_status_[valve_id - 1];
        vs.extend_flow_percent = extend_percent;
        vs.retract_flow_percent = retract_percent;
        vs.state = state;
        vs.failsafe = failsafe;

        publishAuxValveStatus(valve_id, vs);
    }
}

// Parse PGN FF00 Steering Valve Status Response (vendor-specific, not in ISO 11783-7).
void TECUClass3Client::parseValveStatusResponse(const msg::IsobusFrame & imsg)
{
    const double temp = ((imsg.data[0] << 0) | (imsg.data[1] << 8)) / 1023.0;	// Set 1 value.
    steering_valve_status_data_.measured_steering = temp;
    steering_valve_status_data_.mode = imsg.data[2];

    publishValveStatus();
}

// Parse PGN FF01 Valve Command Device (vendor-specific, not in ISO 11783-7).
void TECUClass3Client::parseValveCommandDevice(const msg::IsobusFrame & imsg)
{
    steering_valve_status_data_.command_device = imsg.data[1];
    guidance_status_data_.command_device = imsg.data[1];

    if(steering_valve_status_data_.command_device != 0x05 && start_guidance == 0)
        start_guidance = 2;
}

// Parse PF 0xAC Guidance Machine Status (ISO 11783-7:2015, Annex B.26.2; PGN 44032 / 0xAC00).
void TECUClass3Client::parseGuidanceStatus(const msg::IsobusFrame & imsg)
{
    const double temp = ((imsg.data[0] << 0) | (imsg.data[1] << 8)) * 0.25 - 8032;

    guidance_status_data_.measured_curvature = -temp;
    publishGuidanceStatus();
}

// Parse PGN FE0A Cruise Status (ISO 11783-7:2015, Annex B.24.2; PGN 65034 / Tractor control response).
void TECUClass3Client::parseCruiseStatus(const msg::IsobusFrame & imsg)
{
    if( (imsg.data[0] & 0xF8) == 0x08 ||	// 00001??? --> Ground speed cruise control
        (imsg.data[0] & 0xF8) == 0x10 ||	// 00010??? --> Wheel speed cruise control
        (imsg.data[0] & 0xF8) == 0x18)		// 00011??? --> Navigation speed cruise control
    {
        if((imsg.data[0] & 0x07) == 0x01 )	// Operator
            start_cruise = 1;
        else								// Remote mode / Not limited
            start_cruise = 0;

        cruise_status_data_.command_status = static_cast<int32_t>(imsg.data[0]);
        cruise_status_data_.command_speed = ((imsg.data[2] << 0) | (imsg.data[3] << 8)) * 0.001;
        publishCruiseStatus();
    }
}

// Set desired speed for cruise command.
void TECUClass3Client::setSpeed(double v)
{
    setspeed = v;
    pendingCruiseCommand = usePeriodic();
    cruise_timeout_ = kPeriodicTimeoutTicks;
    if (useDirect() && my_sa_ != 0) {
        sendFrame(cruiseCommand());
    }
}

// Set maximum allowed speed for cruise command.
void TECUClass3Client::setMaxSpeed(double v)
{
    maxspeed = v;
}

// Set desired curvature for guidance command.
void TECUClass3Client::setCurvature(double curvature)
{
    setcurvature = curvature;

    if(setcurvature > 8032)
        setcurvature = 8032;
    else if(setcurvature < -8031)
        setcurvature = -8031;

    pendingCurvatureCommand = usePeriodic();
    curvature_timeout_ = periodic_timeout_ticks_;
    if (useDirect() && my_sa_ != 0) {
        sendFrame(curvatureCommand());
    }
}

// Set rear hitch position request.
void TECUClass3Client::setRearHitch(double pos)
{
    setRearHitchPos = pos;
    pendingHitchPTOCommand = usePeriodic();
    hitchpto_timeout_ = periodic_timeout_ticks_;
    if (useDirect() && my_sa_ != 0) {
        sendFrame(hitchPTOCommand());
    }
}

// Set rear PTO command.
void TECUClass3Client::setPTO(int RPM, bool engagement)
{
    setRearPTORPM = RPM;
    setRearPTOengagement = engagement;
    pendingHitchPTOCommand = usePeriodic();
    hitchpto_timeout_ = periodic_timeout_ticks_;
    if (useDirect() && my_sa_ != 0) {
        sendFrame(hitchPTOCommand());
    }
}

// Periodic tick: send pending curvature/cruise/hitch/PTO commands.
void TECUClass3Client::run()
{
    if(my_sa_ == 0)
        return;
    
    const bool periodic = usePeriodic();

    if(pendingCurvatureCommand)
    {
        sendFrame(curvatureCommand());
        if (!periodic || --curvature_timeout_ <= 0)
            pendingCurvatureCommand = false;
    }

    if(pendingCruiseCommand)
    {
        sendFrame(cruiseCommand());
        if (!periodic || --cruise_timeout_ <= 0)
            pendingCruiseCommand = false;
    }

    if(pendingHitchPTOCommand)
    {
        sendFrame(hitchPTOCommand());
        if (!periodic || --hitchpto_timeout_ <= 0)
            pendingHitchPTOCommand = false;
    }

    for (std::uint8_t idx = 1; idx <= valve_commands_.size(); ++idx)
    {
        auto & cmd = valve_commands_[idx - 1];
        if (cmd.pending)
        {
            sendFrame(valveCommand(idx, cmd));
            if (!periodic || --cmd.timeout_ticks <= 0)
                cmd.pending = false;
        }
    }
}

// Build ISOBUS message to query current valve mode.
msg::IsobusFrame TECUClass3Client::getCurrentModeValve()
{
    msg::IsobusFrame imsg;

    imsg.pgn = 0xEF00;
    imsg.page = 0;
    imsg.priority = 3;
    imsg.sa = my_sa_;

    imsg.data[0] = 0x0F;
    imsg.data[1] = 0xA9;
    imsg.data[2] = 0;
    imsg.data[3] = 0;
    imsg.data[4] = 0;
    imsg.data[5] = 0;
    imsg.data[6] = 0;
    imsg.data[7] = 0;

    return imsg;
}

// Build start/stop status command frame.
msg::IsobusFrame TECUClass3Client::startStopStatus(char set)
{
    msg::IsobusFrame imsg;

    imsg.pgn = 0xEF00;
    imsg.page = 0;
    imsg.priority = 6;
    imsg.sa = my_sa_;

    imsg.data[0] = 0xAB;
    imsg.data[1] = 0x0F;
    imsg.data[2] = set;							// 1 = set1, 0 = stop
    imsg.data[3] = 0;
    imsg.data[4] = 0;
    imsg.data[5] = 0;
    imsg.data[6] = 0;
    imsg.data[7] = 0;

    return imsg;
}

// Build guidance curvature command frame.
msg::IsobusFrame TECUClass3Client::curvatureCommand()
{
    msg::IsobusFrame imsg;

    imsg.pgn = 0xAD00 | esp_sa_;
    imsg.page = 0;
    imsg.priority = 3;
    imsg.sa = my_sa_;

    if(start_guidance == 2)
    {
        unsigned long temp = static_cast<unsigned long>((-guidance_status_data_.measured_curvature + 8032) * 4);

        imsg.data[0] = static_cast<std::uint8_t>(temp & 0xFF);
        imsg.data[1] = static_cast<std::uint8_t>((temp >> 8) & 0xFF);
        imsg.data[2] = 0xFC;                    // ??????00 = not intended for steering.

        start_guidance = 1;
    }
    else
    {
        unsigned long temp = static_cast<unsigned long>((-setcurvature + 8032) * 4);

        imsg.data[0] = static_cast<std::uint8_t>(temp & 0xFF);
        imsg.data[1] = static_cast<std::uint8_t>((temp >> 8) & 0xFF);
        imsg.data[2] = 0xFD;					// ??????01 = intended for steering.

        start_guidance = 0;
    }

    imsg.data[3] = 0xFF;
    imsg.data[4] = 0xFF;
    imsg.data[5] = 0xFF;
    imsg.data[6] = 0xFF;
    imsg.data[7] = 0xFF;

    return imsg;
}

// Build cruise control command frame.
msg::IsobusFrame TECUClass3Client::cruiseCommand()
{
    msg::IsobusFrame imsg;

    imsg.pgn = 0xFE0B;
    imsg.page = 0;
    imsg.priority = 3;
    imsg.sa = my_sa_;

    imsg.data[0] = 0x0F; // Command 1: Cruise control
    imsg.data[1] = 0xFF; // Command 2: Not requested

    long temp;
    switch(start_cruise)
    {
    case 2:
        // Request current TECU command value.
        imsg.data[2] = 0xFF;
        imsg.data[3] = 0xFF;
        break;
    case 1:
        // Request the adjustment of set point for cruise control.
        temp = static_cast<unsigned long>(cruise_status_data_.command_speed * 1000.0);

        imsg.data[2] = static_cast<std::uint8_t>(temp & 0xFF);
        imsg.data[3] = static_cast<std::uint8_t>((temp >> 8) & 0xFF);
        break;
    default:
        // Request change in speed.
        double speedcmd = setspeed;
        if(std::abs(speedcmd) > maxspeed)
            speedcmd = speedcmd > 0 ? maxspeed : -maxspeed;

        if(speedcmd < 0)
        {
            temp = 65535 - static_cast<unsigned long>(-speedcmd * 1000.0);
        }
        else
        {
            temp = static_cast<unsigned long>(speedcmd * 1000.0);
        }

        imsg.data[2] = static_cast<std::uint8_t>(temp & 0xFF);
        imsg.data[3] = static_cast<std::uint8_t>((temp >> 8) & 0xFF);
        break;
    }

    imsg.data[4] = 0xFF;
    imsg.data[5] = 0xFF;
    imsg.data[6] = 0xFF;
    imsg.data[7] = 0xFF;

    return imsg;
}

// Build hitch/PTO command frame.
msg::IsobusFrame TECUClass3Client::hitchPTOCommand()
{
    msg::IsobusFrame imsg;

    imsg.pgn = 0xFE42;
    imsg.page = 0;
    imsg.priority = 3;
    imsg.sa = my_sa_;

    imsg.data[0] = 0xFF;                                    // FrontHitchPos
    imsg.data[1] = static_cast<std::uint8_t>(setRearHitchPos * 2.5);  // RearHitchPos

    if(setRearHitchPos > 100 || setRearHitchPos < 0)        // Ignore invalid values.
        imsg.data[1] = 0xFF;

    imsg.data[2] = 0xFF;                                    // Front PTO
    imsg.data[3] = 0xFF;                                    // Front PTO
 
    imsg.data[6] = 0xCF | ((setRearPTOengagement<<4) & 0x30);  // RearPTOengagement

    long temp = static_cast<long>(setRearPTORPM * 8);
    if(setRearPTORPM >= 8191.875 || setRearPTORPM < 0)      // Ignore invalid values.
    {
        temp = 0xFFFF;
        imsg.data[6] = 0xFF;
    }

    imsg.data[4] = static_cast<std::uint8_t>(temp & 0xFF);            // RearPTO
    imsg.data[5] = static_cast<std::uint8_t>((temp >> 8) & 0xFF);         // RearPTO

    imsg.data[7] = 0xFF;

    return imsg;
}

void TECUClass3Client::setValve(std::uint8_t valve_id, float flow_percent, bool floating, bool failsafe)
{
    if (valve_id < 1 || valve_id > valve_count_)
    {
        return;
    }
    auto & cmd = valve_commands_[valve_id - 1];
    cmd.flow_percent = flow_percent;
    cmd.floating = floating;
    cmd.failsafe = failsafe;
    cmd.pending = usePeriodic();
            cmd.timeout_ticks = periodic_timeout_ticks_;

    if (useDirect() && my_sa_ != 0)
    {
        sendFrame(valveCommand(valve_id, cmd));
    }
}


msg::IsobusFrame TECUClass3Client::valveCommand(std::uint8_t valve_id, const ValveCommand & cmd)
{
    msg::IsobusFrame imsg;
    imsg.pgn = 0xFE30 | (valve_id & 0x0F);
    imsg.page = 0;
    imsg.priority = 3;
    imsg.sa = my_sa_;

    // Flow is 0..100% magnitude; sign determines direction unless floating overrides.
    const float mag = std::fabs(cmd.flow_percent);
    const int raw_flow = std::clamp(static_cast<int>(std::lround(mag * 2.5f)), 0, 250);
    imsg.data[0] = static_cast<std::uint8_t>(raw_flow);  // 0.4 %/bit magnitude.
    const std::uint8_t state =
        cmd.floating ? 0x03 :    // Floating mode.
        (raw_flow == 0 ? 0x00 : (cmd.flow_percent >= 0.0f ? 0x01 : 0x02));  // Block/extend/retract.
    imsg.data[1] = 0;
    imsg.data[2] = static_cast<std::uint8_t>(((cmd.failsafe ? 1 : 0) << 6) | (state & 0x0F));
    imsg.data[3] = 0;
    imsg.data[4] = 0;
    imsg.data[5] = 0;
    imsg.data[6] = 0;
    imsg.data[7] = 0;

    return imsg;
}

}  // namespace ros2_isobus
