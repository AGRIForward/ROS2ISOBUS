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

#include "TECUClass2Client.hpp"
#include <limits>

namespace ros2_isobus
{

// Parse TECU Class2 PGNs and publish status/twist via callbacks.
TECUClass2Client::TECUClass2Client()
{
}

// Decode incoming TECU PGNs for Class2 (speed, hitch, PTO).
void TECUClass2Client::HandleMsg(const msg::IsobusFrame & imsg)
{
    if (imsg.pgn == 0xFE48) // WHEEL SPEED
    {
        parseWheelSpeed(imsg);
    }
    else if (imsg.pgn == 0xFE49) // GROUND SPEED
    {
        parseGroundSpeed(imsg);
    }
    else if (imsg.pgn == 0xFE45) // REAR HITCH STATUS
    {
        parseRearHitchStatus(imsg);
    }
    else if (imsg.pgn == 0xFE43) // REAR PTO STATUS
    {
        parseRearPtoStatus(imsg);
    }
}

// Parse PGN FE48 Wheel Speed per ISO 11783-7:2015, Annex B.3 (Wheel-based speed & distance).
void TECUClass2Client::parseWheelSpeed(const msg::IsobusFrame & imsg)
{
    double temp = ((imsg.data[0] << 0) | (imsg.data[1] << 8)) * 0.001;

    const auto dir_bits = imsg.data[7] & 0x03;  // Bits 2..1.
    wheel_speed_data_.forward = (dir_bits == 0x01);
    if (dir_bits == 0x01)
        wheel_speed_data_.speed_ms = temp;
    else if (dir_bits == 0x00)
        wheel_speed_data_.speed_ms = -temp;
    else
        wheel_speed_data_.speed_ms = temp;  // Default to positive if unknown.

    temp = static_cast<double>((static_cast<std::uint32_t>(imsg.data[2]) << 0) |
                               (static_cast<std::uint32_t>(imsg.data[3]) << 8) |
                               (static_cast<std::uint32_t>(imsg.data[4]) << 16) |
                               (static_cast<std::uint32_t>(imsg.data[5]) << 24)) * 0.001;
    if(temp > 2147483.648)
        temp -= 4294967.296;

    wheel_speed_data_.distance_m = temp;

    const auto key_bits = static_cast<std::uint8_t>((imsg.data[7] >> 2) & 0x03); // Bits 4..3.
    bool new_key_active = key_bits == 0x01;
    if (key_bits == 0x02)
    {
        printError("Key switch state error (PGN FE48)");
        new_key_active = false;
    }
    // If previously active and now off/unknown, warn safe mode.
    if (wheel_speed_data_.key_switch_active && !new_key_active)
    {
        printError("Robot is in safe mode!");
    }
    wheel_speed_data_.key_switch_active = new_key_active;
    wheel_speed_data_.start_stop_state = static_cast<std::uint8_t>((imsg.data[7] >> 4) & 0x03); // Bits 6..5.
    wheel_speed_data_.max_time_of_tractor_power = imsg.data[6]; // Minutes, 1 min/bit.

    publishWheelSpeed();
    publishTwistMeasurement();
}

// Parse PGN FE49 Ground Speed per ISO 11783-7:2015, Annex B.2 (Ground-based speed & distance).
void TECUClass2Client::parseGroundSpeed(const msg::IsobusFrame & imsg)
{
    double temp = ((imsg.data[0] << 0) | (imsg.data[1] << 8)) * 0.001;
    // Direction bits are in the lowest two bits (bits 2..1) of the last byte.
    const auto dir_bits = imsg.data[7] & 0x03;
    ground_speed_data_.forward = (dir_bits == 0x01);
    if (dir_bits == 0x01)
        ground_speed_data_.speed_ms = temp;
    else if (dir_bits == 0x00)
        ground_speed_data_.speed_ms = -temp;
    else
        ground_speed_data_.speed_ms = temp;

    temp = ((imsg.data[2] << 0) | (imsg.data[3] << 8) | (imsg.data[4] << 16) | (imsg.data[5] << 24)) * 0.001;
    if(temp > 2147483.648)
        temp -= 4294967.296;

    ground_speed_data_.distance_m = temp;

    publishGroundSpeed();
}

// Parse PGN FE45 Rear Hitch Status per ISO 11783-7:2015, Annex B.7 (Primary/rear hitch status).
void TECUClass2Client::parseRearHitchStatus(const msg::IsobusFrame & imsg)
{
    rear_hitch_data_.position_percent = imsg.data[0] * 0.4;
    const std::uint8_t b2 = imsg.data[1];
    rear_hitch_data_.in_work = (b2 >> 6) & 0x03;
    rear_hitch_data_.position_limit_status = (b2 >> 3) & 0x07;

    if (imsg.data[2] == 0xFF)
        rear_hitch_data_.nominal_lower_link_force = std::numeric_limits<double>::quiet_NaN();
    else
        rear_hitch_data_.nominal_lower_link_force = static_cast<int>(imsg.data[2]) * 0.8 - 100.0;

    // Draft: 10 N/bit, -320000 offset, bytes 3-4
    std::int32_t draft_raw = static_cast<std::int32_t>(imsg.data[3]) |
                             (static_cast<std::int32_t>(imsg.data[4]) << 8);
    draft_raw = draft_raw & 0xFFFF;
    rear_hitch_data_.draft_n = static_cast<double>(draft_raw) * 10.0 - 320000.0;
    rear_hitch_data_.exit_code = (imsg.data[6] >> 2) & 0x3F;

    publishRearHitchStatus();
}

// Parse PGN FE43 Rear PTO Status per ISO 11783-7:2015, Annex B.9 (Primary/rear PTO output shaft).
void TECUClass2Client::parseRearPtoStatus(const msg::IsobusFrame & imsg)
{
    const std::uint16_t rpm_raw = static_cast<std::uint16_t>(imsg.data[0]) |
                                  (static_cast<std::uint16_t>(imsg.data[1]) << 8);
    if (rpm_raw == 0xFFFF)
        rear_pto_data_.rpm = std::numeric_limits<double>::quiet_NaN();
    else
        rear_pto_data_.rpm = rpm_raw * 0.125;

    const std::uint16_t set_raw = static_cast<std::uint16_t>(imsg.data[2]) |
                                  (static_cast<std::uint16_t>(imsg.data[3]) << 8);
    if (set_raw == 0xFFFF)
        rear_pto_data_.setpoint_rpm = std::numeric_limits<double>::quiet_NaN();
    else
        rear_pto_data_.setpoint_rpm = set_raw * 0.125;

    const std::uint8_t b5 = imsg.data[4];
    rear_pto_data_.engagement = (b5 >> 6) & 0x03;
    rear_pto_data_.mode = (b5 >> 4) & 0x03;
    rear_pto_data_.economy_mode = (b5 >> 2) & 0x03;

    const std::uint8_t b6 = imsg.data[5];
    rear_pto_data_.engagement_request = (b6 >> 6) & 0x03;
    rear_pto_data_.mode_request = (b6 >> 4) & 0x03;
    rear_pto_data_.economy_request = (b6 >> 2) & 0x03;

    const std::uint8_t b7 = imsg.data[6];
    rear_pto_data_.speed_limit_status = (b7 >> 6) & 0x03;

    publishRearPtoStatus();
}

}  // namespace ros2_isobus
