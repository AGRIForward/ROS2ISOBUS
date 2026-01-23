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

#include <string>

#include "ros2_isobus/ISOBUSframe.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace ros2_isobus
{

/*
 *
 * NMEA2000Client parses NMEA 2000 PGNs into ROS standard messages:
 *  - GNSS Position (PGN 129029 / 0xF805) -> NavSatFix
 *  - GNSS Pseudo Noise Stats (PGN 129539 / 0xFA06) -> DiagnosticArray
 *  - Rapid updates (Position 0xF801, COG/SOG 0xF802, Attitude 0xF119)
 * Implements a SocketCAN-facing bridge for N2K navigation data.
 * Transport/output hooks are virtual; ROS2 binding lives in NMEA2000ClientNode.*
 *
 */
struct FastPacketData
{
    unsigned char data[256];
    unsigned int lenght = 0;
    unsigned int frame = 0;
    unsigned int seq = 0;
};

class NMEA2000Client
{
public:
    explicit NMEA2000Client(const std::string & frame_id = "map");
    virtual ~NMEA2000Client() = default;

    void HandleMsg(const msg::IsobusFrame &imsg);

protected:
    int Parse_FastPacketProtocol(const ByteArray8 &msgData, FastPacketData *data);
    void Parse_GNSS_Position(FastPacketData *data);
    void Parse_PositionRapidUpdate(const ByteArray8 &data);
    void Parse_PositionDelta(const ByteArray8 &data);
    void Parse_GNSS_PseudoNoiseStats(FastPacketData *data);
    void Parse_COG_SOG_RapidUpdate(const ByteArray8 &data);
    void Parse_Attitude(const ByteArray8 &data);
    
    virtual void publishNavSatFix() = 0;
    virtual void publishCogSog() = 0;
    virtual void publishAttitude() = 0;
    virtual void publishPseudoNoise() = 0;
    virtual void publishRapidPosition() = 0;
    uint16_t gnss_service(char type) const;
    virtual builtin_interfaces::msg::Time nowStamp() const = 0;
    virtual void printInfo(const std::string & msg) = 0;
    virtual void printWarn(const std::string & msg) = 0;
    FastPacketData GNSS_Position_Data;
    FastPacketData GNSS_PseudoNoiseStatistics;

    // GNSS Position Data
    int PositionDate = 0;
    double PositionTime = 0;
    double latitude = 0;
    double longitude = 0;
    double altitude = 0;
    char GPSType = 0;
    char GPSFix = 0;
    char integrity = 0;
    int satellites = 0;
    double HDOP = 0;
    double PDOP = 0;
    double GeoidalSeparation = 0;

    // GNSS Pseudo Noise Statistics
    double RMS_uncertainty = 0;
    double STD_of_Major = 0;
    double STD_of_Minor = 0;
    double Orientation_of_Major = 0;
    double STD_lat = 0;
    double STD_lon = 0;
    double STD_alt = 0;

    // COG & SOG, Rapid Update
    double compass_rad = 0;
    double speed_ms = 0;

    // Position rapid update
    double rapid_latitude = 0;
    double rapid_longitude = 0;

    // Attitude (IMU)
    double Yaw = 0;
    double Pitch = 0;
    double Roll = 0;

    bool GPS_OK = false;
protected:
    std::string frame_id_{"map"};
};

}  // namespace ros2_isobus
