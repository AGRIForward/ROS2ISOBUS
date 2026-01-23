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

#include "NMEA2000Client.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"


namespace ros2_isobus
{
NMEA2000Client::NMEA2000Client(const std::string & frame_id)
: frame_id_(frame_id)
{}

// Map NMEA2000 GPSType to NavSatStatus::service bitmask.
uint16_t NMEA2000Client::gnss_service(char type) const
{
    using sensor_msgs::msg::NavSatStatus;
    switch (type)
    {
        case 0: // GPS
            return NavSatStatus::SERVICE_GPS;
        case 1: // GLONASS
            return NavSatStatus::SERVICE_GLONASS;
        case 2: // GPS + GLONASS
            return NavSatStatus::SERVICE_GPS | NavSatStatus::SERVICE_GLONASS;
        case 3: // GPS + SBAS/WAAS
            return NavSatStatus::SERVICE_GPS;
        case 4: // GPS + SBAS + GLONASS
            return NavSatStatus::SERVICE_GPS | NavSatStatus::SERVICE_GLONASS;
        case 8: // Galileo
            return NavSatStatus::SERVICE_GALILEO;
        default:
            return NavSatStatus::SERVICE_GPS;
    }
}

// Dispatch incoming PGNs to specific N2K parsers and publish results.
void NMEA2000Client::HandleMsg(const ros2_isobus::msg::IsobusFrame & imsg)
{
    switch(imsg.pgn)
    {
        case 0xF119:    // Attitude
            Parse_Attitude(imsg.data);
            publishAttitude();
            break;
        case 0xF801:    // Position, Rapid update
            Parse_PositionRapidUpdate(imsg.data);
            publishRapidPosition();
            break;
        case 0xF802:    // COG & SOG, Rapid Update
            Parse_COG_SOG_RapidUpdate(imsg.data);
            publishCogSog();
            break;
        case 0xF803:    // Position delta
            Parse_PositionDelta(imsg.data);
            publishNavSatFix();
            break;
        case 0xF805:    // GNSS Position Data
            if( Parse_FastPacketProtocol(imsg.data, &GNSS_Position_Data) )
            {
                Parse_GNSS_Position(&GNSS_Position_Data);
                publishNavSatFix();
            }
            break;
        case 0xFA06:    // GNSS Pseudo Noise Statistics
            if( Parse_FastPacketProtocol(imsg.data, &GNSS_PseudoNoiseStatistics) )
            {
                Parse_GNSS_PseudoNoiseStats(&GNSS_PseudoNoiseStatistics);
                publishPseudoNoise();
            }
            break;
    }

    if(GPSFix >= 4)
    {
        if(!GPS_OK)
        {
            GPS_OK = true;
            printInfo("NMEA2000: GPS signal OK");
        }
    }
    else
    {
        if(GPS_OK)
        {
            GPS_OK = false;
            printWarn("NMEA2000: GPS signal is not accurate");
        }
    }
}

// Accumulate fast-packet PGNs (multi-frame) per NMEA 2000 Fast Packet transport
// (IEC 61162-3 / ISO 11783-3, Annex C.1 fast-packet).
int NMEA2000Client::Parse_FastPacketProtocol(const ByteArray8& msgData, struct FastPacketData *data)
{
    unsigned int SeqCount = msgData[0] >> 5;
    unsigned int FrameCount = msgData[0] & 0x1F;

    if (FrameCount == 0)
    {
        data->seq = SeqCount;
        data->frame = 1;
        data->lenght = msgData[1];

        if(data->lenght > sizeof(data->data))
        {
            data->lenght = 0;
        }
        else
        {
            for(int i=0; i<6; i++)
                data->data[i] = msgData[2+i];
        }
    }
    else if (data->frame > 0)
    {
        if( data->frame != FrameCount || data->seq != SeqCount)
        {
            data->frame = 0;
            data->lenght = 0;
        }
        else
        {
            for(int i=0; i<7; i++)
            {
                unsigned int pos = 6 + (data->frame - 1) * 7 + i;
                if (pos < data->lenght)
                    data->data[pos] = msgData[1+i];
            }

            if( (13 + (data->frame - 1) * 7) >= data->lenght)
            {
                data->frame = 0;
                return 1;
            }

            data->frame++;
        }
    }

    return 0;
}

// Parse PGN 129029 GNSS Position Data (NMEA 2000 Appendix B1).
void NMEA2000Client::Parse_GNSS_Position(struct FastPacketData *data)
{
    if(data->lenght < 43)
    {
        return;
    }

    // Sequence ID is stored in data[0].
    PositionDate = static_cast<int>(data->data[1] | (data->data[2] << 8));
    PositionTime = static_cast<double>(data->data[3] | (data->data[4] << 8) | (data->data[5] << 16) | (data->data[6] << 24)) * 1e-4;

    unsigned long long lat = 0;
    for (int i = 0; i < 8; i++)
        lat += (static_cast<unsigned long long>(data->data[7 + i])) << (8 * i);

    latitude = static_cast<double>(lat) * 1e-16;

    unsigned long long lon = 0;
    for (int i = 0; i < 8; i++)
        lon += (static_cast<unsigned long long>(data->data[15 + i])) << (8 * i);

    longitude = static_cast<double>(lon) * 1e-16;

    unsigned long long alt = 0;
    for (int i = 0; i < 8; i++)
        alt += (static_cast<unsigned long long>(data->data[23 + i])) << (8 * i);

    altitude = static_cast<double>(alt) * 1e-6;

    GPSType = data->data[31] & 0xF;
    GPSFix = data->data[31] >> 4;
    integrity = data->data[32] & 0x03;
    satellites = data->data[33];

    HDOP = static_cast<double>(data->data[34] | (data->data[35] << 8)) * 1e-2;
    PDOP = static_cast<double>(data->data[36] | (data->data[37] << 8)) * 1e-2;

    GeoidalSeparation = static_cast<double>(data->data[38] | (data->data[39] << 8) | (data->data[40] << 16) | (data->data[41] << 24)) * 1e-2;
}

// Parse PGN 129027 Position Delta, High Precision Rapid Update (NMEA 2000 Appendix B1).
void NMEA2000Client::Parse_PositionDelta(const ByteArray8 &data) 
{ 
    const uint8_t sequenceID = data[0];
    const uint8_t timeDelta = data[1];
    (void)sequenceID;
    (void)timeDelta;

    int32_t latitudeDelta = data[2] | (data[3] << 8) | (data[4] << 16);
    if (latitudeDelta & 0x800000) {
        latitudeDelta |= 0xFF000000;
    }
    int32_t longitudeDelta = data[5] | (data[6] << 8) | (data[7] << 16);
    if (longitudeDelta & 0x800000) {
        longitudeDelta |= 0xFF000000;
    }

    double latitude_delta_deg = latitudeDelta * 1e-5 / 3600.0;
    double longitude_delta_deg = longitudeDelta * 1e-5 / 3600.0;
    // Apply deltas to the main coordinates.
    latitude += latitude_delta_deg;
    longitude += longitude_delta_deg;
    // PositionTime could be updated with timeDelta * 5e-3.
}

// Parse PGN 129539 GNSS Pseudo Noise Statistics (NMEA 2000 Appendix B1).
void NMEA2000Client::Parse_GNSS_PseudoNoiseStats(struct FastPacketData *data)
{
    if(data->lenght < 15)
    {
        return;
    }

    RMS_uncertainty = static_cast<double>(data->data[1] | (data->data[2] << 8)) * 1e-2;
    STD_of_Major = static_cast<double>(data->data[3] | (data->data[4] << 8)) * 1e-2;
    STD_of_Minor = static_cast<double>(data->data[5] | (data->data[6] << 8)) * 1e-2;
    Orientation_of_Major = static_cast<double>(data->data[7] | (data->data[8] << 8)) * 1e-4;
    STD_lat = static_cast<double>(data->data[9] | (data->data[10] << 8)) * 1e-2;
    STD_lon = static_cast<double>(data->data[11] | (data->data[12] << 8)) * 1e-2;
    STD_alt = static_cast<double>(data->data[13] | (data->data[14] << 8)) * 1e-2;
}

// Parse PGN 129025 Position, Rapid Update (NMEA 2000 Appendix B1).
void NMEA2000Client::Parse_PositionRapidUpdate(const ByteArray8& data)
{
    unsigned long long lat = 0;
    for (int i = 0; i < 4; i++)
        lat += (static_cast<unsigned long long>(data[0 + i])) << (8 * i);

    unsigned long long lon = 0;
    for (int i = 0; i < 4; i++)
        lon += (static_cast<unsigned long long>(data[4 + i])) << (8 * i);

    rapid_latitude = (static_cast<double>(lat)) * 1e-7;
    rapid_longitude = (static_cast<double>(lon)) * 1e-7;
}

// Parse PGN 129026 COG & SOG, Rapid Update (NMEA 2000 Appendix B1).
void NMEA2000Client::Parse_COG_SOG_RapidUpdate(const ByteArray8& data)
{
    compass_rad = static_cast<double>(static_cast<std::uint16_t>(data[2] | (data[3] << 8))) * 1e-4;
    speed_ms = static_cast<double>(static_cast<std::uint16_t>(data[4] | (data[5] << 8))) * 1e-2;
}

// Parse PGN 127257 Attitude (NMEA 2000 Appendix B1).
void NMEA2000Client::Parse_Attitude(const ByteArray8& data)
{
    Yaw = static_cast<int16_t>((data[1] << 0) | (data[2] << 8)) * 0.0001;
    Pitch = static_cast<int16_t>((data[3] << 0) | (data[4] << 8)) * 0.0001;
    Roll = static_cast<int16_t>((data[5] << 0) | (data[6] << 8)) * 0.0001;
}

}  // namespace ros2_isobus
