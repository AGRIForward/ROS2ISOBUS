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

namespace ros2_isobus
{
constexpr char kCommandTopic[] = "ISOBUS/commands";
constexpr char kStatusTopic[] = "ISOBUS/status";
constexpr char kDiagnosticsTopic[] = "ISOBUS/diagnostics";

constexpr char kAddressClaimRequests[] = "ISOBUS/address_manager/claims";
constexpr char kAddressManagerStatus[] = "ISOBUS/address_manager/status";
constexpr char kAddressManagerAddressBook[] = "ISOBUS/address_manager/address_book";

constexpr char kTECUCommandTopic[] = "ISOBUS/tecu/commands";
constexpr char kTECUStatusTopic[] = "ISOBUS/tecu/status";
constexpr char kTECUWheelSpeedTopic[] = "ISOBUS/tecu/wheel_speed";
constexpr char kTECUGroundSpeedTopic[] = "ISOBUS/tecu/ground_speed";
constexpr char kTECURearHitchTopic[] = "ISOBUS/tecu/rear_hitch_status";
constexpr char kTECURearPtoTopic[] = "ISOBUS/tecu/rear_pto_status";
constexpr char kTECUSteeringWheelTopic[] = "ISOBUS/tecu/steering_wheel";
constexpr char kTECUSteeringValveStatusTopic[] = "ISOBUS/tecu/steering_valve_status";
constexpr char kTECUGuidanceStatusTopic[] = "ISOBUS/tecu/guidance_status";
constexpr char kTECUCruiseStatusTopic[] = "ISOBUS/tecu/cruise_status";
constexpr char kTECUTwistMeasurementTopic[] = "ISOBUS/tecu/twist_measured";
constexpr char kTECUTwistCommandTopic[] = "ISOBUS/tecu/twist_command";
constexpr char kAuxValveCommandTopic[] = "ISOBUS/tecu/aux_valve_command";
constexpr char kAuxValveStatusTopic[] = "ISOBUS/tecu/aux_valve_status";

constexpr char kTECUCruiseCommandTopic[] = "ISOBUS/tecu/commands/cruise";
constexpr char kTECUCurvatureCommandTopic[] = "ISOBUS/tecu/commands/curvature";
constexpr char kTECURearHitchCommandTopic[] = "ISOBUS/tecu/commands/rear_hitch";
constexpr char kTECURearPtoCommandTopic[] = "ISOBUS/tecu/commands/rear_pto";

constexpr char kTIMCruiseStatusTopic[] = "ISOBUS/tim/cruise_status";
constexpr char kTIMCurvatureStatusTopic[] = "ISOBUS/tim/curvature_status";
constexpr char kTIMRearHitchStatusTopic[] = "ISOBUS/tim/rear_hitch_status";
constexpr char kTIMRearPtoStatusTopic[] = "ISOBUS/tim/rear_pto_status";
constexpr char kTIMAuxValveStatusTopic[] = "ISOBUS/tim/aux_valve_status";

constexpr char kTIMCruiseCommandTopic[] = "ISOBUS/tim/commands/cruise";
constexpr char kTIMCurvatureCommandTopic[] = "ISOBUS/tim/commands/curvature";
constexpr char kTIMRearHitchCommandTopic[] = "ISOBUS/tim/commands/rear_hitch";
constexpr char kTIMRearPtoCommandTopic[] = "ISOBUS/tim/commands/rear_pto";
constexpr char kTIMAuxValveCommandTopic[] = "ISOBUS/tim/commands/aux_valve";

constexpr char kNmea2000FrameTopic[] = "ISOBUS/nmea2000/frames";
constexpr char kNmea2000DiagnosticsTopic[] = "ISOBUS/nmea2000/diagnostics";
constexpr char kNmea2000GnssPositionTopic[] = "ISOBUS/nmea2000/gnss_position_data";
constexpr char kNmea2000PseudoNoiseTopic[] = "ISOBUS/nmea2000/gnss_pseudo_noise_statistics";
constexpr char kNmea2000CogSogTopic[] = "ISOBUS/nmea2000/cog_sog";
constexpr char kNmea2000AttitudeTopic[] = "ISOBUS/nmea2000/attitude";
constexpr char kNmea2000RapidPositionTopic[] = "ISOBUS/nmea2000/rapid_position";

constexpr char kBusRxTopic[] = "ISOBUS/bus_rx_frames";
constexpr char kBusTxTopic[] = "ISOBUS/bus_tx_frames";
constexpr char kBusRxTpTopic[] = "ISOBUS/bus_rx_tp_frames";
constexpr char kBusTxTpTopic[] = "ISOBUS/bus_tx_tp_frames";
}  // namespace ros2_isobus
