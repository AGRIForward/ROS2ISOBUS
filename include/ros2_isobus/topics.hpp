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

// Tractor-side Class 1/2/3 server interfaces. Inputs describe the physical
// tractor; Class 3 commands are decoded requests received from an implement.
constexpr char kTECUServerTwistMeasurementTopic[] =
  "ISOBUS/tecu/server/inputs/twist_measured";
constexpr char kTECUServerRearHitchStatusTopic[] =
  "ISOBUS/tecu/server/inputs/rear_hitch_status";
constexpr char kTECUServerRearPtoStatusTopic[] =
  "ISOBUS/tecu/server/inputs/rear_pto_status";
constexpr char kTECUServerAuxValveStatusTopic[] =
  "ISOBUS/tecu/server/inputs/aux_valve_status";
constexpr char kTECUServerEngineSpeedTopic[] =
  "ISOBUS/tecu/server/inputs/engine_speed_rpm";
constexpr char kTECUServerMaximumPowerTimeTopic[] =
  "ISOBUS/tecu/server/inputs/maximum_power_time_min";
constexpr char kTECUServerKeySwitchTopic[] =
  "ISOBUS/tecu/server/inputs/key_switch_active";
constexpr char kTECUServerGuidanceReadyTopic[] =
  "ISOBUS/tecu/server/inputs/guidance_ready";
constexpr char kTECUServerMechanicalLockoutTopic[] =
  "ISOBUS/tecu/server/inputs/mechanical_lockout";
constexpr char kTECUServerTwistCommandTopic[] =
  "ISOBUS/tecu/server/commands/twist";
constexpr char kTECUServerCruiseCommandTopic[] =
  "ISOBUS/tecu/server/commands/cruise";
constexpr char kTECUServerCurvatureCommandTopic[] =
  "ISOBUS/tecu/server/commands/curvature";
constexpr char kTECUServerRearHitchCommandTopic[] =
  "ISOBUS/tecu/server/commands/rear_hitch";
constexpr char kTECUServerRearPtoCommandTopic[] =
  "ISOBUS/tecu/server/commands/rear_pto";
constexpr char kTECUServerAuxValveCommandTopic[] =
  "ISOBUS/tecu/server/commands/aux_valve";

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
constexpr char kNmea2000ImuTopic[] = "ISOBUS/nmea2000/imu";
constexpr char kNmea2000VelocityTopic[] = "ISOBUS/nmea2000/velocity";
// Application measurement inputs encoded and transmitted as NMEA 2000 PGNs.
// TX topics are deliberately separate from decoded RX topics to prevent a
// CAN -> ROS -> CAN feedback loop.
constexpr char kNmea2000AttitudeTxTopic[] = "ISOBUS/nmea2000/tx/attitude";
constexpr char kNmea2000ImuTxTopic[] = "ISOBUS/nmea2000/tx/imu";
constexpr char kNmea2000VelocityTxTopic[] = "ISOBUS/nmea2000/tx/velocity";
constexpr char kNmea2000RapidPositionTxTopic[] =
  "ISOBUS/nmea2000/tx/gnss_position_data_rapid";
constexpr char kNmea2000CogSogTxTopic[] = "ISOBUS/nmea2000/tx/cog_sog";
constexpr char kNmea2000PositionDeltaTxTopic[] =
  "ISOBUS/nmea2000/tx/position_delta";
constexpr char kNmea2000GnssPositionTxTopic[] =
  "ISOBUS/nmea2000/tx/gnss_position_data";
constexpr char kNmea2000PseudoNoiseTxTopic[] =
  "ISOBUS/nmea2000/tx/gnss_pseudo_noise_statistics";
constexpr char kNmea2000RapidPositionTopic[] = "ISOBUS/nmea2000/gnss_position_data_rapid";

constexpr char kBusRxTopic[] = "ISOBUS/bus_rx_frames";
constexpr char kBusTxTopic[] = "ISOBUS/bus_tx_frames";
constexpr char kBusRxTpTopic[] = "ISOBUS/bus_rx_tp_frames";
constexpr char kBusTxTpTopic[] = "ISOBUS/bus_tx_tp_frames";
constexpr char kBusTxTpStatusTopic[] = "ISOBUS/bus_tp_tx_status";
}  // namespace ros2_isobus
