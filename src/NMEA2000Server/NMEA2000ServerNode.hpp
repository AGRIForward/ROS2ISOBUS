/*
 *  This file is part of ROS2ISOBUS
 *
 *  Copyright 2026 Juha Backman / Natural Resources Institute Finland
 *
 *  SPDX-License-Identifier: LGPL-3.0-only
 */

#pragma once

#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_isobus/msg/isobus_address_status.hpp"
#include "ros2_isobus/msg/isobus_frame.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

namespace ros2_isobus
{

/**
 * Converts ROS measurements into the same NMEA 2000 PGNs decoded by
 * NMEA2000Client. Each input topic is independent and event-driven.
 */
class NMEA2000Server : public rclcpp::Node
{
public:
  explicit NMEA2000Server(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onAttitude(const geometry_msgs::msg::Vector3Stamped & msg);
  void onImu(const sensor_msgs::msg::Imu & msg);
  void onVelocity(const geometry_msgs::msg::TwistWithCovarianceStamped & msg);
  void onRapidPosition(const sensor_msgs::msg::NavSatFix & msg);
  void onCogSog(const geometry_msgs::msg::TwistStamped & msg);
  void onPositionDelta(const sensor_msgs::msg::NavSatFix & msg);
  void onGnssPosition(const sensor_msgs::msg::NavSatFix & msg);
  void onPseudoNoise(const diagnostic_msgs::msg::DiagnosticArray & msg);
  void onAddressStatus(const msg::IsobusAddressStatus & msg);

  bool ready(const char * measurement);
  void publishPayload(std::uint32_t pgn, const std::vector<std::uint8_t> & payload);
  std::uint8_t nextSid();

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr rapid_position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cog_sog_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_delta_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_position_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    pseudo_noise_sub_;
  rclcpp::Subscription<msg::IsobusAddressStatus>::SharedPtr address_sub_;
  rclcpp::Publisher<msg::IsobusFrame>::SharedPtr bus_pub_;

  std::uint8_t source_address_{0xFE};
  std::uint8_t sequence_id_{0};
  std::uint8_t fast_packet_sequence_{0};
  int priority_{2};
  bool address_claimed_{false};
  bool address_warning_reported_{false};
};

}  // namespace ros2_isobus
