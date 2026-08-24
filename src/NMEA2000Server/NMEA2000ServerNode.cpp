/*
 *  This file is part of ROS2ISOBUS
 *
 *  Copyright 2026 Juha Backman / Natural Resources Institute Finland
 *
 *  SPDX-License-Identifier: LGPL-3.0-only
 */

#include "NMEA2000ServerNode.hpp"

#include "NMEA2000ServerCodec.hpp"
#include "ros2_isobus/topics.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>

#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace ros2_isobus
{

namespace
{
constexpr std::uint8_t kAddressClaimedState = 4U;

double diagnosticValue(
  const diagnostic_msgs::msg::DiagnosticArray & message,
  const std::string & key, bool & found)
{
  for (const auto & status : message.status) {
    for (const auto & value : status.values) {
      if (value.key == key) {
        try {
          found = true;
          return std::stod(value.value);
        } catch (const std::exception &) {
          found = false;
          return 0.0;
        }
      }
    }
  }
  return 0.0;
}
}  // namespace

NMEA2000Server::NMEA2000Server(const rclcpp::NodeOptions & options)
: Node("nmea2000_server", options)
{
  priority_ = static_cast<int>(
    std::clamp<std::int64_t>(declare_parameter<int>("priority", 2), 0, 7));
  const auto qos = rclcpp::SensorDataQoS();

  attitude_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    kNmea2000AttitudeTxTopic, qos,
    std::bind(&NMEA2000Server::onAttitude, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    kNmea2000ImuTxTopic, qos,
    std::bind(&NMEA2000Server::onImu, this, std::placeholders::_1));
  velocity_sub_ =
    create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    kNmea2000VelocityTxTopic, qos,
    std::bind(&NMEA2000Server::onVelocity, this, std::placeholders::_1));
  rapid_position_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    kNmea2000RapidPositionTxTopic, qos,
    std::bind(&NMEA2000Server::onRapidPosition, this, std::placeholders::_1));
  cog_sog_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    kNmea2000CogSogTxTopic, qos,
    std::bind(&NMEA2000Server::onCogSog, this, std::placeholders::_1));
  position_delta_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    kNmea2000PositionDeltaTxTopic, qos,
    std::bind(&NMEA2000Server::onPositionDelta, this, std::placeholders::_1));
  gnss_position_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    kNmea2000GnssPositionTxTopic, qos,
    std::bind(&NMEA2000Server::onGnssPosition, this, std::placeholders::_1));
  pseudo_noise_sub_ =
    create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    kNmea2000PseudoNoiseTxTopic, qos,
    std::bind(&NMEA2000Server::onPseudoNoise, this, std::placeholders::_1));
  address_sub_ = create_subscription<msg::IsobusAddressStatus>(
    kAddressManagerStatus, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&NMEA2000Server::onAddressStatus, this, std::placeholders::_1));
  bus_pub_ = create_publisher<msg::IsobusFrame>(kBusTxTopic, 100);

  RCLCPP_INFO(get_logger(), "NMEA2000Server ready; waiting for TX measurements");
}

void NMEA2000Server::onAttitude(
  const geometry_msgs::msg::Vector3Stamped & message)
{
  if (!ready("attitude")) {
    return;
  }
  NMEA2000ServerCodec::Payload payload;
  if (!NMEA2000ServerCodec::encodeAttitude(
      message.vector.x, message.vector.y, message.vector.z, nextSid(), payload))
  {
    RCLCPP_WARN(get_logger(), "Attitude not sent: invalid measurement");
    return;
  }
  publishPayload(NMEA2000ServerCodec::kAttitudePgn, payload);
}

void NMEA2000Server::onImu(const sensor_msgs::msg::Imu & message)
{
  const auto & q = message.orientation;
  const double roll = std::atan2(
    2.0 * (q.w * q.x + q.y * q.z),
    1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  const double pitch = std::asin(std::clamp(
    2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  if (!ready("attitude")) {
    return;
  }
  NMEA2000ServerCodec::Payload payload;
  if (NMEA2000ServerCodec::encodeAttitude(
      roll, pitch, yaw, nextSid(), payload))
  {
    publishPayload(NMEA2000ServerCodec::kAttitudePgn, payload);
  }
}

void NMEA2000Server::onVelocity(
  const geometry_msgs::msg::TwistWithCovarianceStamped & message)
{
  const double east = message.twist.twist.linear.x;
  const double north = message.twist.twist.linear.y;
  const double speed = std::hypot(east, north);
  const double compass = std::atan2(east, north);
  if (!ready("COG/SOG")) {
    return;
  }
  NMEA2000ServerCodec::Payload payload;
  if (NMEA2000ServerCodec::encodeCogSog(
      compass, speed, nextSid(), payload))
  {
    publishPayload(NMEA2000ServerCodec::kCogSogPgn, payload);
  }
}

void NMEA2000Server::onRapidPosition(
  const sensor_msgs::msg::NavSatFix & message)
{
  if (!ready("rapid position")) {
    return;
  }
  NMEA2000ServerCodec::Payload payload;
  if (!NMEA2000ServerCodec::encodeRapidPosition(
      message.latitude, message.longitude, payload))
  {
    RCLCPP_WARN(get_logger(), "Rapid position not sent: invalid coordinates");
    return;
  }
  publishPayload(NMEA2000ServerCodec::kRapidPositionPgn, payload);
}

void NMEA2000Server::onCogSog(
  const geometry_msgs::msg::TwistStamped & message)
{
  if (!ready("COG/SOG")) {
    return;
  }
  NMEA2000ServerCodec::Payload payload;
  if (!NMEA2000ServerCodec::encodeCogSog(
      message.twist.angular.z, message.twist.linear.x, nextSid(), payload))
  {
    RCLCPP_WARN(get_logger(), "COG/SOG not sent: invalid measurement");
    return;
  }
  publishPayload(NMEA2000ServerCodec::kCogSogPgn, payload);
}

void NMEA2000Server::onPositionDelta(
  const sensor_msgs::msg::NavSatFix & message)
{
  if (!ready("position delta")) {
    return;
  }
  NMEA2000ServerCodec::Payload payload;
  if (!NMEA2000ServerCodec::encodePositionDelta(
      message.latitude, message.longitude, nextSid(), payload))
  {
    RCLCPP_WARN(get_logger(), "Position delta not sent: invalid delta");
    return;
  }
  publishPayload(NMEA2000ServerCodec::kPositionDeltaPgn, payload);
}

void NMEA2000Server::onGnssPosition(
  const sensor_msgs::msg::NavSatFix & message)
{
  if (!ready("GNSS position")) {
    return;
  }

  const std::int64_t header_seconds =
    static_cast<std::int64_t>(message.header.stamp.sec);
  const std::int64_t stamp_seconds =
    header_seconds > 0 ? header_seconds : now().seconds();
  std::uint8_t gnss_type = 0U;
  const auto service = message.status.service;
  if ((service & sensor_msgs::msg::NavSatStatus::SERVICE_GPS) &&
    (service & sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS))
  {
    gnss_type = 2U;
  } else if (service & sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS) {
    gnss_type = 1U;
  } else if (service & sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO) {
    gnss_type = 8U;
  }
  const std::uint8_t fix_method =
    message.status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ?
    0U : 4U;
  const double hdop = message.position_covariance[0] > 0.0 ?
    std::sqrt(message.position_covariance[0]) : 0.0;
  const double pdop = message.position_covariance[8] > 0.0 ?
    std::sqrt(message.position_covariance[8]) : hdop;

  NMEA2000ServerCodec::Payload payload;
  if (!NMEA2000ServerCodec::encodeGnssPosition(
      message.latitude, message.longitude, message.altitude,
      stamp_seconds, gnss_type, fix_method, hdop, pdop, nextSid(), payload))
  {
    RCLCPP_WARN(get_logger(), "GNSS position not sent: invalid measurement");
    return;
  }
  publishPayload(NMEA2000ServerCodec::kGnssPositionPgn, payload);
}

void NMEA2000Server::onPseudoNoise(
  const diagnostic_msgs::msg::DiagnosticArray & message)
{
  if (!ready("GNSS pseudo noise")) {
    return;
  }
  const std::array<std::string, 7> keys = {
    "RMS_uncertainty", "STD_major", "STD_minor", "Orientation_major_rad",
    "STD_lat", "STD_lon", "STD_alt"};
  std::array<double, 7> values{};
  for (std::size_t index = 0; index < keys.size(); ++index) {
    bool found = false;
    values[index] = diagnosticValue(message, keys[index], found);
    if (!found) {
      RCLCPP_WARN(
        get_logger(), "GNSS pseudo noise not sent: missing key '%s'",
        keys[index].c_str());
      return;
    }
  }

  NMEA2000ServerCodec::Payload payload;
  if (!NMEA2000ServerCodec::encodePseudoNoise(
      values[0], values[1], values[2], values[3], values[4], values[5],
      values[6], nextSid(), payload))
  {
    RCLCPP_WARN(get_logger(), "GNSS pseudo noise not sent: invalid measurement");
    return;
  }
  publishPayload(NMEA2000ServerCodec::kPseudoNoisePgn, payload);
}

bool NMEA2000Server::ready(const char * measurement)
{
  if (address_claimed_) {
    return true;
  }
  if (!address_warning_reported_) {
    RCLCPP_WARN(
      get_logger(), "%s not sent: ISOBUS source address is not claimed",
      measurement);
    address_warning_reported_ = true;
  }
  return false;
}

void NMEA2000Server::publishPayload(
  std::uint32_t pgn, const std::vector<std::uint8_t> & payload)
{
  if (payload.empty() || payload.size() > 223U) {
    return;
  }

  const auto publish_frame =
    [this, pgn](const std::array<std::uint8_t, 8> & data)
    {
      msg::IsobusFrame frame;
      frame.timestamp = now();
      frame.priority = static_cast<std::uint8_t>(priority_);
      frame.page = true;
      frame.pgn = pgn;
      frame.sa = source_address_;
      frame.pf = static_cast<std::uint8_t>((pgn >> 8U) & 0xFFU);
      frame.ps = static_cast<std::uint8_t>(pgn & 0xFFU);
      frame.dlc = 8U;
      frame.data = data;
      bus_pub_->publish(frame);
    };

  if (payload.size() <= 8U) {
    std::array<std::uint8_t, 8> frame_data;
    frame_data.fill(0xFFU);
    std::copy(payload.begin(), payload.end(), frame_data.begin());
    publish_frame(frame_data);
    return;
  }

  const std::uint8_t sequence =
    static_cast<std::uint8_t>((fast_packet_sequence_++ & 0x07U) << 5U);
  std::array<std::uint8_t, 8> frame_data;
  frame_data.fill(0xFFU);
  frame_data[0] = sequence;
  frame_data[1] = static_cast<std::uint8_t>(payload.size());
  const auto first_count = std::min<std::size_t>(6U, payload.size());
  std::copy_n(payload.begin(), first_count, frame_data.begin() + 2);
  publish_frame(frame_data);

  std::size_t offset = first_count;
  std::uint8_t frame_index = 1U;
  while (offset < payload.size()) {
    frame_data.fill(0xFFU);
    frame_data[0] = static_cast<std::uint8_t>(sequence | frame_index++);
    const auto count = std::min<std::size_t>(7U, payload.size() - offset);
    std::copy_n(payload.begin() + offset, count, frame_data.begin() + 1);
    publish_frame(frame_data);
    offset += count;
  }
}

std::uint8_t NMEA2000Server::nextSid()
{
  const auto current = sequence_id_;
  sequence_id_ = sequence_id_ >= 252U ?
    0U : static_cast<std::uint8_t>(sequence_id_ + 1U);
  return current;
}

void NMEA2000Server::onAddressStatus(
  const msg::IsobusAddressStatus & message)
{
  source_address_ = message.sa;
  address_claimed_ =
    message.state == kAddressClaimedState && source_address_ < 0xFEU;
  if (address_claimed_) {
    address_warning_reported_ = false;
  }
}

}  // namespace ros2_isobus

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_isobus::NMEA2000Server>());
  rclcpp::shutdown();
  return 0;
}
