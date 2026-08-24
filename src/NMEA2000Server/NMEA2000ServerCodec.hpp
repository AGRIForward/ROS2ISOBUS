/*
 *  This file is part of ROS2ISOBUS
 *
 *  Copyright 2026 Juha Backman / Natural Resources Institute Finland
 *
 *  SPDX-License-Identifier: LGPL-3.0-only
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace ros2_isobus
{

class NMEA2000ServerCodec
{
public:
  using Payload = std::vector<std::uint8_t>;

  static constexpr std::uint32_t kAttitudePgn = 0xF119U;
  static constexpr std::uint32_t kRapidPositionPgn = 0xF801U;
  static constexpr std::uint32_t kCogSogPgn = 0xF802U;
  static constexpr std::uint32_t kPositionDeltaPgn = 0xF803U;
  static constexpr std::uint32_t kGnssPositionPgn = 0xF805U;
  static constexpr std::uint32_t kPseudoNoisePgn = 0xFA06U;

  static bool encodeAttitude(
    double roll, double pitch, double yaw, std::uint8_t sid, Payload & data)
  {
    if (!finite(roll, pitch, yaw)) {
      return false;
    }
    constexpr double resolution = 1e-4;
    constexpr double two_pi = 6.28318530717958647692;
    double normalized_yaw = std::fmod(yaw, two_pi);
    if (normalized_yaw < 0.0) {
      normalized_yaw += two_pi;
    }
    const auto yaw_raw = std::llround(normalized_yaw / resolution);
    const auto pitch_raw = std::llround(pitch / resolution);
    const auto roll_raw = std::llround(roll / resolution);
    if (yaw_raw < 0 || yaw_raw >= 0xFFFF ||
      !validSigned16(pitch_raw) || !validSigned16(roll_raw))
    {
      return false;
    }
    data.assign(8, 0xFFU);
    data[0] = sid;
    putUnsigned(data, 1, static_cast<std::uint64_t>(yaw_raw), 2);
    putSigned(data, 3, pitch_raw, 2);
    putSigned(data, 5, roll_raw, 2);
    return true;
  }

  static bool encodeRapidPosition(double latitude, double longitude, Payload & data)
  {
    if (!validCoordinates(latitude, longitude)) {
      return false;
    }
    const auto lat = std::llround(latitude / 1e-7);
    const auto lon = std::llround(longitude / 1e-7);
    if (!validSigned32(lat) || !validSigned32(lon)) {
      return false;
    }
    data.assign(8, 0xFFU);
    putSigned(data, 0, lat, 4);
    putSigned(data, 4, lon, 4);
    return true;
  }

  static bool encodeCogSog(
    double course_rad, double speed_ms, std::uint8_t sid, Payload & data)
  {
    if (!std::isfinite(course_rad) || !std::isfinite(speed_ms) || speed_ms < 0.0) {
      return false;
    }
    constexpr double two_pi = 6.28318530717958647692;
    double course = std::fmod(course_rad, two_pi);
    if (course < 0.0) {
      course += two_pi;
    }
    const auto cog = std::llround(course / 1e-4);
    const auto sog = std::llround(speed_ms / 1e-2);
    if (cog >= 0xFFFF || sog < 0 || sog >= 0xFFFF) {
      return false;
    }
    data.assign(8, 0xFFU);
    data[0] = sid;
    data[1] = 0xFCU;  // true-reference COG, reserved bits set
    putUnsigned(data, 2, static_cast<std::uint64_t>(cog), 2);
    putUnsigned(data, 4, static_cast<std::uint64_t>(sog), 2);
    return true;
  }

  // Latitude and longitude are deltas in degrees, not absolute coordinates.
  static bool encodePositionDelta(
    double latitude_delta, double longitude_delta, std::uint8_t sid,
    Payload & data)
  {
    if (!std::isfinite(latitude_delta) || !std::isfinite(longitude_delta)) {
      return false;
    }
    const auto lat = std::llround(latitude_delta * 3600.0 / 1e-5);
    const auto lon = std::llround(longitude_delta * 3600.0 / 1e-5);
    if (!validSigned24(lat) || !validSigned24(lon)) {
      return false;
    }
    data.assign(8, 0xFFU);
    data[0] = sid;
    data[1] = 0U;
    putSigned(data, 2, lat, 3);
    putSigned(data, 5, lon, 3);
    return true;
  }

  static bool encodeGnssPosition(
    double latitude, double longitude, double altitude,
    std::int64_t stamp_seconds, std::uint8_t gnss_type,
    std::uint8_t fix_method, double hdop, double pdop,
    std::uint8_t sid, Payload & data)
  {
    if (!validCoordinates(latitude, longitude) || !std::isfinite(altitude)) {
      return false;
    }
    const auto lat = std::llround(latitude / 1e-16);
    const auto lon = std::llround(longitude / 1e-16);
    const auto alt = std::llround(altitude / 1e-6);
    if (stamp_seconds < 0 || !validSigned64(lat) || !validSigned64(lon) ||
      !validSigned64(alt))
    {
      return false;
    }
    const auto days = static_cast<std::uint64_t>(stamp_seconds / 86400);
    const auto seconds_in_day = static_cast<std::uint64_t>(stamp_seconds % 86400);
    if (days >= 0xFFFFU) {
      return false;
    }
    data.assign(43, 0xFFU);
    data[0] = sid;
    putUnsigned(data, 1, days, 2);
    putUnsigned(data, 3, seconds_in_day * 10000U, 4);
    putSigned(data, 7, lat, 8);
    putSigned(data, 15, lon, 8);
    putSigned(data, 23, alt, 8);
    data[31] = static_cast<std::uint8_t>(
      (fix_method & 0x0FU) << 4U | (gnss_type & 0x0FU));
    data[32] = 0xFCU;  // integrity safe, reserved bits set
    data[33] = 0xFFU;  // satellite count unavailable in NavSatFix
    putScaledUnsigned16(data, 34, hdop, 1e-2);
    putScaledUnsigned16(data, 36, pdop, 1e-2);
    return true;
  }

  static bool encodePseudoNoise(
    double rms, double std_major, double std_minor, double orientation,
    double std_lat, double std_lon, double std_alt, std::uint8_t sid,
    Payload & data)
  {
    if (!finite(rms, std_major, std_minor) ||
      !finite(orientation, std_lat, std_lon) || !std::isfinite(std_alt))
    {
      return false;
    }
    data.assign(15, 0xFFU);
    data[0] = sid;
    if (!putScaledUnsigned16(data, 1, rms, 1e-2) ||
      !putScaledUnsigned16(data, 3, std_major, 1e-2) ||
      !putScaledUnsigned16(data, 5, std_minor, 1e-2) ||
      !putScaledUnsigned16(data, 7, orientation, 1e-4) ||
      !putScaledUnsigned16(data, 9, std_lat, 1e-2) ||
      !putScaledUnsigned16(data, 11, std_lon, 1e-2) ||
      !putScaledUnsigned16(data, 13, std_alt, 1e-2))
    {
      return false;
    }
    return true;
  }

private:
  static bool finite(double a, double b, double c)
  {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
  }

  static bool validCoordinates(double latitude, double longitude)
  {
    return finite(latitude, longitude, 0.0) &&
           latitude >= -90.0 && latitude <= 90.0 &&
           longitude >= -180.0 && longitude <= 180.0;
  }

  static bool validSigned16(std::int64_t value)
  {
    return value >= std::numeric_limits<std::int16_t>::min() &&
           value <= 32766;
  }

  static bool validSigned24(std::int64_t value)
  {
    return value >= -8388608LL && value <= 8388606LL;
  }

  static bool validSigned32(std::int64_t value)
  {
    return value >= std::numeric_limits<std::int32_t>::min() &&
           value <= 2147483646LL;
  }

  static bool validSigned64(long double value)
  {
    return value >= static_cast<long double>(std::numeric_limits<std::int64_t>::min()) &&
           value <= static_cast<long double>(std::numeric_limits<std::int64_t>::max() - 1);
  }

  static void putUnsigned(
    Payload & data, std::size_t offset, std::uint64_t value, std::size_t bytes)
  {
    for (std::size_t index = 0; index < bytes; ++index) {
      data[offset + index] =
        static_cast<std::uint8_t>((value >> (8U * index)) & 0xFFU);
    }
  }

  static void putSigned(
    Payload & data, std::size_t offset, std::int64_t value, std::size_t bytes)
  {
    putUnsigned(data, offset, static_cast<std::uint64_t>(value), bytes);
  }

  static bool putScaledUnsigned16(
    Payload & data, std::size_t offset, double value, double resolution)
  {
    if (!std::isfinite(value) || value < 0.0) {
      return false;
    }
    const auto raw = std::llround(value / resolution);
    if (raw < 0 || raw >= 0xFFFF) {
      return false;
    }
    putUnsigned(data, offset, static_cast<std::uint64_t>(raw), 2);
    return true;
  }
};

}  // namespace ros2_isobus
