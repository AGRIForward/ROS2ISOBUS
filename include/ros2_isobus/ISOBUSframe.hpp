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

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>

#include "ros2_isobus/msg/isobus_frame.hpp"

namespace ros2_isobus
{

using ByteArray8 = std::array<std::uint8_t, 8>;

inline ByteArray8 make_byte_array(const std::uint8_t *data, std::size_t len)
{
  ByteArray8 result{};
  const auto copy_len = std::min(result.size(), len);
  std::copy_n(data, copy_len, result.begin());
  return result;
}

inline ByteArray8 make_byte_array(const char *data, std::size_t len)
{
  return make_byte_array(reinterpret_cast<const std::uint8_t *>(data), len);
}

inline ByteArray8 hex_to_byte_array(const std::string & hex)
{
  ByteArray8 result{};
  std::size_t idx = 0;
  for (std::size_t i = 0; i < result.size() && (idx + 1) < hex.size(); ++i, idx += 2) {
    const auto byte = hex.substr(idx, 2);
    result[i] = static_cast<std::uint8_t>(std::stoul(byte, nullptr, 16));
  }
  return result;
}

inline std::string to_hex(const ByteArray8 &arr)
{
  static constexpr char hex[] = "0123456789ABCDEF";
  std::string out;
  out.reserve(arr.size() * 2);
  for (auto byte : arr) {
    out.push_back(hex[(byte >> 4) & 0x0F]);
    out.push_back(hex[byte & 0x0F]);
  }
  return out;
}

inline std::string to_hex(std::uint32_t value)
{
  std::ostringstream oss;
  oss << std::hex << std::uppercase << value;
  return oss.str();
}

inline std::uint32_t get_can_id(const msg::IsobusFrame & frame)
{
  // Build 29-bit ID from explicit PF/PS if present, otherwise derive from PGN
  const std::uint8_t pf = frame.pf ? frame.pf : static_cast<std::uint8_t>((frame.pgn >> 8) & 0xFF);
  const std::uint8_t ps = frame.ps ? frame.ps : static_cast<std::uint8_t>(frame.pgn & 0xFF);
  const std::uint8_t page = static_cast<std::uint8_t>(frame.page ? 1 : 0);

  return (static_cast<std::uint32_t>(frame.priority) << 26) |
         (static_cast<std::uint32_t>(page) << 24) |
         (static_cast<std::uint32_t>(pf) << 16) |
         (static_cast<std::uint32_t>(ps) << 8) |
         (frame.sa);
}

inline msg::IsobusFrame make_isobus_frame_from_can(std::uint32_t id, const ByteArray8 & payload)
{
  msg::IsobusFrame msg;
  msg.sa = static_cast<std::uint8_t>((id >> 0) & 0xFF);
  msg.pgn = static_cast<std::uint32_t>((id >> 8) & 0xFFFF);
  msg.page = static_cast<bool>((id >> 24) & 0x01);
  msg.priority = static_cast<std::uint8_t>((id >> 26) & 0x07);
  msg.pf = static_cast<std::uint8_t>((msg.pgn >> 8) & 0xFF);
  msg.ps = static_cast<std::uint8_t>((msg.pgn >> 0) & 0xFF);
  msg.data = payload;
  return msg;
}

}  // namespace ros2_isobus
