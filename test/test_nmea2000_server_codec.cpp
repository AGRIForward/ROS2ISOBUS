/*
 *  This file is part of ROS2ISOBUS
 *
 *  Copyright 2026 Juha Backman / Natural Resources Institute Finland
 *
 *  SPDX-License-Identifier: LGPL-3.0-only
 */

#include <gtest/gtest.h>

#include "NMEA2000ServerCodec.hpp"

#include <cmath>
#include <limits>

using Codec = ros2_isobus::NMEA2000ServerCodec;

TEST(NMEA2000ServerCodec, EncodesAttitudePgn127257)
{
  Codec::Payload data;
  ASSERT_TRUE(Codec::encodeAttitude(-0.2, 0.1, 1.5, 7, data));
  EXPECT_EQ(data, (Codec::Payload{
      7, 0x98, 0x3A, 0xE8, 0x03, 0x30, 0xF8, 0xFF}));
}

TEST(NMEA2000ServerCodec, EncodesSignedRapidPositionPgn129025)
{
  Codec::Payload data;
  ASSERT_TRUE(Codec::encodeRapidPosition(-60.1234567, 24.1234567, data));
  const auto latitude =
    static_cast<std::int32_t>(
    static_cast<std::uint32_t>(data[0]) |
    static_cast<std::uint32_t>(data[1]) << 8U |
    static_cast<std::uint32_t>(data[2]) << 16U |
    static_cast<std::uint32_t>(data[3]) << 24U);
  EXPECT_EQ(latitude, -601234567);
  EXPECT_EQ(data.size(), 8U);
}

TEST(NMEA2000ServerCodec, EncodesCogSogPgn129026)
{
  Codec::Payload data;
  ASSERT_TRUE(Codec::encodeCogSog(1.5, 2.25, 3, data));
  EXPECT_EQ(data[0], 3);
  EXPECT_EQ(data[2], 0x98);
  EXPECT_EQ(data[3], 0x3A);
  EXPECT_EQ(data[4], 225);
  EXPECT_EQ(data.size(), 8U);
}

TEST(NMEA2000ServerCodec, EncodesPositionDeltaPgn129027)
{
  Codec::Payload data;
  ASSERT_TRUE(Codec::encodePositionDelta(1e-5, -1e-5, 4, data));
  EXPECT_EQ(data[0], 4);
  EXPECT_EQ(data.size(), 8U);
}

TEST(NMEA2000ServerCodec, EncodesGnssFastPacketPayloadPgn129029)
{
  Codec::Payload data;
  ASSERT_TRUE(Codec::encodeGnssPosition(
      60.123, 24.456, 12.5, 172800, 0, 4, 0.8, 1.2, 5, data));
  EXPECT_EQ(data.size(), 43U);
  EXPECT_EQ(data[0], 5);
  EXPECT_EQ(data[1], 2);
  EXPECT_EQ(data[2], 0);
  EXPECT_EQ(data[31], 0x40);
}

TEST(NMEA2000ServerCodec, EncodesPseudoNoiseFastPacketPayloadPgn129539)
{
  Codec::Payload data;
  ASSERT_TRUE(Codec::encodePseudoNoise(
      0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 6, data));
  EXPECT_EQ(data.size(), 15U);
  EXPECT_EQ(data[0], 6);
  EXPECT_EQ(data[1], 10);
  EXPECT_EQ(data[7], 0xA0);
  EXPECT_EQ(data[8], 0x0F);
}

TEST(NMEA2000ServerCodec, RejectsMissingMeasurement)
{
  Codec::Payload data;
  EXPECT_FALSE(Codec::encodeAttitude(
      0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0, data));
}
