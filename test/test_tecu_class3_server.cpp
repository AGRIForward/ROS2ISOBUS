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

#include "TECUClass3Server.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace
{
class TestServer : public ros2_isobus::TECUClass3Server
{
public:
  std::vector<ros2_isobus::msg::IsobusFrame> frames;
  ros2_isobus::msg::TecuCruiseCommand cruise;
  ros2_isobus::msg::TecuGuidanceCommand guidance;
  ros2_isobus::msg::TecuRearHitchCommand hitch;
  ros2_isobus::msg::TecuRearPtoCommand pto;
  ros2_isobus::msg::AuxValveCommand valve;
  double twist_speed{0.0};
  double twist_yaw_rate{0.0};
  unsigned cruise_count{0};
  unsigned guidance_count{0};
  unsigned hitch_count{0};
  unsigned pto_count{0};
  unsigned valve_count{0};

protected:
  void sendFrame(const ros2_isobus::msg::IsobusFrame & value) override {frames.push_back(value);}
  void publishCruiseCommand(const ros2_isobus::msg::TecuCruiseCommand & value) override
  {cruise = value; ++cruise_count;}
  void publishGuidanceCommand(const ros2_isobus::msg::TecuGuidanceCommand & value) override
  {guidance = value; ++guidance_count;}
  void publishRearHitchCommand(const ros2_isobus::msg::TecuRearHitchCommand & value) override
  {hitch = value; ++hitch_count;}
  void publishRearPtoCommand(const ros2_isobus::msg::TecuRearPtoCommand & value) override
  {pto = value; ++pto_count;}
  void publishAuxValveCommand(const ros2_isobus::msg::AuxValveCommand & value) override
  {valve = value; ++valve_count;}
  void publishTwistCommand(double speed, double yaw_rate) override {twist_speed = speed; twist_yaw_rate = yaw_rate;}
  void printWarn(const std::string &) override {}
};

ros2_isobus::msg::IsobusFrame frame(std::uint32_t pgn)
{
  ros2_isobus::msg::IsobusFrame value;
  value.pgn = pgn;
  value.pf = static_cast<std::uint8_t>(pgn >> 8U);
  value.ps = static_cast<std::uint8_t>(pgn);
  value.sa = 0x1CU;
  value.dlc = 8U;
  value.data.fill(0xFFU);
  return value;
}
}  // namespace

TEST(TecuClass3Server, WaitsForAddressManagerBeforeSending)
{
  TestServer server;
  server.run();
  EXPECT_TRUE(server.frames.empty());
  server.setSourceAddress(240U);
  server.setAddressClaimed(true);
  server.setTwist(-1.25, 0.25, 1.0);
  server.run();
  ASSERT_FALSE(server.frames.empty());
  const auto wheel = std::find_if(
    server.frames.begin(), server.frames.end(),
    [](const auto & value) {return value.pgn == 0xFE48U;});
  ASSERT_NE(wheel, server.frames.end());
  EXPECT_EQ(wheel->sa, 240U);
  EXPECT_EQ(wheel->data[0], 0xE2U);
  EXPECT_EQ(wheel->data[1], 0x04U);
  EXPECT_EQ(wheel->data[7] & 0x03U, 0U);
}

TEST(TecuClass3Server, DecodesStandardGuidanceUnits)
{
  TestServer server;
  server.setSourceAddress(240U);
  server.setAddressClaimed(true);
  server.setGuidanceAvailable(true, false);
  auto command = frame(0xADF0U);
  command.pf = 0xADU;
  command.ps = 240U;
  const std::uint16_t raw = 31628U;  // ROS +0.125 1/m = ISO -125 1/km.
  command.data[0] = static_cast<std::uint8_t>(raw);
  command.data[1] = static_cast<std::uint8_t>(raw >> 8U);
  command.data[2] = 1U;
  server.HandleMsg(command);
  EXPECT_NEAR(server.guidance.curvature, 0.125, 0.00013);
}

TEST(TecuClass3Server, DecodesClassThreeHitchPtoAndValveCommands)
{
  TestServer server;
  server.setSourceAddress(240U);
  server.setAddressClaimed(true);

  auto hitch_pto = frame(0xFE42U);
  hitch_pto.data[1] = 125U;
  hitch_pto.data[4] = 0xE0U;
  hitch_pto.data[5] = 0x2EU;  // 1500 rpm at 0.125 rpm/bit.
  hitch_pto.data[6] = 0xDFU;
  server.HandleMsg(hitch_pto);
  EXPECT_DOUBLE_EQ(server.hitch.position_percent, 50.0);
  EXPECT_DOUBLE_EQ(server.pto.rpm, 1500.0);
  EXPECT_TRUE(server.pto.engagement);

  auto valve = frame(0xFE31U);
  valve.data[0] = 100U;
  valve.data[2] = 2U;
  server.HandleMsg(valve);
  EXPECT_EQ(server.valve.valve_number, 1U);
  EXPECT_FLOAT_EQ(server.valve.flow_percent, -40.0F);
}

TEST(TecuClass3Server, ClassOneAdvertisesAndSendsOnlyClassOneData)
{
  TestServer server;
  ros2_isobus::TECUClass3Server::Configuration configuration;
  configuration.tecu_class = 1U;
  server.setConfiguration(configuration);
  server.setSourceAddress(240U);
  server.setAddressClaimed(true);
  server.run();

  const auto facilities = std::find_if(
    server.frames.begin(), server.frames.end(),
    [](const auto & value) {return value.pgn == 0xFE09U;});
  ASSERT_NE(facilities, server.frames.end());
  EXPECT_EQ(facilities->data[0] & 0xC0U, 0U);
  EXPECT_EQ(facilities->data[2], 0U);

  const auto wheel = std::find_if(
    server.frames.begin(), server.frames.end(),
    [](const auto & value) {return value.pgn == 0xFE48U;});
  ASSERT_NE(wheel, server.frames.end());
  EXPECT_TRUE(std::all_of(wheel->data.begin() + 2, wheel->data.begin() + 6,
    [](auto byte) {return byte == 0xFFU;}));
  EXPECT_EQ(wheel->data[7] & 0x03U, 3U);
  EXPECT_EQ(std::count_if(
      server.frames.begin(), server.frames.end(),
      [](const auto & value) {return value.pgn == 0xFE10U;}), 0);
}

TEST(TecuClass3Server, ClassTwoAdvertisesClassTwoAndRejectsClassThreeCommands)
{
  TestServer server;
  ros2_isobus::TECUClass3Server::Configuration configuration;
  configuration.tecu_class = 2U;
  configuration.aux_valve_count = 2U;
  server.setConfiguration(configuration);
  server.setSourceAddress(240U);
  server.setAddressClaimed(true);

  auto command = frame(0xFE31U);
  command.data[0] = 100U;
  command.data[2] = 1U;
  server.HandleMsg(command);
  EXPECT_EQ(server.valve_count, 0U);
  ASSERT_FALSE(server.frames.empty());
  const auto & nack = server.frames.back();
  EXPECT_EQ(nack.pf, 0xE8U);
  EXPECT_EQ(nack.ps, command.sa);
  EXPECT_EQ(nack.data[0], 1U);
  EXPECT_EQ(nack.data[5], 0x31U);
  EXPECT_EQ(nack.data[6], 0xFEU);

  for (unsigned cycle = 0; cycle < 10U; ++cycle) server.run();
  const auto facilities = std::find_if(
    server.frames.begin(), server.frames.end(),
    [](const auto & value) {return value.pgn == 0xFE09U;});
  ASSERT_NE(facilities, server.frames.end());
  EXPECT_EQ(facilities->data[0] & 0xC0U, 0x40U);
  EXPECT_NE(std::find_if(
      server.frames.begin(), server.frames.end(),
      [](const auto & value) {return value.pgn == 0xFEE6U;}), server.frames.end());
}

TEST(TecuClass3Server, ClassThreeFacilitiesFollowConfiguredHardware)
{
  TestServer server;
  ros2_isobus::TECUClass3Server::Configuration configuration;
  configuration.tecu_class = 3U;
  configuration.enable_guidance = false;
  configuration.enable_rear_hitch = false;
  configuration.enable_rear_pto = false;
  configuration.aux_valve_count = 0U;
  server.setConfiguration(configuration);
  server.setSourceAddress(240U);
  server.setAddressClaimed(true);

  const auto facilities = std::find_if(
    server.frames.begin(), server.frames.end(),
    [](const auto & value) {return value.pgn == 0xFE09U;});
  ASSERT_NE(facilities, server.frames.end());
  EXPECT_EQ(facilities->data[0] & 0xC0U, 0x80U);
  EXPECT_EQ(facilities->data[3] & 0xF8U, 0U);
  EXPECT_EQ(facilities->data[6] & 0x02U, 0U);
}

TEST(TecuClass3Server, MissingMeasurementsUseNotAvailableEncoding)
{
  TestServer server;
  server.setSourceAddress(240U);
  server.setAddressClaimed(true);
  server.run();

  const auto wheel = std::find_if(
    server.frames.begin(), server.frames.end(),
    [](const auto & value) {return value.pgn == 0xFE48U;});
  ASSERT_NE(wheel, server.frames.end());
  EXPECT_TRUE(std::all_of(
      wheel->data.begin(), wheel->data.begin() + 6,
      [](auto byte) {return byte == 0xFFU;}));
  EXPECT_EQ(wheel->data[6], 0xFFU);
  EXPECT_EQ((wheel->data[7] >> 2U) & 0x03U, 3U);

  const auto engine = std::find_if(
    server.frames.begin(), server.frames.end(),
    [](const auto & value) {return value.pgn == 0xF004U;});
  ASSERT_NE(engine, server.frames.end());
  EXPECT_EQ(engine->data[3], 0xFFU);
  EXPECT_EQ(engine->data[4], 0xFFU);

  const auto hitch = std::find_if(
    server.frames.begin(), server.frames.end(),
    [](const auto & value) {return value.pgn == 0xFE45U;});
  ASSERT_NE(hitch, server.frames.end());
  EXPECT_TRUE(std::all_of(
      hitch->data.begin(), hitch->data.end(),
      [](auto byte) {return byte == 0xFFU;}));
}

TEST(TecuClass3Server, MeasurementsExpireBackToNotAvailable)
{
  TestServer server;
  server.setInputTimeoutTicks(1U);
  server.setEngineSpeed(1500.0);
  server.setMaximumPowerTime(42U);
  server.setKeySwitch(true);
  server.setSourceAddress(240U);
  server.setAddressClaimed(true);
  server.run();

  const auto first_engine = std::find_if(
    server.frames.begin(), server.frames.end(),
    [](const auto & value) {return value.pgn == 0xF004U;});
  ASSERT_NE(first_engine, server.frames.end());
  EXPECT_EQ(first_engine->data[3], 0xE0U);
  EXPECT_EQ(first_engine->data[4], 0x2EU);

  server.frames.clear();
  server.run();
  const auto expired_wheel = std::find_if(
    server.frames.begin(), server.frames.end(),
    [](const auto & value) {return value.pgn == 0xFE48U;});
  ASSERT_NE(expired_wheel, server.frames.end());
  EXPECT_EQ(expired_wheel->data[6], 0xFFU);
  EXPECT_EQ((expired_wheel->data[7] >> 2U) & 0x03U, 3U);
  const auto expired_engine = std::find_if(
    server.frames.begin(), server.frames.end(),
    [](const auto & value) {return value.pgn == 0xF004U;});
  ASSERT_NE(expired_engine, server.frames.end());
  EXPECT_EQ(expired_engine->data[3], 0xFFU);
  EXPECT_EQ(expired_engine->data[4], 0xFFU);
}
