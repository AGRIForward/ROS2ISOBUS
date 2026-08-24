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

#include <algorithm>
#include <cmath>
#include <limits>
#include <ctime>

namespace ros2_isobus
{
namespace
{
using Payload = std::array<std::uint8_t, 8>;

std::uint16_t getU16(const Payload & data, unsigned offset)
{
  return static_cast<std::uint16_t>(data[offset]) |
    static_cast<std::uint16_t>(data[offset + 1]) << 8U;
}

void putU16(Payload & data, unsigned offset, std::uint16_t value)
{
  data[offset] = static_cast<std::uint8_t>(value);
  data[offset + 1] = static_cast<std::uint8_t>(value >> 8U);
}

void putU32(Payload & data, unsigned offset, std::uint32_t value)
{
  for (unsigned byte = 0; byte < 4; ++byte) {
    data[offset + byte] = static_cast<std::uint8_t>(value >> (byte * 8U));
  }
}

std::uint16_t speedRaw(double speed)
{
  return static_cast<std::uint16_t>(
    std::clamp(std::llround(std::abs(speed) * 1000.0), 0LL, 64255LL));
}

std::uint16_t curvatureRaw(double ros_curvature)
{
  const double iso_km_inverse = -ros_curvature * 1000.0;
  return static_cast<std::uint16_t>(std::clamp(
    std::llround((iso_km_inverse + 8032.0) * 4.0), 0LL, 65535LL));
}

double decodeCurvature(std::uint16_t raw)
{
  return -(static_cast<double>(raw) * 0.25 - 8032.0) / 1000.0;
}

void setBit(Payload & data, unsigned bit)
{
  data[bit / 8U] |= static_cast<std::uint8_t>(1U << (bit % 8U));
}
}  // namespace

TECUClass3Server::TECUClass3Server()
{
  hitch_status_.in_work = 3U;
  hitch_status_.position_limit_status = 7U;
  pto_status_.engagement = 3U;
  pto_status_.mode = 3U;
  pto_status_.economy_mode = 3U;
  pto_status_.engagement_request = 3U;
  pto_status_.mode_request = 3U;
  pto_status_.economy_request = 3U;
  pto_status_.speed_limit_status = 3U;
  for (std::uint8_t index = 0; index < valve_status_.size(); ++index) {
    valve_status_[index].valve_number = index;
    valve_status_[index].state = 15U;
  }
}

void TECUClass3Server::setSourceAddress(std::uint8_t address) {source_address_ = address;}
void TECUClass3Server::setAddressClaimed(bool claimed)
{
  const bool became_claimed = claimed && !address_claimed_;
  address_claimed_ = claimed;
  if (became_claimed) sendFacilitiesResponse();
}
void TECUClass3Server::setKeySwitch(bool active)
{
  key_switch_active_ = active;
  key_switch_valid_ = true;
  key_switch_age_ = 0U;
}

void TECUClass3Server::setEngineSpeed(double rpm)
{
  engine_speed_rpm_ = rpm;
  engine_speed_valid_ = std::isfinite(rpm) && rpm >= 0.0;
  engine_speed_age_ = 0U;
}

void TECUClass3Server::setMaximumPowerTime(std::uint8_t minutes)
{
  maximum_power_time_min_ = std::min<std::uint8_t>(minutes, 250U);
  maximum_power_time_valid_ = minutes <= 250U;
  maximum_power_time_age_ = 0U;
}

void TECUClass3Server::setInputTimeoutTicks(unsigned ticks)
{
  input_timeout_ticks_ = std::max(1U, ticks);
}
void TECUClass3Server::setValveCount(std::uint8_t count) {valve_count_ = std::min<std::uint8_t>(count, 16U);}

void TECUClass3Server::setConfiguration(const Configuration & configuration)
{
  configuration_ = configuration;
  configuration_.tecu_class = std::clamp<std::uint8_t>(configuration_.tecu_class, 1U, 3U);
  valve_count_ = configuration_.tecu_class >= 2U ?
    std::min<std::uint8_t>(configuration_.aux_valve_count, 16U) : 0U;
}

void TECUClass3Server::setGuidanceAvailable(bool ready, bool lockout)
{
  guidance_ready_ = ready;
  mechanical_lockout_ = lockout;
  guidance_available_valid_ = true;
  guidance_available_age_ = 0U;
}

void TECUClass3Server::setTwist(double speed, double yaw_rate, double elapsed)
{
  twist_valid_ = std::isfinite(speed) && std::isfinite(yaw_rate);
  twist_age_ = 0U;
  if (!twist_valid_) return;
  speed_m_s_ = speed;
  curvature_m_inv_ = std::abs(speed) > 1e-6 ? yaw_rate / speed : 0.0;
  distance_m_ += std::abs(speed) * std::clamp(elapsed, 0.0, 1.0);
}

void TECUClass3Server::setRearHitchStatus(const msg::TecuRearHitchStatus & value)
{
  hitch_status_ = value;
  hitch_valid_ = true;
  hitch_age_ = 0U;
}

void TECUClass3Server::setRearPtoStatus(const msg::TecuRearPtoStatus & value)
{
  pto_status_ = value;
  pto_valid_ = true;
  pto_age_ = 0U;
}

void TECUClass3Server::setAuxValveStatus(const msg::AuxValveStatus & value)
{
  if (value.valve_number < valve_status_.size()) {
    valve_status_[value.valve_number] = value;
    valve_valid_[value.valve_number] = true;
    valve_age_[value.valve_number] = 0U;
  }
}

void TECUClass3Server::HandleMsg(const msg::IsobusFrame & frame)
{
  // PDU1 frames must address this T-ECU or use the global destination.
  const bool addressed = frame.pf >= 240U || frame.ps == source_address_ || frame.ps == 0xFFU;
  if (!addressed || !address_claimed_) return;
  if (frame.pgn == 0xFE08U) {
    sendFacilitiesResponse();
  } else if (frame.pgn == 0xEA00U) {
    const auto requested = static_cast<std::uint32_t>(frame.data[0]) |
      static_cast<std::uint32_t>(frame.data[1]) << 8U |
      static_cast<std::uint32_t>(frame.data[2]) << 16U;
    if (requested == 0xFE09U) sendFacilitiesResponse();
    else if (requested == 0xFE0FU) sendLanguage();
    else if (requested == 0xFEE6U && configuration_.tecu_class >= 2U) sendTimeDate();
  } else if (frame.pgn == 0xFE0BU) {
    if (configuration_.tecu_class == 3U && configuration_.enable_cruise) {
      decodeCruiseCommand(frame);
    } else {
      sendNegativeAcknowledgement(frame);
    }
  } else if (frame.pf == 0xADU) {
    if (configuration_.tecu_class == 3U && configuration_.enable_guidance) {
      decodeGuidanceCommand(frame);
    } else {
      sendNegativeAcknowledgement(frame);
    }
  } else if (frame.pgn == 0xFE42U) {
    if (configuration_.tecu_class == 3U &&
      (configuration_.enable_rear_hitch || configuration_.enable_rear_pto))
    {
      decodeHitchPtoCommand(frame);
    } else {
      sendNegativeAcknowledgement(frame);
    }
  } else if (frame.pgn >= 0xFE30U && frame.pgn <= 0xFE3FU) {
    const auto valve = static_cast<std::uint8_t>(frame.pgn & 0x0FU);
    if (configuration_.tecu_class == 3U && valve < valve_count_) {
      decodeValveCommand(frame);
    } else {
      sendNegativeAcknowledgement(frame);
    }
  } else if (frame.pgn == 0xFE51U && configuration_.tecu_class >= 2U &&
    configuration_.enable_lighting && (frame.data[7] & 0x03U) == 1U)
  {
    Payload lighting;
    lighting.fill(0xFFU);
    sendFrame(pdu2(0xFE50U, lighting));
  }
}

void TECUClass3Server::decodeCruiseCommand(const msg::IsobusFrame & frame)
{
  const auto raw = getU16(frame.data, 2);
  if (raw == 0xFFFFU) return;
  commanded_speed_m_s_ = raw > 32767U ?
    -static_cast<double>(65535U - raw) * 0.001 : static_cast<double>(raw) * 0.001;
  msg::TecuCruiseCommand command;
  command.speed = commanded_speed_m_s_;
  command.max_speed = 64.255;
  cruise_command_age_ = 0;
  publishCruiseCommand(command);
  publishTwistCommand(commanded_speed_m_s_, commanded_speed_m_s_ * commanded_curvature_m_inv_);
}

void TECUClass3Server::decodeGuidanceCommand(const msg::IsobusFrame & frame)
{
  const auto status = frame.data[2] & 0x03U;
  guidance_destination_ = frame.sa;
  const bool available = guidance_available_valid_ && guidance_ready_ && !mechanical_lockout_;
  commanded_curvature_m_inv_ = status == 1U && available ?
    decodeCurvature(getU16(frame.data, 0)) : 0.0;
  msg::TecuGuidanceCommand command;
  command.curvature = commanded_curvature_m_inv_;
  guidance_command_age_ = status == 1U ? 0 : -1;
  publishGuidanceCommand(command);
  publishTwistCommand(commanded_speed_m_s_, commanded_speed_m_s_ * commanded_curvature_m_inv_);
}

void TECUClass3Server::decodeHitchPtoCommand(const msg::IsobusFrame & frame)
{
  if (configuration_.enable_rear_hitch && frame.data[1] <= 250U) {
    msg::TecuRearHitchCommand command;
    command.position_percent = static_cast<double>(frame.data[1]) * 0.4;
    publishRearHitchCommand(command);
  }
  const auto rpm_raw = getU16(frame.data, 4);
  if (configuration_.enable_rear_pto && rpm_raw != 0xFFFFU) {
    msg::TecuRearPtoCommand command;
    command.rpm = static_cast<double>(rpm_raw) * 0.125;
    command.engagement = ((frame.data[6] >> 4U) & 0x03U) == 1U;
    publishRearPtoCommand(command);
  }
}

void TECUClass3Server::decodeValveCommand(const msg::IsobusFrame & frame)
{
  const auto valve = static_cast<std::uint8_t>(frame.pgn & 0x0FU);
  if (valve >= valve_count_) return;
  msg::AuxValveCommand command;
  command.valve_number = valve;
  const auto state = frame.data[2] & 0x0FU;
  command.floating = state == 3U;
  command.failsafe = ((frame.data[2] >> 6U) & 1U) != 0U;
  const float magnitude = static_cast<float>(frame.data[0]) * 0.4F;
  command.flow_percent = state == 2U ? -magnitude : (state == 1U ? magnitude : 0.0F);
  publishAuxValveCommand(command);
}

void TECUClass3Server::run()
{
  // Do not transmit before AddressManager has completed address claiming.
  if (!address_claimed_) return;
  const auto expire = [this](bool & valid, unsigned & age) {
      if (valid && ++age > input_timeout_ticks_) valid = false;
    };
  expire(twist_valid_, twist_age_);
  expire(hitch_valid_, hitch_age_);
  expire(pto_valid_, pto_age_);
  expire(engine_speed_valid_, engine_speed_age_);
  expire(maximum_power_time_valid_, maximum_power_time_age_);
  expire(key_switch_valid_, key_switch_age_);
  expire(guidance_available_valid_, guidance_available_age_);
  for (std::uint8_t valve = 0U; valve < valve_count_; ++valve) {
    expire(valve_valid_[valve], valve_age_[valve]);
  }
  bool changed = false;
  if (cruise_command_age_ >= 0 && ++cruise_command_age_ > 3) {
    commanded_speed_m_s_ = 0.0;
    cruise_command_age_ = -1;
    msg::TecuCruiseCommand command;
    command.speed = 0.0;
    command.max_speed = 64.255;
    publishCruiseCommand(command);
    changed = true;
  }
  if (guidance_command_age_ >= 0 && ++guidance_command_age_ > 3) {
    commanded_curvature_m_inv_ = 0.0;
    guidance_command_age_ = -1;
    msg::TecuGuidanceCommand command;
    command.curvature = 0.0;
    publishGuidanceCommand(command);
    changed = true;
  }
  if (changed) {
    publishTwistCommand(
      commanded_speed_m_s_, commanded_speed_m_s_ * commanded_curvature_m_inv_);
  }
  sendPeriodicStatus();
  if (++periodic_counter_ % 10U == 0U && configuration_.tecu_class >= 2U) {
    sendTimeDate();
  }
}

void TECUClass3Server::sendFacilitiesResponse()
{
  // ISO 11783-9 facilities bits are generated from the configured class and
  // installed hardware instead of advertising the complete Class 3 superset.
  Payload data{};
  setBit(data, 0); setBit(data, 1); setBit(data, 2); setBit(data, 5);
  data[0] |= static_cast<std::uint8_t>((configuration_.tecu_class - 1U) << 6U);
  if (configuration_.enable_power_management) {setBit(data, 3); setBit(data, 4);}
  setBit(data, 10);
  if (configuration_.enable_rear_pto) {setBit(data, 12); setBit(data, 13);}
  if (configuration_.enable_rear_hitch) {setBit(data, 14); setBit(data, 15);}
  if (configuration_.tecu_class >= 2U) {
    if (valve_count_ > 0U) setBit(data, 16);
    if (configuration_.enable_lighting) setBit(data, 17);
    if (configuration_.enable_rear_hitch) setBit(data, 18);
    setBit(data, 19); setBit(data, 20); setBit(data, 21); setBit(data, 22); setBit(data, 23);
  }
  if (configuration_.tecu_class == 3U) {
    if (configuration_.enable_rear_hitch || configuration_.enable_rear_pto || valve_count_ > 0U) {
      setBit(data, 27);
    }
    if (valve_count_ > 0U) setBit(data, 28);
    if (configuration_.enable_rear_pto) {setBit(data, 29); setBit(data, 30);}
    if (configuration_.enable_rear_hitch) setBit(data, 31);
    if (configuration_.enable_guidance) setBit(data, 49);
  }
  sendFrame(pdu2(0xFE09U, data));
  sendLanguage();
}

void TECUClass3Server::sendPeriodicStatus()
{
  // Class 1 publishes speed but marks distance and direction unavailable.
  Payload speed; speed.fill(0xFFU);
  if (twist_valid_) putU16(speed, 0, speedRaw(speed_m_s_));
  if (configuration_.tecu_class >= 2U && twist_valid_) {
    putU32(speed, 2, static_cast<std::uint32_t>(
      std::fmod(distance_m_ * 1000.0, 4211081216.0)));
  }
  speed[6] = configuration_.enable_power_management && maximum_power_time_valid_ ?
    maximum_power_time_min_ : 0xFFU;
  const std::uint8_t direction = configuration_.tecu_class >= 2U && twist_valid_ ?
    (speed_m_s_ < 0.0 ? 0U : 1U) : 3U;
  const std::uint8_t key_switch = key_switch_valid_ ? (key_switch_active_ ? 1U : 0U) : 3U;
  speed[7] = static_cast<std::uint8_t>(0xF0U | direction | key_switch << 2U);
  sendFrame(pdu2(0xFE48U, speed));
  speed[6] = 0xFFU;
  speed[7] = static_cast<std::uint8_t>(0xFCU | direction);
  sendFrame(pdu2(0xFE49U, speed));

  Payload engine; engine.fill(0xFFU);
  if (engine_speed_valid_) {
    putU16(engine, 3, static_cast<std::uint16_t>(std::clamp(
      std::lround(engine_speed_rpm_ * 8.0), 0L, 64255L)));
  }
  sendFrame(pdu2(0xF004U, engine));

  if (configuration_.enable_rear_hitch) {
    Payload hitch;
    hitch.fill(0xFFU);
    if (hitch_valid_) {
      hitch[0] = static_cast<std::uint8_t>(std::clamp(
        std::lround(hitch_status_.position_percent * 2.5), 0L, 250L));
      hitch[1] = static_cast<std::uint8_t>((hitch_status_.in_work & 3U) << 6U |
        (hitch_status_.position_limit_status & 7U) << 3U | 0x07U);
      if (std::isfinite(hitch_status_.nominal_lower_link_force)) {
        hitch[2] = static_cast<std::uint8_t>(std::clamp(
          std::lround((hitch_status_.nominal_lower_link_force + 100.0) / 0.8), 0L, 250L));
      }
      if (std::isfinite(hitch_status_.draft_n)) {
        putU16(hitch, 3, static_cast<std::uint16_t>(std::clamp(
          std::lround((hitch_status_.draft_n + 320000.0) / 10.0), 0L, 65534L)));
      }
      hitch[6] = static_cast<std::uint8_t>(
        (hitch_status_.exit_code & 0x3FU) << 2U | 0x03U);
    }
    sendFrame(pdu2(0xFE45U, hitch));
  }

  if (configuration_.enable_rear_pto) {
    Payload pto;
    pto.fill(0xFFU);
    if (pto_valid_) {
      if (std::isfinite(pto_status_.rpm)) {
        putU16(pto, 0, static_cast<std::uint16_t>(std::clamp(
          std::lround(pto_status_.rpm * 8.0), 0L, 65534L)));
      }
      if (std::isfinite(pto_status_.setpoint_rpm)) {
        putU16(pto, 2, static_cast<std::uint16_t>(std::clamp(
          std::lround(pto_status_.setpoint_rpm * 8.0), 0L, 65534L)));
      }
      pto[4] = static_cast<std::uint8_t>(
        (pto_status_.engagement & 3U) << 6U |
        (pto_status_.mode & 3U) << 4U |
        (pto_status_.economy_mode & 3U) << 2U |
        (pto_status_.engagement_request & 3U));
      pto[5] = static_cast<std::uint8_t>(
        (pto_status_.mode_request & 3U) << 6U |
        (pto_status_.economy_request & 3U) << 4U |
        (pto_status_.speed_limit_status & 7U) << 1U | 1U);
      pto[6] = 0U;
    }
    sendFrame(pdu2(0xFE43U, pto));
  }

  if (configuration_.tecu_class == 3U && configuration_.enable_guidance) {
    Payload guidance;
    guidance.fill(0xFFU);
    if (twist_valid_) putU16(guidance, 0, curvatureRaw(curvature_m_inv_));
    if (guidance_available_valid_) {
      guidance[2] = static_cast<std::uint8_t>((mechanical_lockout_ ? 1U : 0U) |
        (guidance_ready_ ? 0U : 1U) << 2U | 0xF0U);
      guidance[3] = 0U;
      guidance[4] = 0U;
    }
    sendFrame(pdu1(0xACU, guidance_destination_, guidance));
  }

  if (configuration_.tecu_class == 3U && configuration_.enable_cruise) {
    Payload cruise;
    cruise.fill(0xFFU);
    cruise[0] = 0x0AU;
    if (twist_valid_) putU16(cruise, 2, speedRaw(speed_m_s_));
    sendFrame(pdu2(0xFE0AU, cruise));
  }

  for (std::uint8_t valve = 0U; valve < valve_count_; ++valve) {
    const auto & status = valve_status_[valve];
    Payload flow; flow.fill(0xFFU);
    if (valve_valid_[valve]) {
      flow[0] = static_cast<std::uint8_t>(std::clamp(
        std::lround(status.extend_flow_percent + 125.0F), 0L, 250L));
      flow[1] = static_cast<std::uint8_t>(std::clamp(
        std::lround(status.retract_flow_percent + 125.0F), 0L, 250L));
      flow[2] = static_cast<std::uint8_t>(
        (status.failsafe ? 1U : 0U) << 6U | (status.state & 0x0FU));
    }
    sendFrame(pdu2(0xFE10U | valve, flow));
  }
}

void TECUClass3Server::sendLanguage()
{
  Payload language;
  language.fill(0xFFU);
  language[0] = 'e'; language[1] = 'n';
  language[2] = 0U;  // 24 h clock and decimal point.
  language[3] = 1U;  // Day-month-year.
  language[4] = 0U; language[5] = 0U;
  language[6] = 'F'; language[7] = 'I';
  sendFrame(pdu2(0xFE0FU, language));
}

void TECUClass3Server::sendTimeDate()
{
  const std::time_t current = std::time(nullptr);
  std::tm utc{};
  gmtime_r(&current, &utc);
  Payload data;
  data.fill(0xFFU);
  data[0] = static_cast<std::uint8_t>(utc.tm_sec * 4);
  data[1] = static_cast<std::uint8_t>(utc.tm_min);
  data[2] = static_cast<std::uint8_t>(utc.tm_hour);
  data[3] = static_cast<std::uint8_t>(utc.tm_mon + 1);
  data[4] = static_cast<std::uint8_t>(utc.tm_mday * 4);
  data[5] = static_cast<std::uint8_t>(utc.tm_year + 1900 - 1985);
  data[6] = 125U; data[7] = 125U;
  sendFrame(pdu2(0xFEE6U, data));
}

void TECUClass3Server::sendNegativeAcknowledgement(const msg::IsobusFrame & request)
{
  // SAE J1939 acknowledgement PGN 59392, control byte 1 = negative ACK.
  Payload data;
  data.fill(0xFFU);
  data[0] = 1U;  // J1939 negative acknowledgement.
  const std::uint32_t requested_pgn = request.pf < 240U ?
    static_cast<std::uint32_t>(request.pf) << 8U : request.pgn;
  data[5] = static_cast<std::uint8_t>(requested_pgn);
  data[6] = static_cast<std::uint8_t>(requested_pgn >> 8U);
  data[7] = static_cast<std::uint8_t>(requested_pgn >> 16U);
  sendFrame(pdu1(0xE8U, request.sa, data));
}

msg::IsobusFrame TECUClass3Server::pdu2(std::uint32_t pgn, const Payload & data) const
{
  msg::IsobusFrame frame;
  frame.priority = 3U; frame.page = false; frame.pgn = pgn; frame.sa = source_address_;
  frame.pf = static_cast<std::uint8_t>(pgn >> 8U); frame.ps = static_cast<std::uint8_t>(pgn); frame.dlc = 8U; frame.data = data;
  return frame;
}

msg::IsobusFrame TECUClass3Server::pdu1(std::uint8_t pf, std::uint8_t destination, const Payload & data) const
{
  msg::IsobusFrame frame;
  frame.priority = 3U; frame.page = false; frame.pgn = static_cast<std::uint32_t>(pf) << 8U | destination;
  frame.sa = source_address_; frame.pf = pf; frame.ps = destination; frame.dlc = 8U; frame.data = data;
  return frame;
}

}  // namespace ros2_isobus
