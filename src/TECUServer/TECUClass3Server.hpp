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

#include <array>
#include <cstdint>
#include <string>

#include "ros2_isobus/msg/aux_valve_command.hpp"
#include "ros2_isobus/msg/aux_valve_status.hpp"
#include "ros2_isobus/msg/isobus_frame.hpp"
#include "ros2_isobus/msg/tecu_cruise_command.hpp"
#include "ros2_isobus/msg/tecu_guidance_command.hpp"
#include "ros2_isobus/msg/tecu_rear_hitch_command.hpp"
#include "ros2_isobus/msg/tecu_rear_hitch_status.hpp"
#include "ros2_isobus/msg/tecu_rear_pto_command.hpp"
#include "ros2_isobus/msg/tecu_rear_pto_status.hpp"

namespace ros2_isobus
{

/*
 *
 * TECUClass3Server implements the tractor side of ISO 11783-7/9:
 *  - Class 1 publishes the basic tractor facilities and measurements
 *  - Class 2 adds distance, direction, time, lighting and AUX valve status
 *  - Class 3 adds configured hitch, PTO, AUX, guidance and cruise commands
 *
 * The historical class name denotes the complete Class 3 superset. The
 * tecu_class configuration selects the advertised and active subset.
 *
 */
class TECUClass3Server
{
public:
  // Server class and installed tractor capabilities.
  struct Configuration
  {
    std::uint8_t tecu_class{3};
    bool enable_guidance{true};
    bool enable_cruise{true};
    bool enable_rear_hitch{true};
    bool enable_rear_pto{true};
    bool enable_lighting{true};
    bool enable_power_management{true};
    std::uint8_t aux_valve_count{8};
  };

  TECUClass3Server();
  virtual ~TECUClass3Server() = default;

  // Configuration, address ownership and physical tractor state.
  void setSourceAddress(std::uint8_t address);
  void setAddressClaimed(bool claimed);
  void setKeySwitch(bool active);
  void setEngineSpeed(double rpm);
  void setMaximumPowerTime(std::uint8_t minutes);
  void setGuidanceAvailable(bool ready, bool lockout);
  void setInputTimeoutTicks(unsigned ticks);
  void setValveCount(std::uint8_t count);
  void setTwist(double speed_m_s, double yaw_rate_rad_s, double elapsed_s);
  void setRearHitchStatus(const msg::TecuRearHitchStatus & status);
  void setRearPtoStatus(const msg::TecuRearPtoStatus & status);
  void setAuxValveStatus(const msg::AuxValveStatus & status);
  void setConfiguration(const Configuration & configuration);

  // Core processing. run() is called at 100 ms intervals by the ROS wrapper.
  void HandleMsg(const msg::IsobusFrame & frame);
  void run();

protected:
  // Transport hooks implemented by the ROS 2 wrapper.
  virtual void sendFrame(const msg::IsobusFrame & frame) = 0;
  virtual void publishCruiseCommand(const msg::TecuCruiseCommand & command) = 0;
  virtual void publishGuidanceCommand(const msg::TecuGuidanceCommand & command) = 0;
  virtual void publishRearHitchCommand(const msg::TecuRearHitchCommand & command) = 0;
  virtual void publishRearPtoCommand(const msg::TecuRearPtoCommand & command) = 0;
  virtual void publishAuxValveCommand(const msg::AuxValveCommand & command) = 0;
  virtual void publishTwistCommand(double speed_m_s, double yaw_rate_rad_s) = 0;
  virtual void printWarn(const std::string & text) = 0;

private:
  using Payload = std::array<std::uint8_t, 8>;

  // Class 3 command decoders.
  void decodeCruiseCommand(const msg::IsobusFrame & frame);
  void decodeGuidanceCommand(const msg::IsobusFrame & frame);
  void decodeHitchPtoCommand(const msg::IsobusFrame & frame);
  void decodeValveCommand(const msg::IsobusFrame & frame);

  // Class 1/2/3 status and protocol response builders.
  void sendFacilitiesResponse();
  void sendLanguage();
  void sendTimeDate();
  void sendPeriodicStatus();
  void sendNegativeAcknowledgement(const msg::IsobusFrame & request);
  msg::IsobusFrame pdu2(std::uint32_t pgn, const Payload & data) const;
  msg::IsobusFrame pdu1(std::uint8_t pf, std::uint8_t destination, const Payload & data) const;

  // Physical tractor status supplied by ROS input topics.
  std::array<msg::AuxValveStatus, 16> valve_status_;
  msg::TecuRearHitchStatus hitch_status_{};
  msg::TecuRearPtoStatus pto_status_{};
  std::uint8_t source_address_{0xFE};
  std::uint8_t guidance_destination_{0xFF};
  std::uint8_t valve_count_{8};
  Configuration configuration_{};

  // Measured and commanded vehicle motion.
  double speed_m_s_{0.0};
  double curvature_m_inv_{0.0};
  double distance_m_{0.0};
  double engine_speed_rpm_{0.0};
  std::uint8_t maximum_power_time_min_{0U};
  double commanded_speed_m_s_{0.0};
  double commanded_curvature_m_inv_{0.0};

  // Address, safety and command-timeout state.
  bool address_claimed_{false};
  bool key_switch_active_{false};
  bool guidance_ready_{false};
  bool mechanical_lockout_{false};
  bool twist_valid_{false};
  bool hitch_valid_{false};
  bool pto_valid_{false};
  bool engine_speed_valid_{false};
  bool maximum_power_time_valid_{false};
  bool key_switch_valid_{false};
  bool guidance_available_valid_{false};
  std::array<bool, 16> valve_valid_{};
  unsigned input_timeout_ticks_{10U};
  unsigned twist_age_{0U};
  unsigned hitch_age_{0U};
  unsigned pto_age_{0U};
  unsigned engine_speed_age_{0U};
  unsigned maximum_power_time_age_{0U};
  unsigned key_switch_age_{0U};
  unsigned guidance_available_age_{0U};
  std::array<unsigned, 16> valve_age_{};
  int cruise_command_age_{-1};
  int guidance_command_age_{-1};
  unsigned periodic_counter_{0};
};

}  // namespace ros2_isobus
