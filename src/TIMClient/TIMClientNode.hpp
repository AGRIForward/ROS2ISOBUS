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

#include "TIMClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_isobus/msg/isobus_address_book.hpp"
#include "ros2_isobus/msg/isobus_address_status.hpp"
#include "ros2_isobus/msg/isobus_frame.hpp"
#include "ros2_isobus/msg/isobus_tp_frame.hpp"
#include "ros2_isobus/msg/tim_aux_valve_command.hpp"
#include "ros2_isobus/msg/tim_aux_valve_status.hpp"
#include "ros2_isobus/msg/tim_cruise_command.hpp"
#include "ros2_isobus/msg/tim_cruise_status.hpp"
#include "ros2_isobus/msg/tim_curvature_command.hpp"
#include "ros2_isobus/msg/tim_curvature_status.hpp"
#include "ros2_isobus/msg/tim_rear_hitch_command.hpp"
#include "ros2_isobus/msg/tim_rear_hitch_status.hpp"
#include "ros2_isobus/msg/tim_rear_pto_command.hpp"
#include "ros2_isobus/msg/tim_rear_pto_status.hpp"
#include "ros2_isobus/topics.hpp"

namespace ros2_isobus
{

/*
 * TIMClientROS2 binds TimClient workflow to ROS 2 transport and command topics.
 *
 * Reference mapping:
 *  - AEF 023 RIG 4: TIM initialization + status + function control workflow
 *    (notably chapters 6.x, 8.x, 9.x and Annex D / 10.x)
 *  - ISO 11783-3: single-frame vs TP carriage for ISOBUS traffic
 *  - ISO 11783-5: SA/NAME tracking via Address Claim ecosystem
 */
class TIMClientROS2 : public rclcpp::Node, public TimClient
{
public:
    explicit TIMClientROS2(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // Build TIM function map from ROS parameters (AEF 023 RIG 4 function/control mode mapping).
    static FunctionConfig load_function_config(rclcpp::Node & node);
    // ISO 11783/J1939 node SA domain is 0x00..0xFD.
    static bool is_valid_node_sa(std::uint8_t sa);
    // AddressManager publishes NAME as little-endian byte array.
    static std::uint64_t name_to_u64_le(const std::array<std::uint8_t, 8> & name);
    // Learn TIM server SA from broadcast TIM_ServerStatus before explicit parameter/static SA is available.
    void try_detect_server_sa_from_broadcast(const msg::IsobusFrame & fr);
    // Track TIM client/server NAME<->SA changes from AddressManager address book.
    void update_names_from_address_book(const msg::IsobusAddressBook::SharedPtr & msg);

    // TimClient platform hook implementations.
    std::uint32_t nowMs() const override;
    void sendFrame(const msg::IsobusFrame & fr) override;
    void sendTpFrame(const msg::IsobusTpFrame & tp) override;
    void logInfo(const std::string & msg) override;
    void logWarn(const std::string & msg) override;
    void logError(const std::string & msg) override;
    void publish_tim_statuses();
    void refresh_operator_enable();
    void apply_command_timeouts();

    // ROS interfaces for bus ingress/egress and periodic TimClient::process() tick.
    rclcpp::Subscription<msg::IsobusFrame>::SharedPtr bus_sub_;
    rclcpp::Subscription<msg::IsobusTpFrame>::SharedPtr bus_tp_sub_;
    rclcpp::Subscription<msg::IsobusAddressBook>::SharedPtr address_book_sub_;
    rclcpp::Subscription<msg::IsobusAddressStatus>::SharedPtr address_status_sub_;
    rclcpp::Subscription<msg::TimCruiseCommand>::SharedPtr cruise_cmd_sub_;
    rclcpp::Subscription<msg::TimCurvatureCommand>::SharedPtr curvature_cmd_sub_;
    rclcpp::Subscription<msg::TimRearHitchCommand>::SharedPtr rear_hitch_cmd_sub_;
    rclcpp::Subscription<msg::TimRearPtoCommand>::SharedPtr rear_pto_cmd_sub_;
    rclcpp::Subscription<msg::TimAuxValveCommand>::SharedPtr aux_valve_cmd_sub_;
    rclcpp::Publisher<msg::IsobusFrame>::SharedPtr tx_pub_;
    rclcpp::Publisher<msg::IsobusTpFrame>::SharedPtr tx_tp_pub_;
    rclcpp::Publisher<msg::TimCruiseStatus>::SharedPtr cruise_status_pub_;
    rclcpp::Publisher<msg::TimCurvatureStatus>::SharedPtr curvature_status_pub_;
    rclcpp::Publisher<msg::TimRearHitchStatus>::SharedPtr rear_hitch_status_pub_;
    rclcpp::Publisher<msg::TimRearPtoStatus>::SharedPtr rear_pto_status_pub_;
    rclcpp::Publisher<msg::TimAuxValveStatus>::SharedPtr aux_valve_status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Command caches mirror latest ROS command topics and support timeout-driven safe disable.
    // Timeout fallback aligns with "lost command input" safety intent in TIM automation operation.
    std::array<bool, MAX_AUX> aux_enable_cache_{};
    std::array<float, MAX_AUX> aux_flow_cache_{};
    std::array<std::uint32_t, MAX_AUX> aux_last_cmd_ms_{};
    std::array<bool, MAX_AUX> aux_cmd_seen_{};
    bool speed_enable_cache_ = false;
    bool curvature_enable_cache_ = false;
    bool rear_hitch_enable_cache_ = false;
    bool rear_pto_enable_cache_ = false;
    float commanded_speed_mps_ = 0.0f;
    float commanded_curvature_km_inv_ = 0.0f;
    float commanded_rear_hitch_pct_ = 0.0f;
    float commanded_rear_pto_rpm_ = 0.0f;
    std::uint32_t speed_last_cmd_ms_ = 0;
    std::uint32_t curvature_last_cmd_ms_ = 0;
    std::uint32_t rear_hitch_last_cmd_ms_ = 0;
    std::uint32_t rear_pto_last_cmd_ms_ = 0;
    bool speed_cmd_seen_ = false;
    bool curvature_cmd_seen_ = false;
    bool rear_hitch_cmd_seen_ = false;
    bool rear_pto_cmd_seen_ = false;
    std::uint32_t command_timeout_ms_ = 250;
    bool client_sa_ready_ = false;
    bool client_name_ready_ = false;
    std::uint64_t client_name_cached_ = 0;
    bool server_sa_ready_ = false;
    bool server_name_ready_ = false;
    std::uint64_t server_name_cached_ = 0;
};

}  // namespace ros2_isobus
