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

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "ros2_isobus/msg/isobus_address_book.hpp"
#include "ros2_isobus/msg/isobus_address_status.hpp"
#include "ros2_isobus/msg/command_result.hpp"
#include "ros2_isobus/msg/isobus_frame.hpp"
#include "ros2_isobus/msg/isobus_tp_frame.hpp"
#include "ros2_isobus/msg/isobus_tp_tx_status.hpp"
#include "ros2_isobus/msg/vt_navigation_event.hpp"
#include "ros2_isobus/msg/vt_pointing_event.hpp"
#include "ros2_isobus/msg/vt_session_state.hpp"
#include "ros2_isobus/msg/vt_status.hpp"
#include "ros2_isobus/msg/vt_update_result.hpp"
#include "ros2_isobus/msg/vt_aux_input_raw.hpp"
#include "ros2_isobus/msg/vt_aux_assignment.hpp"
#include "ros2_isobus/msg/vt_aux_status.hpp"
#include "ros2_isobus/topics.hpp"

#include "VTClient.hpp"

namespace ros2_isobus
{

/*
 *
 * VTClientNode binds VTClient protocol logic to ROS 2:
 *  - Subscribes/publishes TP traffic via CanBridge topics
 *  - Tracks local SA and VT SA via AddressManager status/address-book (ISO 11783-5)
 *  - Defers XML->POOL build until VT version response to allow VT profile override
 *  - Creates dynamic button/numeric topics based on object names in XML
 *
 */
class VTClientNode : public rclcpp::Node, public VTClient
{
public:
  // Construct node, declare params, wire pub/sub and session timer.
  explicit VTClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  // VTClient transport hooks.
  void send_frame(const msg::IsobusFrame & frame) override;
  void send_tp_frame(const msg::IsobusTpFrame & tp) override;
  // VTClient event/result hooks.
  void on_button_event(const ObjectBinding & binding, bool pressed, std::uint8_t activation_code) override;
  void on_softkey_event(const ObjectBinding & binding, bool pressed, std::uint8_t activation_code) override;
  void on_numeric_event(const ObjectBinding & binding, std::int32_t value) override;
  void on_string_event(const ObjectBinding & binding, const std::string & value) override;
  void on_list_event(const ObjectBinding & binding, std::int32_t index) override;
  void on_vt_status(const VtStatus & status) override;
  void on_pointing_event(const PointingEvent & ev) override;
  void on_navigation_event(const NavigationEvent & ev) override;
  void on_aux_input_status(
    const ObjectBinding & binding, const VtAuxInputStatus & status, double normalized_value) override;
  void on_aux_assignment_value(const ObjectBinding & binding, const VtAuxAssignmentCommand & assignment) override;
  void on_aux_assignment_result(
    const ObjectBinding & binding, bool success, std::uint8_t error_code_bits,
    const std::string & detail) override;
  bool validate_aux_input_name(std::uint64_t aux_input_name, std::uint8_t * resolved_sa) override;
  bool validate_aux_input_source(std::uint8_t src_sa, std::uint64_t expected_aux_input_name) override;
  bool resolve_name_from_sa(std::uint8_t sa, std::uint64_t * name_out) override;
  void on_preferred_assignments_changed(const std::vector<PreferredAssignmentEntry> & entries) override;
  void on_change_active_mask_result(std::uint16_t mask_id, std::uint8_t error_code) override;
  void on_change_soft_key_mask_result(std::uint16_t mask_id, std::uint8_t error_code) override;
  void on_change_string_value_result(std::uint16_t object_id, std::uint8_t error_code) override;
  void on_change_list_item_result(std::uint16_t object_id, std::uint8_t list_index, std::uint8_t error_code) override;
  void on_hide_show_object_result(std::uint16_t object_id, std::uint8_t error_code) override;
  void on_hide_show_object_state(std::uint16_t object_id, bool visible) override;
  void on_enable_disable_object_result(std::uint16_t object_id, std::uint8_t error_code) override;
  void on_select_input_object_result(std::uint16_t object_id, std::uint8_t error_code) override;
  void on_pool_ready(const std::string & xml_path, const BuildConfig & used_cfg, bool vt_reported) override;
  void on_pool_build_failed(const std::string & xml_path) override;
  void on_runtime_update_result(
    bool success, const std::string & detail, std::uint32_t pending_updates,
    std::uint32_t in_progress_updates) override;
  // VTClient logging hooks.
  void printInfo(const std::string & msg) override;
  void printWarn(const std::string & msg) override;

private:
  // Per-object-number ROS I/O bundle.
  struct NumericIo
  {
    std::uint16_t object_id{0};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr value_pub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr set_sub;
  };
  // Per-object-string ROS I/O bundle.
  struct StringIo
  {
    std::uint16_t object_id{0};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr value_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr set_sub;
  };
  // Per-object-list ROS I/O bundle.
  struct ListIo
  {
    std::uint16_t object_id{0};
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr value_pub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr set_sub;
  };
  struct BoolIo
  {
    std::uint16_t object_id{0};
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr value_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_sub;
  };
  struct EnabledIo
  {
    std::uint16_t object_id{0};
    std::string token;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr value_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_sub;
    rclcpp::Publisher<msg::CommandResult>::SharedPtr result_pub;
  };
  // Generic visible attribute I/O bundle.
  struct ContainerVisibleIo
  {
    std::uint16_t object_id{0};
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr value_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_sub;
    rclcpp::Publisher<msg::CommandResult>::SharedPtr result_pub;
  };
  // Active/softkey mask control topic bundle.
  struct MaskIo
  {
    std::uint16_t object_id{0};
    std::string token;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr value_pub;
    rclcpp::Publisher<msg::CommandResult>::SharedPtr result_pub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr set_sub;
  };
  struct AuxFunctionIo
  {
    std::uint16_t function_object_id{0};
    std::uint8_t type_id{0xFF};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr input_value_pub;
    rclcpp::Publisher<msg::VTAuxInputRaw>::SharedPtr input_raw_pub;
    rclcpp::Publisher<msg::VTAuxAssignment>::SharedPtr assignment_value_pub;
    rclcpp::Publisher<msg::CommandResult>::SharedPtr assignment_result_pub;
  };

  // Build dynamic VT topics from parsed XML bindings.
  void create_dynamic_topics();

  // Bus/address callbacks.
  void on_address_status(const msg::IsobusAddressStatus & status);
  void on_address_book(const msg::IsobusAddressBook & book);
  void on_bus_frame(const msg::IsobusFrame & frame);
  void on_bus_tp(const msg::IsobusTpFrame & tp);
  void on_bus_tp_tx_status(const msg::IsobusTpTxStatus & st);
  void on_vt_update(const std_msgs::msg::String & update);
  void on_acknowledgement(const msg::IsobusFrame & frame);
  // Periodic session tick handler.
  void on_session_tick();
  // Track VT SA candidates seen from VT status source address.
  void observe_vt_status_source(std::uint8_t sa);
  // Utility helpers.
  static std::uint64_t now_ms();
  static bool is_valid_sa(std::uint8_t sa);
  static std::uint64_t name_from_bytes_be(const std::array<std::uint8_t, 8> & name);
  static std::uint8_t name_function_id(std::uint64_t name);
  void load_aux_preferred_assignments_from_file();
  void save_aux_preferred_assignments_to_file(const std::vector<PreferredAssignmentEntry> & entries);

  // Bus-facing publishers/subscribers.
  rclcpp::Publisher<msg::IsobusFrame>::SharedPtr tx_pub_;
  rclcpp::Subscription<msg::IsobusFrame>::SharedPtr rx_sub_;
  rclcpp::Publisher<msg::IsobusTpFrame>::SharedPtr tx_tp_pub_;
  rclcpp::Subscription<msg::IsobusTpFrame>::SharedPtr rx_tp_sub_;
  rclcpp::Subscription<msg::IsobusTpTxStatus>::SharedPtr tp_tx_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vt_update_sub_;
  rclcpp::Subscription<msg::IsobusAddressStatus>::SharedPtr addr_status_sub_;
  rclcpp::Subscription<msg::IsobusAddressBook>::SharedPtr addr_book_sub_;
  rclcpp::TimerBase::SharedPtr session_timer_;

  // Dynamic event/value topic registries keyed by topic token.
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr> button_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr> softkey_pubs_;
  std::unordered_map<std::string, NumericIo> numeric_ios_;
  std::unordered_map<std::string, StringIo> string_ios_;
  std::unordered_map<std::string, ListIo> list_ios_;
  std::unordered_map<std::string, BoolIo> bool_ios_;
  std::unordered_map<std::string, EnabledIo> enabled_ios_;
  std::unordered_map<std::uint16_t, std::string> enabled_by_id_;
  std::unordered_map<std::string, ContainerVisibleIo> container_visible_ios_;
  std::unordered_map<std::string, MaskIo> active_mask_ios_;
  std::unordered_map<std::string, MaskIo> softkey_mask_ios_;
  std::unordered_map<std::uint16_t, std::string> active_mask_by_id_;
  std::unordered_map<std::uint16_t, std::string> softkey_mask_by_id_;
  std::unordered_map<std::string, AuxFunctionIo> aux_function_ios_;
  std::unordered_map<std::uint16_t, std::string> aux_function_by_id_;
  std::unordered_map<std::uint64_t, std::uint8_t> name_to_sa_;
  std::unordered_map<std::uint8_t, std::uint64_t> sa_to_name_;
  rclcpp::Publisher<msg::VTStatus>::SharedPtr vt_status_pub_;
  rclcpp::Publisher<msg::VTUpdateResult>::SharedPtr vt_update_result_pub_;
  rclcpp::Publisher<msg::VTSessionState>::SharedPtr vt_session_state_pub_;
  rclcpp::Publisher<msg::VTAuxStatus>::SharedPtr vt_aux_status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr vt_diag_pub_;
  rclcpp::Publisher<msg::VTPointingEvent>::SharedPtr vt_pointing_pub_;
  rclcpp::Publisher<msg::VTNavigationEvent>::SharedPtr vt_navigation_pub_;
  // Keep dynamic entities alive for node lifetime.
  std::vector<rclcpp::PublisherBase::SharedPtr> dynamic_publishers_keepalive_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> dynamic_subscriptions_keepalive_;

  // Protocol/session runtime flags and parameters.
  std::uint32_t pgn_vt_to_node_{0x00E600};
  bool session_started_{false};
  bool vt_sa_locked_from_address_book_{false};
  bool vt_seen_in_address_book_{false};
  bool vt_wait_address_claim_{false};
  bool vt_aux_n_support_{false};
  bool vt_aux_preferred_store_override_{false};
  std::string vt_aux_preferred_assignment_file_{};
  std::uint8_t aid_width_{1};
  std::uint8_t aid_height_{2};
  bool local_sa_valid_{false};
  bool vt_present_{false};
  bool vt_status_seen_{false};
  bool topics_created_{false};
  std::string xml_file_path_;
  std::vector<std::uint8_t> vt_candidates_;
  std::unordered_set<std::uint8_t> vt_candidate_set_;
  std::size_t vt_candidate_index_{0};
  SessionState last_session_state_{SessionState::Idle};
  // Working Set maintenance transmission pacing.
  std::uint32_t ws_maintenance_period_ms_{1000};
  std::uint64_t last_ws_maintenance_tx_ms_{0};
  // Working Set definition (PGN 65037/65036) is sent on change, not cyclically.
  bool ws_definition_sent_{false};
  std::uint8_t ws_definition_sa_{0xFE};
  // VT status watchdog (Active state): detect VT disappearing from bus.
  std::uint32_t vt_status_timeout_ms_{3000};
  std::uint64_t last_vt_status_rx_ms_{0};
  bool vt_status_timeout_reported_{false};
  // Cache latest published VT status for timeout event publication.
  msg::VTStatus last_vt_status_msg_{};
  bool has_last_vt_status_msg_{false};
};

}  // namespace ros2_isobus
