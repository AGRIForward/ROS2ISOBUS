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

// Entry point for ISO 11783-6 Virtual Terminal client node.
#include "VTClientNode.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <algorithm>
#include <filesystem>
#include <exception>
#include <cctype>
#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ros2_isobus
{
namespace
{
constexpr std::uint32_t kPgnAcknowledgement = 0x00E800;  // 59392
constexpr std::uint32_t kPgnVtToWs = 0x00E600;           // 58880
constexpr std::uint32_t kPgnWsToVt = 0x00E700;           // 59136
constexpr std::uint32_t kPgnWorkingSetMaster = 0x00FE0D; // 65037
constexpr std::uint32_t kPgnWorkingSetMember = 0x00FE0C; // 65036
constexpr std::uint8_t kAddressStateClaimed = 4;         // AddressManager::NameState::ADDRESS_CLAIMED

}  // namespace

VTClientNode::VTClientNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vt_client_node", options),
  VTClient(
    static_cast<std::uint8_t>(declare_parameter<int>("sa_local", 0xFE)),
    static_cast<std::uint8_t>(declare_parameter<int>("sa_vt", 0x26)))
{
  std::string pkg_share;
  try {
    pkg_share = ament_index_cpp::get_package_share_directory("ros2_isobus");
  } catch (const std::exception &) {
    pkg_share.clear();
  }

  const std::string xml_default = pkg_share.empty() ?
    std::string("config/vt_examples/vtecu_startup_reconstructed.xml") :
    (pkg_share + "/config/vt_examples/vtecu_startup_reconstructed.xml");
  xml_file_path_ = declare_parameter<std::string>("xml_file", xml_default);

  if (!xml_file_path_.empty()) {
    std::filesystem::path xml_path(xml_file_path_);
    if (xml_path.is_relative() && !pkg_share.empty()) {
      xml_path = std::filesystem::path(pkg_share) / xml_path;
      xml_file_path_ = xml_path.lexically_normal().string();
    }
  }

  BuildConfig fallback_cfg;
  fallback_cfg.vt_dimension = declare_parameter<int>("vt_dimension", 200);
  fallback_cfg.vt_softkey_width = declare_parameter<int>("vt_softkey_width", 60);
  fallback_cfg.vt_softkey_height = declare_parameter<int>("vt_softkey_height", 32);
  fallback_cfg.vt_colors = declare_parameter<int>("vt_colors", 256);
  const auto use_vt_reported_profile = declare_parameter<bool>("vt_use_reported_display_profile", true);
  vt_aux_n_support_ = declare_parameter<bool>("vt_aux_n_support", false);
  set_aux_n_support(vt_aux_n_support_);
  vt_aux_preferred_store_override_ =
    declare_parameter<bool>("vt_aux_preferred_store_override", false);
  set_aux_preferred_store_override(vt_aux_preferred_store_override_);
  vt_aux_preferred_assignment_file_ =
    declare_parameter<std::string>("vt_aux_preferred_assignment_file", "");
  if (!vt_aux_preferred_assignment_file_.empty()) {
    std::filesystem::path pref_path(vt_aux_preferred_assignment_file_);
    if (pref_path.is_relative() && !pkg_share.empty()) {
      pref_path = std::filesystem::path(pkg_share) / pref_path;
      vt_aux_preferred_assignment_file_ = pref_path.lexically_normal().string();
    }
  }
  const auto session_timeout_ms =
    static_cast<std::uint32_t>(declare_parameter<int>("vt_session_timeout_ms", 1000));
  const auto session_retries =
    static_cast<std::uint32_t>(declare_parameter<int>("vt_session_retries", 3));
  vt_wait_address_claim_ = declare_parameter<bool>("vt_wait_address_claim", false);
  aid_width_ = 1;
  aid_height_ = 2;
  const auto session_tick_ms = declare_parameter<int>("vt_session_tick_ms", 100);
  ws_maintenance_period_ms_ = static_cast<std::uint32_t>(
    declare_parameter<int>("vt_ws_maintenance_period_ms", 1000));
  vt_status_timeout_ms_ = static_cast<std::uint32_t>(
    declare_parameter<int>("vt_status_timeout_ms", 3000));
  const auto working_set_id =
    static_cast<std::uint16_t>(declare_parameter<int>("vt_working_set_id", 0));
  const auto object_pool_id =
    static_cast<std::uint16_t>(declare_parameter<int>("vt_object_pool_id", 0));
  const auto ws_version_number =
    static_cast<std::uint8_t>(declare_parameter<int>("vt_working_set_version", 6));
  // Optional Working Set member list is intentionally disabled in constructor path
  // to avoid launch-time parameter null/unset exceptions from external overrides.
  const std::vector<std::string> ws_member_names_hex{};
  set_pool_identifiers(working_set_id, object_pool_id);
  set_working_set_maintenance_version(ws_version_number);
  configure_pool_source(xml_file_path_, fallback_cfg, use_vt_reported_profile);
  if (!ws_member_names_hex.empty()) {
    std::vector<std::uint64_t> member_names;
    member_names.reserve(ws_member_names_hex.size());
    for (const auto & s : ws_member_names_hex) {
      try {
        member_names.push_back(std::stoull(s, nullptr, 16));
      } catch (const std::exception &) {
        printWarn("Invalid vt_working_set_member_names_hex entry: " + s);
      }
    }
    set_working_set_member_names(member_names);
    printInfo(
      "Configured Working Set member NAME entries: " + std::to_string(member_names.size()));
  }
  set_session_timing(session_timeout_ms, session_retries);
  load_aux_preferred_assignments_from_file();

  tx_pub_ = create_publisher<msg::IsobusFrame>(kBusTxTopic, rclcpp::QoS(100));
  rx_sub_ = create_subscription<msg::IsobusFrame>(
    kBusRxTopic, rclcpp::QoS(100),
    std::bind(&VTClientNode::on_bus_frame, this, std::placeholders::_1));
  tx_tp_pub_ = create_publisher<msg::IsobusTpFrame>(kBusTxTpTopic, rclcpp::QoS(10));
  rx_tp_sub_ = create_subscription<msg::IsobusTpFrame>(
    kBusRxTpTopic, rclcpp::QoS(50),
    std::bind(&VTClientNode::on_bus_tp, this, std::placeholders::_1));
  tp_tx_status_sub_ = create_subscription<msg::IsobusTpTxStatus>(
    kBusTxTpStatusTopic, rclcpp::QoS(50),
    std::bind(&VTClientNode::on_bus_tp_tx_status, this, std::placeholders::_1));
  vt_status_pub_ = create_publisher<msg::VTStatus>("ISOBUS/vt/status", rclcpp::QoS(10));
  vt_update_result_pub_ = create_publisher<msg::VTUpdateResult>("ISOBUS/vt/update_result", rclcpp::QoS(10));
  vt_session_state_pub_ = create_publisher<msg::VTSessionState>("ISOBUS/vt/session/state", rclcpp::QoS(10));
  vt_aux_status_pub_ = create_publisher<msg::VTAuxStatus>("ISOBUS/vt/aux/status", rclcpp::QoS(10));
  vt_diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "ISOBUS/vt/diagnostics", rclcpp::QoS(10));
  vt_pointing_pub_ = create_publisher<msg::VTPointingEvent>("ISOBUS/vt/event/pointing", rclcpp::QoS(10));
  vt_navigation_pub_ = create_publisher<msg::VTNavigationEvent>("ISOBUS/vt/event/navigation", rclcpp::QoS(10));
  vt_update_sub_ = create_subscription<std_msgs::msg::String>(
    "ISOBUS/vt/update", rclcpp::QoS(10),
    std::bind(&VTClientNode::on_vt_update, this, std::placeholders::_1));
  const auto latched_reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  addr_status_sub_ = create_subscription<msg::IsobusAddressStatus>(
    kAddressManagerStatus, latched_reliable,
    std::bind(&VTClientNode::on_address_status, this, std::placeholders::_1));
  addr_book_sub_ = create_subscription<msg::IsobusAddressBook>(
    kAddressManagerAddressBook, latched_reliable,
    std::bind(&VTClientNode::on_address_book, this, std::placeholders::_1));

  if (xml_file_path_.empty()) {
    printWarn("Parameter 'xml_file' missing, VT pool not loaded");
    return;
  }
  // XML -> object pool build is intentionally deferred until we receive
  // VT version response. This allows runtime profile override (dimension,
  // softkey size, color depth) when VT reports those fields.
  printInfo("VT pool build deferred until VT version response");
  printInfo("Waiting valid SA from AddressManager before VT session start");
  session_timer_ = create_wall_timer(
    std::chrono::milliseconds(session_tick_ms),
    std::bind(&VTClientNode::on_session_tick, this));
}

void VTClientNode::send_frame(const msg::IsobusFrame & frame)
{
  if (tx_pub_) {
    tx_pub_->publish(frame);
  }
}

void VTClientNode::send_tp_frame(const msg::IsobusTpFrame & tp)
{
  if (tx_tp_pub_) {
    tx_tp_pub_->publish(tp);
  }
}

void VTClientNode::on_button_event(
  const ObjectBinding & binding, bool pressed, std::uint8_t activation_code)
{
  const auto it = button_pubs_.find(binding.topic_token);
  if (it == button_pubs_.end()) {
    return;
  }
  std_msgs::msg::UInt8 msg;
  msg.data = activation_code;
  it->second->publish(msg);
  if (vt_navigation_pub_) {
    msg::VTNavigationEvent ev;
    ev.function_code = 0x01;
    ev.object_id = binding.object_id;
    ev.pressed = pressed;
    vt_navigation_pub_->publish(ev);
  }
}

void VTClientNode::on_softkey_event(
  const ObjectBinding & binding, bool pressed, std::uint8_t activation_code)
{
  const auto it = softkey_pubs_.find(binding.topic_token);
  if (it == softkey_pubs_.end()) {
    return;
  }
  std_msgs::msg::UInt8 msg;
  msg.data = activation_code;
  it->second->publish(msg);
  if (vt_navigation_pub_) {
    msg::VTNavigationEvent ev;
    ev.function_code = 0x00;
    ev.object_id = binding.object_id;
    ev.pressed = pressed;
    vt_navigation_pub_->publish(ev);
  }
}

void VTClientNode::on_numeric_event(const ObjectBinding & binding, std::int32_t value)
{
  bool published = false;

  const auto it = numeric_ios_.find(binding.topic_token);
  if (it != numeric_ios_.end() && it->second.value_pub) {
    std_msgs::msg::Float64 out;
    out.data = static_cast<double>(value);
    it->second.value_pub->publish(out);
    published = true;
  }

  const auto bit = bool_ios_.find(binding.topic_token);
  if (bit != bool_ios_.end() && bit->second.value_pub) {
    std_msgs::msg::Bool b;
    b.data = (value != 0);
    bit->second.value_pub->publish(b);
    published = true;
  }

  if (!published) {
    RCLCPP_DEBUG(
      get_logger(), "VT numeric event dropped: token=%s object_id=0x%04X",
      binding.topic_token.c_str(), binding.object_id);
  }
}

void VTClientNode::on_string_event(const ObjectBinding & binding, const std::string & value)
{
  const auto it = string_ios_.find(binding.topic_token);
  if (it == string_ios_.end() || !it->second.value_pub) {
    return;
  }
  std_msgs::msg::String out;
  out.data = value;
  it->second.value_pub->publish(out);
}

void VTClientNode::on_list_event(const ObjectBinding & binding, std::int32_t index)
{
  const auto it = list_ios_.find(binding.topic_token);
  if (it == list_ios_.end() || !it->second.value_pub) {
    return;
  }
  std_msgs::msg::Int32 out;
  out.data = index;
  it->second.value_pub->publish(out);
}

void VTClientNode::on_vt_status(const VtStatus & status)
{
  last_vt_status_rx_ms_ = now_ms();
  vt_status_seen_ = true;
  vt_status_timeout_reported_ = false;
  if (!vt_status_pub_) {
    return;
  }
  msg::VTStatus msg;
  msg.active_ws_sa = status.active_ws_sa;
  msg.visible_data_alarm_mask_id = status.visible_data_alarm_mask_id;
  msg.visible_soft_key_mask_id = status.visible_soft_key_mask_id;
  msg.busy_codes = status.busy_codes;
  msg.current_command_function = status.current_command_function;
  msg.busy_updating_visible_mask = status.busy_updating_visible_mask;
  msg.busy_saving_to_nonvolatile = status.busy_saving_to_nonvolatile;
  msg.busy_executing_command = status.busy_executing_command;
  msg.busy_executing_macro = status.busy_executing_macro;
  msg.parsing_active = status.parsing_active;
  msg.aux_learn_mode_active = status.aux_learn_mode_active;
  msg.out_of_memory = status.out_of_memory;
  msg.status_timeout = false;
  last_vt_status_msg_ = msg;
  has_last_vt_status_msg_ = true;
  vt_status_pub_->publish(msg);
  if (vt_aux_n_support_ && vt_aux_status_pub_) {
    msg::VTAuxStatus ast;
    ast.enabled = true;
    ast.learn_mode_active = status.aux_learn_mode_active;
    ast.last_input_locked = false;
    ast.last_input_interaction_detected = false;
    ast.last_function_object_id = 0xFFFFu;
    ast.last_token = "";
    ast.detail = "vt_status";
    vt_aux_status_pub_->publish(ast);
  }
  for (auto & kv : active_mask_ios_) {
    if (!kv.second.value_pub) {
      continue;
    }
    std_msgs::msg::String out;
    out.data = (status.visible_data_alarm_mask_id == kv.second.object_id) ? "active" : "inactive";
    kv.second.value_pub->publish(out);
  }
  for (auto & kv : softkey_mask_ios_) {
    if (!kv.second.value_pub) {
      continue;
    }
    std_msgs::msg::String out;
    out.data = (status.visible_soft_key_mask_id == kv.second.object_id) ? "active" : "inactive";
    kv.second.value_pub->publish(out);
  }
}

void VTClientNode::on_pointing_event(const PointingEvent & ev)
{
  if (!vt_pointing_pub_) {
    return;
  }
  msg::VTPointingEvent out;
  out.event_code = ev.event_code;
  out.object_id = ev.object_id;
  out.x = ev.x;
  out.y = ev.y;
  vt_pointing_pub_->publish(out);
}

void VTClientNode::on_navigation_event(const NavigationEvent & ev)
{
  if (!vt_navigation_pub_) {
    return;
  }
  msg::VTNavigationEvent out;
  out.function_code = ev.function_code;
  out.object_id = ev.object_id;
  out.pressed = (ev.function_code == 0x00 || ev.function_code == 0x01);
  vt_navigation_pub_->publish(out);
}

void VTClientNode::on_aux_input_status(
  const ObjectBinding & binding, const VtAuxInputStatus & status, double normalized_value)
{
  if (!vt_aux_n_support_) {
    return;
  }
  const auto it = aux_function_ios_.find(binding.topic_token);
  if (it == aux_function_ios_.end()) {
    return;
  }
  const auto & io = it->second;
  if (io.input_value_pub) {
    std_msgs::msg::Float64 v;
    v.data = normalized_value;
    io.input_value_pub->publish(v);
  }
  if (io.input_raw_pub) {
    msg::VTAuxInputRaw raw;
    raw.token = binding.topic_token;
    raw.type_id = (binding.aux_function_type_id != 0xFFu) ? binding.aux_function_type_id : io.type_id;
    raw.function_object_id = binding.object_id;
    raw.input_object_id = status.input_object_id;
    raw.normalized_value = normalized_value;
    raw.value1 = status.value1;
    raw.value2 = status.value2;
    raw.operating_state_bits = status.operating_state_bits;
    raw.learn_mode_active = status.learn_mode_active;
    raw.input_activated_in_learn_mode = status.input_activated_in_learn_mode;
    raw.locked = status.locked;
    raw.interaction_detected = status.interaction_detected;
    io.input_raw_pub->publish(raw);
  }
  if (vt_aux_status_pub_) {
    msg::VTAuxStatus st;
    st.enabled = true;
    st.learn_mode_active = status.learn_mode_active;
    st.last_input_locked = status.locked;
    st.last_input_interaction_detected = status.interaction_detected;
    st.last_function_object_id = binding.object_id;
    st.last_token = binding.topic_token;
    st.detail = "aux_input_status";
    vt_aux_status_pub_->publish(st);
  }
}

void VTClientNode::on_aux_assignment_value(
  const ObjectBinding & binding, const VtAuxAssignmentCommand & assignment)
{
  if (!vt_aux_n_support_) {
    return;
  }
  const auto it = aux_function_ios_.find(binding.topic_token);
  if (it == aux_function_ios_.end() || !it->second.assignment_value_pub) {
    return;
  }
  msg::VTAuxAssignment out;
  out.token = binding.topic_token;
  out.type_id = binding.aux_function_type_id;
  out.assigned = !assignment.remove_assignment;
  out.aux_input_name = assignment.aux_input_name;
  out.aux_input_object_id = assignment.aux_input_object_id;
  out.function_object_id = assignment.aux_function_object_id;
  out.store_as_preferred_assignment = assignment.store_as_preferred_assignment;
  it->second.assignment_value_pub->publish(out);
}

void VTClientNode::on_aux_assignment_result(
  const ObjectBinding & binding, bool success, std::uint8_t error_code_bits,
  const std::string & detail)
{
  if (!vt_aux_n_support_) {
    return;
  }
  const auto it = aux_function_ios_.find(binding.topic_token);
  if (it == aux_function_ios_.end() || !it->second.assignment_result_pub) {
    return;
  }
  msg::CommandResult out;
  out.success = success;
  out.error_code = error_code_bits;
  out.target = binding.topic_token + ".assignment";
  out.message = detail;
  it->second.assignment_result_pub->publish(out);
}

bool VTClientNode::validate_aux_input_name(std::uint64_t aux_input_name, std::uint8_t * resolved_sa)
{
  const auto it = name_to_sa_.find(aux_input_name);
  if (it == name_to_sa_.end()) {
    return false;
  }
  if (resolved_sa != nullptr) {
    *resolved_sa = it->second;
  }
  return true;
}

bool VTClientNode::validate_aux_input_source(std::uint8_t src_sa, std::uint64_t expected_aux_input_name)
{
  const auto it = name_to_sa_.find(expected_aux_input_name);
  if (it == name_to_sa_.end()) {
    return false;
  }
  return it->second == src_sa;
}

bool VTClientNode::resolve_name_from_sa(std::uint8_t sa, std::uint64_t * name_out)
{
  const auto it = sa_to_name_.find(sa);
  if (it == sa_to_name_.end()) {
    return false;
  }
  if (name_out != nullptr) {
    *name_out = it->second;
  }
  return true;
}

void VTClientNode::on_preferred_assignments_changed(
  const std::vector<PreferredAssignmentEntry> & entries)
{
  save_aux_preferred_assignments_to_file(entries);
}

void VTClientNode::on_change_active_mask_result(std::uint16_t mask_id, std::uint8_t error_code)
{
  const auto it = active_mask_by_id_.find(mask_id);
  if (it == active_mask_by_id_.end()) {
    return;
  }
  const auto io_it = active_mask_ios_.find(it->second);
  if (io_it == active_mask_ios_.end() || !io_it->second.result_pub) {
    return;
  }
  msg::CommandResult out;
  out.success = (error_code == 0x00);
  out.error_code = error_code;
  out.target = it->second;
  out.message = out.success ? "ok" : "error";
  io_it->second.result_pub->publish(out);
}

void VTClientNode::on_change_soft_key_mask_result(std::uint16_t mask_id, std::uint8_t error_code)
{
  const auto it = softkey_mask_by_id_.find(mask_id);
  if (it == softkey_mask_by_id_.end()) {
    return;
  }
  const auto io_it = softkey_mask_ios_.find(it->second);
  if (io_it == softkey_mask_ios_.end() || !io_it->second.result_pub) {
    return;
  }
  msg::CommandResult out;
  out.success = (error_code == 0x00);
  out.error_code = error_code;
  out.target = it->second;
  out.message = out.success ? "ok" : "error";
  io_it->second.result_pub->publish(out);
}

void VTClientNode::on_change_string_value_result(std::uint16_t object_id, std::uint8_t error_code)
{
  (void)object_id;
  (void)error_code;
}

void VTClientNode::on_change_list_item_result(
  std::uint16_t object_id, std::uint8_t list_index, std::uint8_t error_code)
{
  (void)object_id;
  (void)list_index;
  (void)error_code;
}

void VTClientNode::on_hide_show_object_result(std::uint16_t object_id, std::uint8_t error_code)
{
  std::string token;
  for (const auto & b : bindings()) {
    if (b.object_id == object_id) {
      token = b.topic_token;
      break;
    }
  }
  if (token.empty()) {
    return;
  }
  msg::CommandResult out;
  out.success = (error_code == 0x00);
  out.error_code = error_code;
  out.target = token + ".visible";
  out.message = out.success ? "ack" : "nack";
  const auto it = container_visible_ios_.find(token);
  if (it != container_visible_ios_.end() && it->second.result_pub) {
    it->second.result_pub->publish(out);
  }
}

void VTClientNode::on_hide_show_object_state(std::uint16_t object_id, bool visible)
{
  std::string token;
  for (const auto & b : bindings()) {
    if (b.object_id == object_id) {
      token = b.topic_token;
      break;
    }
  }
  if (token.empty()) {
    return;
  }
  const auto it = container_visible_ios_.find(token);
  if (it == container_visible_ios_.end() || !it->second.value_pub) {
    return;
  }
  std_msgs::msg::Bool out;
  out.data = visible;
  it->second.value_pub->publish(out);
}

void VTClientNode::on_enable_disable_object_result(std::uint16_t object_id, std::uint8_t error_code)
{
  const auto id_it = enabled_by_id_.find(object_id);
  if (id_it == enabled_by_id_.end()) {
    return;
  }
  const auto io_it = enabled_ios_.find(id_it->second);
  if (io_it == enabled_ios_.end()) {
    return;
  }
  msg::CommandResult out;
  out.success = (error_code == 0x00);
  out.error_code = error_code;
  out.target = id_it->second + ".enabled";
  out.message = out.success ? "ack" : "nack";
  if (io_it->second.result_pub) {
    io_it->second.result_pub->publish(out);
  }
}

void VTClientNode::on_select_input_object_result(std::uint16_t object_id, std::uint8_t error_code)
{
  (void)object_id;
  (void)error_code;
}

void VTClientNode::on_pool_ready(
  const std::string & xml_path,
  const BuildConfig & used_cfg,
  bool vt_reported)
{
  printInfo(
    std::string("Using ") + (vt_reported ? "VT-reported" : "fallback") +
    " display profile: dimension=" + std::to_string(used_cfg.vt_dimension) +
    ", softkey_width=" + std::to_string(used_cfg.vt_softkey_width) +
    ", softkey_height=" + std::to_string(used_cfg.vt_softkey_height) +
    ", colors=" + std::to_string(used_cfg.vt_colors));
  printInfo(
    "VT XML loaded and object pool ready from: " + xml_path +
    " (size=" + std::to_string(pool_bytes().size()) +
    " bytes, profile=" + (vt_reported ? std::string("VT-reported") : std::string("fallback")) + ")");
  if (!topics_created_) {
    create_dynamic_topics();
    topics_created_ = true;
  }
}

void VTClientNode::on_pool_build_failed(const std::string & xml_path)
{
  printWarn("VT XML load/pool parse failed: file=" + xml_path);
}

void VTClientNode::printInfo(const std::string & msg)
{
  RCLCPP_INFO(get_logger(), "%s", msg.c_str());
}

void VTClientNode::printWarn(const std::string & msg)
{
  RCLCPP_WARN(get_logger(), "%s", msg.c_str());
}

void VTClientNode::on_runtime_update_result(
  bool success, const std::string & detail, std::uint32_t pending_updates,
  std::uint32_t in_progress_updates)
{
  if (!vt_update_result_pub_) {
    return;
  }
  msg::VTUpdateResult out;
  out.success = success;
  out.pending_updates = pending_updates;
  out.in_progress_updates = in_progress_updates;
  out.detail = detail;
  vt_update_result_pub_->publish(out);
}

void VTClientNode::create_dynamic_topics()
{
  button_pubs_.clear();
  softkey_pubs_.clear();
  numeric_ios_.clear();
  string_ios_.clear();
  list_ios_.clear();
  bool_ios_.clear();
  enabled_ios_.clear();
  enabled_by_id_.clear();
  container_visible_ios_.clear();
  active_mask_ios_.clear();
  softkey_mask_ios_.clear();
  active_mask_by_id_.clear();
  softkey_mask_by_id_.clear();
  aux_function_ios_.clear();
  aux_function_by_id_.clear();
  dynamic_publishers_keepalive_.clear();
  dynamic_subscriptions_keepalive_.clear();

  for (const auto & b : bindings()) {
    switch (b.kind) {
      case ObjectKind::Button: {
        const std::string topic = "ISOBUS/vt/event/button/" + b.topic_token;
        button_pubs_[b.topic_token] = create_publisher<std_msgs::msg::UInt8>(topic, rclcpp::QoS(10));
        RCLCPP_INFO(get_logger(), "VT button base topic: %s", topic.c_str());
        break;
      }
      case ObjectKind::SoftKey: {
        const std::string topic = "ISOBUS/vt/event/softkey/" + b.topic_token;
        softkey_pubs_[b.topic_token] = create_publisher<std_msgs::msg::UInt8>(topic, rclcpp::QoS(10));
        RCLCPP_INFO(get_logger(), "VT softkey base topic: %s", topic.c_str());
        break;
      }
      case ObjectKind::NumberVariable: {
        const std::string value_topic = "ISOBUS/vt/number/" + b.topic_token + "/value";
        const std::string set_topic = "ISOBUS/vt/number/" + b.topic_token + "/set";
        NumericIo io;
        io.object_id = b.object_id;
        io.value_pub = create_publisher<std_msgs::msg::Float64>(value_topic, rclcpp::QoS(10));
        io.set_sub = create_subscription<std_msgs::msg::Float64>(
          set_topic, rclcpp::QoS(10),
          [this, oid = b.object_id](const std_msgs::msg::Float64 & msg) {
            if (oid == 0) {
              RCLCPP_WARN(this->get_logger(), "Ignoring numeric set for unresolved object id (0)");
              return;
            }
            send_numeric_value(oid, static_cast<std::int32_t>(msg.data));
          });
        numeric_ios_[b.topic_token] = io;
        RCLCPP_INFO(get_logger(), "VT number base topic: %s", ("ISOBUS/vt/number/" + b.topic_token).c_str());
        break;
      }
      case ObjectKind::InputNumber: {
        const std::string value_topic = "ISOBUS/vt/input_number/" + b.topic_token + "/value";
        const std::string set_topic = "ISOBUS/vt/input_number/" + b.topic_token + "/set";
        NumericIo io;
        io.object_id = b.object_id;
        io.value_pub = create_publisher<std_msgs::msg::Float64>(value_topic, rclcpp::QoS(10));
        io.set_sub = create_subscription<std_msgs::msg::Float64>(
          set_topic, rclcpp::QoS(10),
          [this, oid = b.object_id](const std_msgs::msg::Float64 & msg) {
            if (oid == 0) {
              RCLCPP_WARN(this->get_logger(), "Ignoring input number set for unresolved object id (0)");
              return;
            }
            send_numeric_value(oid, static_cast<std::int32_t>(msg.data));
          });
        numeric_ios_[b.topic_token] = io;
        RCLCPP_INFO(
          get_logger(), "VT input-number base topic: %s",
          ("ISOBUS/vt/input_number/" + b.topic_token).c_str());
        break;
      }
      case ObjectKind::InputBoolean: {
        const std::string value_topic = "ISOBUS/vt/input_bool/" + b.topic_token + "/value";
        const std::string set_topic = "ISOBUS/vt/input_bool/" + b.topic_token + "/set";
        BoolIo io;
        io.object_id = b.object_id;
        io.value_pub = create_publisher<std_msgs::msg::Bool>(value_topic, rclcpp::QoS(10));
        io.set_sub = create_subscription<std_msgs::msg::Bool>(
          set_topic, rclcpp::QoS(10),
          [this, oid = b.object_id](const std_msgs::msg::Bool & msg) {
            if (oid == 0) {
              RCLCPP_WARN(this->get_logger(), "Ignoring bool set for unresolved object id (0)");
              return;
            }
            send_boolean_value(oid, msg.data);
          });
        bool_ios_[b.topic_token] = io;
        RCLCPP_INFO(get_logger(), "VT input-bool base topic: %s", ("ISOBUS/vt/input_bool/" + b.topic_token).c_str());
        break;
      }
      case ObjectKind::InputString: {
        const std::string value_topic = "ISOBUS/vt/input_string/" + b.topic_token + "/value";
        const std::string set_topic = "ISOBUS/vt/input_string/" + b.topic_token + "/set";
        StringIo io;
        io.object_id = b.object_id;
        io.value_pub = create_publisher<std_msgs::msg::String>(value_topic, rclcpp::QoS(10));
        io.set_sub = create_subscription<std_msgs::msg::String>(
          set_topic, rclcpp::QoS(10),
          [this, oid = b.object_id](const std_msgs::msg::String & msg) {
            if (oid == 0) {
              RCLCPP_WARN(this->get_logger(), "Ignoring input string set for unresolved object id (0)");
              return;
            }
            send_string_value(oid, msg.data);
          });
        string_ios_[b.topic_token] = io;
        RCLCPP_INFO(
          get_logger(), "VT input-string base topic: %s",
          ("ISOBUS/vt/input_string/" + b.topic_token).c_str());
        break;
      }
      case ObjectKind::StringVariable: {
        const std::string value_topic = "ISOBUS/vt/string/" + b.topic_token + "/value";
        const std::string set_topic = "ISOBUS/vt/string/" + b.topic_token + "/set";
        StringIo io;
        io.object_id = b.object_id;
        io.value_pub = create_publisher<std_msgs::msg::String>(value_topic, rclcpp::QoS(10));
        io.set_sub = create_subscription<std_msgs::msg::String>(
          set_topic, rclcpp::QoS(10),
          [this, oid = b.object_id](const std_msgs::msg::String & msg) {
            if (oid == 0) {
              RCLCPP_WARN(this->get_logger(), "Ignoring string set for unresolved object id (0)");
              return;
            }
            send_string_value(oid, msg.data);
          });
        string_ios_[b.topic_token] = io;
        RCLCPP_INFO(get_logger(), "VT string base topic: %s", ("ISOBUS/vt/string/" + b.topic_token).c_str());
        break;
      }
      case ObjectKind::InputList:
      case ObjectKind::OutputList: {
        const std::string value_topic = "ISOBUS/vt/list/" + b.topic_token + "/value";
        const std::string set_topic = "ISOBUS/vt/list/" + b.topic_token + "/set";
        ListIo io;
        io.object_id = b.object_id;
        io.value_pub = create_publisher<std_msgs::msg::Int32>(value_topic, rclcpp::QoS(10));
        io.set_sub = create_subscription<std_msgs::msg::Int32>(
          set_topic, rclcpp::QoS(10),
          [this, oid = b.object_id](const std_msgs::msg::Int32 & msg) {
            if (oid == 0) {
              RCLCPP_WARN(this->get_logger(), "Ignoring list set for unresolved object id (0)");
              return;
            }
            const auto clamped = std::max<int32_t>(0, std::min<int32_t>(255, msg.data));
            send_list_index_value(oid, static_cast<std::uint8_t>(clamped));
          });
        list_ios_[b.topic_token] = io;
        RCLCPP_INFO(get_logger(), "VT list base topic: %s", ("ISOBUS/vt/list/" + b.topic_token).c_str());
        break;
      }
      case ObjectKind::Container: {
        ContainerVisibleIo vio;
        vio.object_id = b.object_id;
        const std::string base = "ISOBUS/vt/container/" + b.topic_token + "/visible";
        vio.value_pub = create_publisher<std_msgs::msg::Bool>(base + "/value", rclcpp::QoS(10));
        vio.result_pub = create_publisher<msg::CommandResult>(base + "/result", rclcpp::QoS(10));
        vio.set_sub = create_subscription<std_msgs::msg::Bool>(
          base + "/set", rclcpp::QoS(10),
          [this, token = b.topic_token, oid = b.object_id](const std_msgs::msg::Bool & msg) {
            msg::CommandResult res;
            res.target = token + ".visible";
            if (oid == 0) {
              res.success = false;
              res.error_code = 1;
              res.message = "unresolved object id";
            } else {
              send_visibility(oid, msg.data);
              res.success = true;
              res.error_code = 0;
              res.message = "sent";
            }
            const auto it = container_visible_ios_.find(token);
            if (it != container_visible_ios_.end() && it->second.result_pub) {
              it->second.result_pub->publish(res);
            }
          });
        container_visible_ios_[b.topic_token] = vio;
        RCLCPP_INFO(
          get_logger(), "VT container base topic: %s",
          ("ISOBUS/vt/container/" + b.topic_token).c_str());
        break;
      }
      case ObjectKind::AuxFunction: {
        if (!vt_aux_n_support_) {
          break;
        }
        AuxFunctionIo io;
        io.function_object_id = b.object_id;
        io.type_id = b.aux_function_type_id;
        const std::string base = "ISOBUS/vt/aux/" + b.topic_token;
        io.input_value_pub = create_publisher<std_msgs::msg::Float64>(base + "/input/value", rclcpp::QoS(10));
        io.input_raw_pub = create_publisher<msg::VTAuxInputRaw>(base + "/input/raw", rclcpp::QoS(10));
        io.assignment_value_pub = create_publisher<msg::VTAuxAssignment>(
          base + "/assignment/value", rclcpp::QoS(10));
        io.assignment_result_pub = create_publisher<msg::CommandResult>(
          base + "/assignment/result", rclcpp::QoS(10));
        aux_function_by_id_[b.object_id] = b.topic_token;
        aux_function_ios_[b.topic_token] = io;
        RCLCPP_INFO(get_logger(), "VT aux base topic: %s", base.c_str());
        break;
      }
      default:
        break;
    }

    // enabled attribute control for input objects and buttons
    if (b.kind == ObjectKind::Button ||
        b.kind == ObjectKind::InputNumber ||
        b.kind == ObjectKind::InputString ||
        b.kind == ObjectKind::InputBoolean) {
      EnabledIo eio;
      eio.object_id = b.object_id;
      eio.token = b.topic_token;
      std::string base;
      if (b.kind == ObjectKind::Button) {
        base = "ISOBUS/vt/button/" + b.topic_token + "/enabled";
      } else if (b.kind == ObjectKind::InputNumber) {
        base = "ISOBUS/vt/input_number/" + b.topic_token + "/enabled";
      } else if (b.kind == ObjectKind::InputString) {
        base = "ISOBUS/vt/input_string/" + b.topic_token + "/enabled";
      } else {
        base = "ISOBUS/vt/input_bool/" + b.topic_token + "/enabled";
      }
      eio.value_pub = create_publisher<std_msgs::msg::Bool>(base + "/value", rclcpp::QoS(10));
      eio.result_pub = create_publisher<msg::CommandResult>(base + "/result", rclcpp::QoS(10));
      eio.set_sub = create_subscription<std_msgs::msg::Bool>(
        base + "/set", rclcpp::QoS(10),
        [this, token = b.topic_token, oid = b.object_id](const std_msgs::msg::Bool & msg) {
          msg::CommandResult res;
          res.target = token + ".enabled";
          if (oid == 0) {
            res.success = false;
            res.error_code = 1;
            res.message = "unresolved object id";
          } else {
            send_enable_disable_object(oid, msg.data);
            res.success = true;
            res.error_code = 0;
            res.message = "sent";
          }
          const auto it = enabled_ios_.find(token);
          if (it != enabled_ios_.end() && it->second.result_pub) {
            it->second.result_pub->publish(res);
          }
        });
      enabled_by_id_[b.object_id] = b.topic_token;
      enabled_ios_[b.topic_token] = eio;
    }
  }

  for (const auto & m : data_masks()) {
    if (m.object_id == 0) {
      continue;
    }
    MaskIo io;
    io.object_id = m.object_id;
    io.token = m.topic_token;
    const auto set_topic = "ISOBUS/vt/active_mask/" + m.topic_token + "/set";
    const auto value_topic = "ISOBUS/vt/active_mask/" + m.topic_token + "/value";
    const auto result_topic = "ISOBUS/vt/active_mask/" + m.topic_token + "/result";
    io.value_pub = create_publisher<std_msgs::msg::String>(value_topic, rclcpp::QoS(10));
    io.result_pub = create_publisher<msg::CommandResult>(result_topic, rclcpp::QoS(10));
    io.set_sub = create_subscription<std_msgs::msg::Empty>(
      set_topic, rclcpp::QoS(10),
      [this, mask_id = m.object_id](const std_msgs::msg::Empty &) {
        (void)send_change_active_mask_by_id(mask_id);
      });
    active_mask_by_id_[m.object_id] = m.topic_token;
    active_mask_ios_[m.topic_token] = io;
    RCLCPP_INFO(get_logger(), "VT active-mask base topic: %s", ("ISOBUS/vt/active_mask/" + m.topic_token).c_str());
  }

  for (const auto & m : soft_key_masks()) {
    if (m.object_id == 0) {
      continue;
    }
    MaskIo io;
    io.object_id = m.object_id;
    io.token = m.topic_token;
    const auto set_topic = "ISOBUS/vt/softkey_mask/" + m.topic_token + "/set";
    const auto value_topic = "ISOBUS/vt/softkey_mask/" + m.topic_token + "/value";
    const auto result_topic = "ISOBUS/vt/softkey_mask/" + m.topic_token + "/result";
    io.value_pub = create_publisher<std_msgs::msg::String>(value_topic, rclcpp::QoS(10));
    io.result_pub = create_publisher<msg::CommandResult>(result_topic, rclcpp::QoS(10));
    io.set_sub = create_subscription<std_msgs::msg::Empty>(
      set_topic, rclcpp::QoS(10),
      [this, mask_id = m.object_id](const std_msgs::msg::Empty &) {
        (void)send_change_soft_key_mask_by_id(mask_id);
      });
    softkey_mask_by_id_[m.object_id] = m.topic_token;
    softkey_mask_ios_[m.topic_token] = io;
    RCLCPP_INFO(get_logger(), "VT softkey-mask base topic: %s", ("ISOBUS/vt/softkey_mask/" + m.topic_token).c_str());
  }
}

void VTClientNode::on_address_status(const msg::IsobusAddressStatus & status)
{
  try {
  const bool is_claimed_state = (status.state == kAddressStateClaimed);
  const bool was_local_sa_valid = local_sa_valid_;
  const std::uint8_t previous_local_sa = local_sa();
  set_local_sa(status.sa);
  local_sa_valid_ = is_claimed_state && is_valid_sa(status.sa);
  if (!local_sa_valid_) {
    return;
  }
  if (!was_local_sa_valid || previous_local_sa != local_sa()) {
    ws_definition_sent_ = false;
  }
  const auto now = now_ms();
  // WS definition (FE0D/FE0C) is sent only after VT is detected.
  // If WS maintenance initial flag is pending, FE0D/FE0C must be sent first.
  const bool can_send_ws_definition = vt_present_ && vt_status_seen_ && is_valid_sa(vt_sa());
  if (can_send_ws_definition && (!ws_definition_sent_ || ws_initial_pending())) {
    send_working_set_master_message(working_set_member_count());
    send_working_set_member_messages();
    ws_definition_sent_ = true;
    ws_definition_sa_ = local_sa();
  }
  // Start cyclic VT maintenance immediately once our own SA is valid/address-claimed.
  const bool can_send_ws_maintenance =
    ws_definition_sent_ && vt_present_ && vt_status_seen_ && is_valid_sa(vt_sa());
  if (can_send_ws_maintenance &&
      (!was_local_sa_valid || last_ws_maintenance_tx_ms_ == 0) &&
      ws_maintenance_period_ms_ > 0) {
    send_working_set_maintenance_message();
    last_ws_maintenance_tx_ms_ = now;
  }

  // Defensive: ensure our own SA is never used as VT destination.
  if (!vt_candidates_.empty()) {
    vt_candidates_.erase(
      std::remove(vt_candidates_.begin(), vt_candidates_.end(), local_sa()),
      vt_candidates_.end());
    vt_candidate_set_.erase(local_sa());
    if (vt_candidates_.empty()) {
      vt_present_ = false;
      vt_candidate_index_ = 0;
    } else if (vt_candidate_index_ >= vt_candidates_.size()) {
      vt_candidate_index_ = 0;
    }
  }

  const bool vt_start_ready = vt_present_ && vt_status_seen_ && (!vt_wait_address_claim_ || vt_seen_in_address_book_);
  if (!session_started_ && vt_start_ready) {
    printInfo("Valid SA and VT presence detected, starting VT session");
    if (!vt_candidates_.empty()) {
      vt_candidate_index_ = std::min(vt_candidate_index_, vt_candidates_.size() - 1);
      set_vt_sa(vt_candidates_[vt_candidate_index_]);
    }
    // Send WS definition once per local SA before first E700/session start.
    if (!ws_definition_sent_ || ws_definition_sa_ != local_sa() || ws_initial_pending()) {
      send_working_set_master_message(working_set_member_count());
      send_working_set_member_messages();
      ws_definition_sent_ = true;
      ws_definition_sa_ = local_sa();
    }
    // Mandatory ordering: do not send any VT session commands before E700 maintenance.
    if (!(local_sa_valid_ && is_valid_sa(vt_sa()))) {
      printWarn("Delay VT session start: cannot send WS maintenance yet (invalid SA state)");
      return;
    }
    send_working_set_maintenance_message();
    last_ws_maintenance_tx_ms_ = now;
    start_session(now_ms());
    session_started_ = true;
    return;
  }

  } catch (const std::exception & ex) {
    printWarn(std::string("Exception in on_address_status: ") + ex.what());
  } catch (...) {
    printWarn("Unknown exception in on_address_status");
  }
}

void VTClientNode::on_address_book(const msg::IsobusAddressBook & book)
{
  try {
  constexpr std::uint8_t kVtFunctionId = 29;
  name_to_sa_.clear();
  sa_to_name_.clear();
  vt_present_ = false;
  vt_candidates_.clear();
  vt_candidate_set_.clear();
  for (const auto & e : book.entries) {
    if (!is_valid_sa(e.sa)) {
      continue;
    }
    const std::uint64_t name_be = name_from_bytes_be(e.name);
    name_to_sa_[name_be] = e.sa;
    sa_to_name_[e.sa] = name_be;
    // AddressClaim NAME is often represented little-endian in payload snapshots.
    std::array<std::uint8_t, 8> rev = e.name;
    std::reverse(rev.begin(), rev.end());
    const std::uint64_t name_le_as_be = name_from_bytes_be(rev);
    name_to_sa_[name_le_as_be] = e.sa;
    // Never consider our own SA as VT candidate.
    if (local_sa_valid_ && e.sa == local_sa()) {
      continue;
    }
    const bool is_vt = (name_function_id(name_be) == kVtFunctionId) ||
                       (name_function_id(name_le_as_be) == kVtFunctionId);
    if (!is_vt) {
      continue;
    }
    if (vt_candidate_set_.insert(e.sa).second) {
      vt_candidates_.push_back(e.sa);
    }
    vt_present_ = true;
    vt_seen_in_address_book_ = true;
    if (!vt_sa_locked_from_address_book_) {
      set_vt_sa(e.sa);
      printInfo(
        "VT SA detected from AddressBook NAME function 29: 0x" +
        [&]() {
          char b[8];
          std::snprintf(b, sizeof(b), "%02X", e.sa);
          return std::string(b);
        }());
      vt_sa_locked_from_address_book_ = true;
    }
  }

  if (!vt_present_) {
    // VT disappeared from address book: reset start gates so a fresh WS/maintenance/session
    // sequence is executed when VT appears again.
    session_started_ = false;
    vt_status_seen_ = false;
    ws_definition_sent_ = false;
    vt_sa_locked_from_address_book_ = false;
    request_ws_initial_maintenance();
    printWarn("No VT found from AddressBook NAME function 29 yet");
    return;
  }

  if (vt_candidate_index_ >= vt_candidates_.size()) {
    vt_candidate_index_ = 0;
  }
  set_vt_sa(vt_candidates_[vt_candidate_index_]);

  const bool vt_start_ready = vt_present_ && vt_status_seen_ && (!vt_wait_address_claim_ || vt_seen_in_address_book_);
  if (!session_started_ && local_sa_valid_ && vt_start_ready) {
    printInfo("Valid SA and VT presence detected, starting VT session");
    // Send WS definition once per local SA before first E700/session start.
    if (!ws_definition_sent_ || ws_definition_sa_ != local_sa() || ws_initial_pending()) {
      send_working_set_master_message(working_set_member_count());
      send_working_set_member_messages();
      ws_definition_sent_ = true;
      ws_definition_sa_ = local_sa();
    }
    // Mandatory ordering: do not send any VT session commands before E700 maintenance.
    if (!is_valid_sa(vt_sa())) {
      printWarn("Delay VT session start: cannot send WS maintenance yet (invalid VT SA)");
      return;
    }
    send_working_set_maintenance_message();
    last_ws_maintenance_tx_ms_ = now_ms();
    start_session(now_ms());
    session_started_ = true;
  }
  } catch (const std::exception & ex) {
    printWarn(std::string("Exception in on_address_book: ") + ex.what());
  } catch (...) {
    printWarn("Unknown exception in on_address_book");
  }
}

void VTClientNode::on_bus_frame(const msg::IsobusFrame & frame)
{
  try {
  if ((frame.pgn & 0x03FF00U) == (kPgnAcknowledgement & 0x03FF00U)) {
    on_acknowledgement(frame);
    return;
  }
  // For PDU1 VT/AUX traffic, accept both PF 0xE6 and 0xE7.
  if (frame.pf != 0xE6 && frame.pf != 0xE7) {
    return;
  }
  if (local_sa_valid_ && frame.sa == local_sa()) {
    return;
  }
  if (local_sa_valid_ && frame.ps != local_sa() && frame.ps != 0xFF) {
    return;
  }
  // VT status (fn=0xFE) can be used as an early VT presence hint.
  if (!frame.data.empty() && frame.data[0] == 0xFE) {
    observe_vt_status_source(frame.sa);
    // Always process VT Status after source observation so status gating can open
    // even before VT SA lock/address-book convergence.
    std::vector<std::uint8_t> payload(frame.data.begin(), frame.data.end());
    handle_vt_payload(payload, now_ms(), frame.sa);
    return;
  }
  // Accept frames from VT SA, or known AUX source SAs resolved from preferred/assignment NAME mapping.
  const bool from_vt = (frame.sa == vt_sa());
  const bool from_known_aux = vt_aux_n_support_ && is_known_aux_source_sa(frame.sa);
  // AUX-N runtime payloads.
  const bool is_aux_input_status = vt_aux_n_support_ && frame.data[0] == 0x26;
  const bool is_aux_input_maintenance = vt_aux_n_support_ && frame.data[0] == 0x23;
  if (!from_vt && !from_known_aux && !is_aux_input_status && !is_aux_input_maintenance) {
    return;
  }

  std::vector<std::uint8_t> payload(frame.data.begin(), frame.data.end());
  handle_vt_payload(payload, now_ms(), frame.sa);
  } catch (const std::exception & ex) {
    printWarn(std::string("Exception in on_bus_frame: ") + ex.what());
  } catch (...) {
    printWarn("Unknown exception in on_bus_frame");
  }
}

void VTClientNode::on_acknowledgement(const msg::IsobusFrame & frame)
{
  if (local_sa_valid_ && frame.sa == local_sa()) {
    return;
  }
  if (local_sa_valid_ && frame.ps != local_sa() && frame.ps != 0xFF) {
    return;
  }
  if (frame.sa != vt_sa()) {
    return;
  }

  const std::uint8_t control = frame.data[0];
  const std::uint8_t group_fn_value = frame.data[1];
  const std::uint8_t nack_sa = frame.data[4];
  const auto hex_u8 = [](std::uint8_t v) {
    char b[8];
    std::snprintf(b, sizeof(b), "%02X", static_cast<unsigned>(v));
    return std::string(b);
  };
  const std::uint32_t acked_pgn = static_cast<std::uint32_t>(frame.data[5]) |
    (static_cast<std::uint32_t>(frame.data[6]) << 8U) |
    (static_cast<std::uint32_t>(frame.data[7]) << 16U);

  if (acked_pgn != kPgnVtToWs &&
      acked_pgn != kPgnWsToVt &&
      acked_pgn != kPgnWorkingSetMaster &&
      acked_pgn != kPgnWorkingSetMember) {
    return;
  }

  const char * control_text = "OTHER";
  switch (control) {
    case 0x00: control_text = "ACK"; break;
    case 0x01: control_text = "NACK"; break;
    case 0x02: control_text = "AccessDenied"; break;
    case 0x03: control_text = "CannotRespond"; break;
    default: break;
  }

  const std::string msg =
    std::string("VT ACKM ") + control_text +
    ": GFV=0x" + hex_u8(group_fn_value) +
    " NACK_SA=0x" + hex_u8(nack_sa) +
    " PGN=0x" + [&]() {
      char b[16];
      std::snprintf(b, sizeof(b), "%06X", static_cast<unsigned>(acked_pgn & 0x3FFFFU));
      return std::string(b);
    }();

  if (control == 0x01 || control == 0x02 || control == 0x03) {
    printWarn(msg);
  } else {
    printInfo(msg);
  }
}

void VTClientNode::on_bus_tp(const msg::IsobusTpFrame & tp)
{
  try {
  const std::uint8_t pgn_pf = static_cast<std::uint8_t>((tp.pgn >> 8) & 0xFF);
  if (tp.pf != 0xE6 && tp.pf != 0xE7 && pgn_pf != 0xE6 && pgn_pf != 0xE7) {
    return;
  }
  if (local_sa_valid_ && tp.sa == local_sa()) {
    return;
  }
  if (local_sa_valid_ && tp.ps != local_sa() && tp.ps != 0xFF) {
    return;
  }
  if (!tp.data.empty() && tp.data[0] == 0xFE) {
    observe_vt_status_source(tp.sa);
    // Always process VT Status after source observation so status gating can open
    // even before VT SA lock/address-book convergence.
    handle_vt_payload(tp.data, now_ms(), tp.sa);
    return;
  }
  const bool from_vt = (tp.sa == vt_sa());
  const bool from_known_aux = vt_aux_n_support_ && is_known_aux_source_sa(tp.sa);
  const bool is_aux_input_status = vt_aux_n_support_ && !tp.data.empty() && tp.data[0] == 0x26;
  const bool is_aux_input_maintenance = vt_aux_n_support_ && !tp.data.empty() && tp.data[0] == 0x23;
  if (!from_vt && !from_known_aux && !is_aux_input_status && !is_aux_input_maintenance) {
    return;
  }

  handle_vt_payload(tp.data, now_ms(), tp.sa);
  } catch (const std::exception & ex) {
    printWarn(std::string("Exception in on_bus_tp: ") + ex.what());
  } catch (...) {
    printWarn("Unknown exception in on_bus_tp");
  }
}

void VTClientNode::on_bus_tp_tx_status(const msg::IsobusTpTxStatus & st)
{
  try {
    notify_tp_tx_status(st.pgn, st.sa, st.da, st.state, now_ms());
  } catch (const std::exception & ex) {
    printWarn(std::string("Exception in on_bus_tp_tx_status: ") + ex.what());
  } catch (...) {
    printWarn("Unknown exception in on_bus_tp_tx_status");
  }
}

void VTClientNode::on_vt_update(const std_msgs::msg::String & update)
{
  try {
    if (update.data.empty()) {
      printWarn("VT update ignored: empty XML fragment");
      return;
    }
    (void)apply_xml_update(update.data);
  } catch (const std::exception & ex) {
    printWarn(std::string("Exception in on_vt_update: ") + ex.what());
  } catch (...) {
    printWarn("Unknown exception in on_vt_update");
  }
}

void VTClientNode::on_session_tick()
{
  try {
  const auto now = now_ms();
  const bool can_send_ws_maintenance =
    ws_definition_sent_ && vt_present_ && vt_status_seen_ && is_valid_sa(vt_sa());
  if (local_sa_valid_ && can_send_ws_maintenance && ws_maintenance_period_ms_ > 0 &&
      (last_ws_maintenance_tx_ms_ == 0 ||
      (now - last_ws_maintenance_tx_ms_) >= ws_maintenance_period_ms_)) {
    send_working_set_maintenance_message();
    last_ws_maintenance_tx_ms_ = now;
  }
  if (!session_started_) {
    // Start attempt can be missed in address callbacks if VT status arrives later.
    const bool vt_start_ready =
      local_sa_valid_ && vt_present_ && vt_status_seen_ &&
      is_valid_sa(vt_sa()) &&
      (!vt_wait_address_claim_ || vt_seen_in_address_book_);
    if (vt_start_ready) {
      printInfo("Valid SA and VT presence detected, starting VT session");
      if (!ws_definition_sent_ || ws_definition_sa_ != local_sa() || ws_initial_pending()) {
        send_working_set_master_message(working_set_member_count());
        send_working_set_member_messages();
        ws_definition_sent_ = true;
        ws_definition_sa_ = local_sa();
      }
      send_working_set_maintenance_message();
      last_ws_maintenance_tx_ms_ = now;
      start_session(now);
      session_started_ = true;
    }
    return;
  }
  if (session_state() == SessionState::Active &&
      vt_status_timeout_ms_ > 0 &&
      last_vt_status_rx_ms_ > 0 &&
      (now - last_vt_status_rx_ms_) >= vt_status_timeout_ms_) {
    if (!vt_status_timeout_reported_) {
      vt_status_timeout_reported_ = true;
      printWarn(
        "VT status timeout: no VT status for " +
        std::to_string(static_cast<unsigned>(now - last_vt_status_rx_ms_)) +
        " ms, restarting VT session");
      if (vt_status_pub_) {
        msg::VTStatus timeout_msg{};
        if (has_last_vt_status_msg_) {
          timeout_msg = last_vt_status_msg_;
        } else {
          timeout_msg.active_ws_sa = 0xFF;
          timeout_msg.visible_data_alarm_mask_id = 0xFFFF;
          timeout_msg.visible_soft_key_mask_id = 0xFFFF;
          timeout_msg.current_command_function = 0xFF;
        }
        timeout_msg.status_timeout = true;
        vt_status_pub_->publish(timeout_msg);
      }
    }
    if (!(local_sa_valid_ && vt_present_ && is_valid_sa(vt_sa()))) {
      printWarn("Delay VT session restart: cannot send WS maintenance yet (invalid SA/VT state)");
      session_started_ = false;
      ws_definition_sent_ = false;
      vt_sa_locked_from_address_book_ = false;
      request_ws_initial_maintenance();
      return;
    }
    // VT timeout: arm full re-handshake and wait for a fresh VT status before restart.
    session_started_ = false;
    vt_status_seen_ = false;
    ws_definition_sent_ = false;
    vt_sa_locked_from_address_book_ = false;
    request_ws_initial_maintenance();
    printWarn("VT session restart deferred: waiting fresh VT status before re-handshake");
    return;
  }
  tick_session(now);
  if (vt_session_state_pub_) {
    msg::VTSessionState st;
    st.state = VTClient::session_state_name(session_state());
    st.retry_count = static_cast<std::uint8_t>(std::min<std::uint32_t>(255u, session_retry_count()));
    st.max_retries = static_cast<std::uint8_t>(std::min<std::uint32_t>(255u, session_max_retries()));
    st.pool_ready = !pool_bytes().empty();
    vt_session_state_pub_->publish(st);
  }
  if (vt_diag_pub_) {
    diagnostic_msgs::msg::DiagnosticArray arr;
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.name = "vt_client";
    ds.hardware_id = "isobus_vt";
    ds.level = (session_state() == SessionState::Error) ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    ds.message = VTClient::session_state_name(session_state());
    arr.status.push_back(ds);
    vt_diag_pub_->publish(arr);
  }

  const auto state = session_state();
  if (state == SessionState::Error && last_session_state_ != SessionState::Error) {
    printWarn("VT session entered Error state, waiting external restart trigger");
  }
  last_session_state_ = state;
  } catch (const std::exception & ex) {
    printWarn(std::string("Exception in on_session_tick: ") + ex.what());
  } catch (...) {
    printWarn("Unknown exception in on_session_tick");
  }
}

void VTClientNode::load_aux_preferred_assignments_from_file()
{
  if (vt_aux_preferred_assignment_file_.empty()) {
    return;
  }
  std::ifstream in(vt_aux_preferred_assignment_file_);
  if (!in.good()) {
    return;
  }
  std::vector<PreferredAssignmentEntry> entries;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    std::string name_hex;
    std::string input_hex;
    std::string function_hex;
    if (!(ss >> name_hex >> input_hex >> function_hex)) {
      continue;
    }
    try {
      PreferredAssignmentEntry e{};
      e.aux_input_name = std::stoull(name_hex, nullptr, 16);
      e.aux_input_object_id = static_cast<std::uint16_t>(std::stoul(input_hex, nullptr, 16));
      e.aux_function_object_id = static_cast<std::uint16_t>(std::stoul(function_hex, nullptr, 16));
      entries.push_back(e);
    } catch (const std::exception &) {
      continue;
    }
  }
  set_preferred_assignments(entries);
  if (!entries.empty()) {
    printInfo(
      "Loaded AUX preferred assignments from file: " +
      vt_aux_preferred_assignment_file_ + " (" + std::to_string(entries.size()) + ")");
  }
}

void VTClientNode::save_aux_preferred_assignments_to_file(
  const std::vector<PreferredAssignmentEntry> & entries)
{
  if (vt_aux_preferred_assignment_file_.empty()) {
    return;
  }
  try {
    const auto parent = std::filesystem::path(vt_aux_preferred_assignment_file_).parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
  } catch (const std::exception &) {
    // Best effort: continue and let ofstream report failure below.
  }
  std::ofstream out(vt_aux_preferred_assignment_file_, std::ios::trunc);
  if (!out.good()) {
    printWarn("Failed to save AUX preferred assignments: " + vt_aux_preferred_assignment_file_);
    return;
  }
  out << "# aux_input_name_hex aux_input_object_id_hex aux_function_object_id_hex\n";
  for (const auto & e : entries) {
    char name_buf[24];
    char in_buf[8];
    char fn_buf[8];
    std::snprintf(name_buf, sizeof(name_buf), "%016llX", static_cast<unsigned long long>(e.aux_input_name));
    std::snprintf(in_buf, sizeof(in_buf), "%04X", static_cast<unsigned>(e.aux_input_object_id));
    std::snprintf(fn_buf, sizeof(fn_buf), "%04X", static_cast<unsigned>(e.aux_function_object_id));
    out << name_buf << " " << in_buf << " " << fn_buf << "\n";
  }
}

void VTClientNode::observe_vt_status_source(std::uint8_t sa)
{
  if (!is_valid_sa(sa)) return;
  if (local_sa_valid_ && sa == local_sa()) return;

  if (vt_candidate_set_.insert(sa).second) {
    vt_candidates_.push_back(sa);
    printInfo(
      "VT SA hinted by VT status message: 0x" + [&]() {
        char b[8];
        std::snprintf(b, sizeof(b), "%02X", sa);
        return std::string(b);
      }());
  }
  vt_present_ = true;

  if (!vt_sa_locked_from_address_book_) {
    set_vt_sa(sa);
  }
}

std::uint64_t VTClientNode::now_ms()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

bool VTClientNode::is_valid_sa(std::uint8_t sa)
{
  return sa <= 0xFD;
}

std::uint64_t VTClientNode::name_from_bytes_be(const std::array<std::uint8_t, 8> & name)
{
  std::uint64_t value = 0;
  for (std::uint8_t byte : name) {
    value = (value << 8) | static_cast<std::uint64_t>(byte);
  }
  return value;
}

std::uint8_t VTClientNode::name_function_id(std::uint64_t name)
{
  return static_cast<std::uint8_t>((name >> 40) & 0xFFu);
}

}  // namespace ros2_isobus

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_isobus::VTClientNode>());
  rclcpp::shutdown();
  return 0;
}
  const auto hex_u8 = [](std::uint8_t v) {
    char b[8];
    std::snprintf(b, sizeof(b), "%02X", static_cast<unsigned>(v));
    return std::string(b);
  };
