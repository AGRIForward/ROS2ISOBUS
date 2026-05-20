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
#include <cstdint>
#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ros2_isobus/msg/isobus_frame.hpp"
#include "ros2_isobus/msg/isobus_tp_frame.hpp"
#include "VTProtocolCodec.hpp"
#include "VTPoolModel.hpp"

namespace ros2_isobus
{

/*
 *
 * VTClient implements ISO 11783-6 Virtual Terminal client-side workflow:
 *  - Builds object pool bytes from PoolEdit XML
 *  - Executes VT session state machine (version -> pool transfer -> activation)
 *  - Encodes/decodes VT function payloads over Node-to-VT / VT-to-Node PGNs
 *  - Exposes button/numeric VT events through transport-agnostic callbacks
 *
 * NOTE:
 *  - Address claim ownership (local SA) is provided by ISO 11783-5 AddressManager
 *  - ROS 2 pub/sub bindings are implemented in VTClientNode
 *
 */
class VTClient
{
public:
  using ObjectKind = ros2_isobus::ObjectKind;
  using ObjectBinding = ros2_isobus::ObjectBinding;
  using MaskBinding = ros2_isobus::MaskBinding;
  using BuildConfig = ros2_isobus::BuildConfig;
  using VersionResponse = ros2_isobus::VersionResponse;
  using PoolTransferResponse = ros2_isobus::PoolTransferResponse;
  using MemoryResponse = ros2_isobus::MemoryResponse;
  using VtStatus = ros2_isobus::VtStatus;
  using PointingEvent = ros2_isobus::PointingEvent;
  using NavigationEvent = ros2_isobus::NavigationEvent;
  using VtAuxInputMaintenance = ros2_isobus::VtAuxInputMaintenance;
  using VtPreferredAssignmentResponse = ros2_isobus::VtPreferredAssignmentResponse;

  VTClient(std::uint8_t local_sa, std::uint8_t vt_sa);
  virtual ~VTClient() = default;
  // VT session orchestration states (ISO 11783-6 service sequence).
  enum class SessionState
  {
    Idle,
    RequestVersion,
    WaitVersionResponse,
    RequestMemory,
    WaitMemoryResponse,
    WaitPoolTransferResponse,
    WaitPoolActivateResponse,
    Active,
    Error
  };

  // Load/parse object pool XML into internal model.
  bool load_pool_from_xml(const std::string & xml_path, const BuildConfig & cfg);
  // Read-only pool model accessors.
  const std::vector<std::uint8_t> & pool_bytes() const { return pool_model_.pool_bytes(); }
  const std::vector<ObjectBinding> & bindings() const { return pool_model_.bindings(); }
  const std::vector<MaskBinding> & data_masks() const { return pool_model_.data_masks(); }
  const std::vector<MaskBinding> & soft_key_masks() const { return pool_model_.soft_key_masks(); }
  std::uint16_t parsed_working_set_id() const { return pool_model_.working_set_id(); }

  // Address/identifier configuration.
  void set_local_sa(std::uint8_t sa) { local_sa_ = sa; }
  std::uint8_t local_sa() const { return local_sa_; }
  void set_vt_sa(std::uint8_t sa) { vt_sa_ = sa; }
  std::uint8_t vt_sa() const { return vt_sa_; }
  void set_pool_identifiers(std::uint16_t working_set_id, std::uint16_t object_pool_id);
  void set_working_set_member_names(const std::vector<std::uint64_t> & member_names);
  void set_working_set_maintenance_version(std::uint8_t version) { ws_version_number_ = version; }
  void request_ws_initial_maintenance() { ws_initial_pending_ = true; }
  void set_aux_n_support(bool enabled) { aux_n_support_ = enabled; }
  void set_aux_preferred_store_override(bool enabled) { aux_preferred_store_override_ = enabled; }
  bool aux_n_support() const { return aux_n_support_; }
  struct PreferredAssignmentEntry
  {
    std::uint64_t aux_input_name{0xFFFFFFFFFFFFFFFFULL};
    std::uint16_t aux_input_object_id{0xFFFF};
    std::uint16_t aux_function_object_id{0xFFFF};
  };
  void set_preferred_assignments(const std::vector<PreferredAssignmentEntry> & entries);
  std::vector<PreferredAssignmentEntry> preferred_assignments() const;
  std::uint8_t working_set_member_count() const
  {
    // ISO 11783-7: member count in WS master message is the number of WS member NAME
    // entries that follow (master itself is not counted here).
    return static_cast<std::uint8_t>(std::min<std::size_t>(250U, ws_member_names_.size()));
  }
  void set_session_timing(std::uint32_t timeout_ms, std::uint32_t max_retries);
  void configure_pool_source(const std::string & xml_path, const BuildConfig & fallback_cfg, bool use_vt_reported);

  // Session request/send primitives.
  void request_vt_version();
  void request_vt_memory();
  void send_working_set_master_message(std::uint8_t member_count);
  void send_working_set_maintenance_message();
  void send_working_set_member_messages();
  void send_pool_to_vt(std::uint64_t now_ms);
  void notify_tp_tx_status(
    std::uint32_t pgn, std::uint8_t sa, std::uint8_t da, std::uint8_t state, std::uint64_t now_ms);

  // VT command helpers for application-facing operations.
  void send_numeric_value(std::uint16_t object_id, std::int32_t value);
  void send_boolean_value(std::uint16_t object_id, bool value);
  void send_string_value(std::uint16_t object_id, const std::string & value);
  void send_list_index_value(std::uint16_t object_id, std::uint8_t index);
  void send_visibility(std::uint16_t object_id, bool visible);
  void send_enable_disable_object(std::uint16_t object_id, bool enabled);
  void send_select_input_object(std::uint16_t object_id);
  void send_change_attribute(std::uint16_t object_id, std::uint8_t attribute_id, std::uint32_t value);
  void send_change_child_position(
    std::uint16_t parent_object_id, std::uint16_t child_object_id, std::uint16_t x, std::uint16_t y);
  bool send_change_active_mask_by_id(std::uint16_t mask_id);
  bool send_change_soft_key_mask_by_id(std::uint16_t mask_id);
  // Apply one XML element update fragment and transmit matching VT command(s).
  // Example: <inputnumber id="123" value="42"/>
  bool apply_xml_update(const std::string & xml_element);

  // Main receive/session control entry points.
  void handle_vt_payload(const std::vector<std::uint8_t> & payload, std::uint64_t now_ms, std::uint8_t src_sa = 0xFF);
  void start_session(std::uint64_t now_ms);
  void tick_session(std::uint64_t now_ms);
  bool session_active() const { return session_state_ == SessionState::Active; }
  SessionState session_state() const { return session_state_; }
  std::uint32_t session_retry_count() const { return session_retry_count_; }
  std::uint32_t session_max_retries() const { return session_max_retries_; }
  bool ws_initial_pending() const { return ws_initial_pending_; }
  bool is_known_aux_source_sa(std::uint8_t sa);
  static const char * session_state_name(SessionState s);

protected:
  // Transport hooks implemented by VTClientNode.
  virtual void send_frame(const msg::IsobusFrame & frame) = 0;
  virtual void send_tp_frame(const msg::IsobusTpFrame & tp) = 0;

  // Event hooks implemented by VTClientNode.
  virtual void on_button_event(
    const ObjectBinding & binding, bool pressed, std::uint8_t activation_code) = 0;
  virtual void on_softkey_event(
    const ObjectBinding & binding, bool pressed, std::uint8_t activation_code) = 0;
  virtual void on_numeric_event(const ObjectBinding & binding, std::int32_t value) = 0;
  virtual void on_string_event(const ObjectBinding & binding, const std::string & value)
  {
    (void)binding;
    (void)value;
  }
  virtual void on_list_event(const ObjectBinding & binding, std::int32_t index)
  {
    (void)binding;
    (void)index;
  }
  virtual void on_vt_status(const VtStatus & status) {(void)status;}
  virtual void on_pointing_event(const PointingEvent & ev) {(void)ev;}
  virtual void on_navigation_event(const NavigationEvent & ev) {(void)ev;}
  virtual void on_aux_input_status(
    const ObjectBinding & binding, const VtAuxInputStatus & status, double normalized_value)
  {
    (void)binding;
    (void)status;
    (void)normalized_value;
  }
  virtual void on_aux_assignment_value(const ObjectBinding & binding, const VtAuxAssignmentCommand & assignment)
  {
    (void)binding;
    (void)assignment;
  }
  virtual void on_aux_assignment_result(
    const ObjectBinding & binding, bool success, std::uint8_t error_code_bits, const std::string & detail)
  {
    (void)binding;
    (void)success;
    (void)error_code_bits;
    (void)detail;
  }
  // Validate AUX input NAME against bus address-book context (ISO 11783-5 NAME/SA tracking).
  // Return true when the NAME is known/acceptable for assignment handling.
  virtual bool validate_aux_input_name(std::uint64_t aux_input_name, std::uint8_t * resolved_sa)
  {
    (void)aux_input_name;
    if (resolved_sa != nullptr) {
      *resolved_sa = 0xFE;
    }
    return true;
  }
  // Validate that incoming AUX status source SA corresponds to expected AUX NAME.
  virtual bool validate_aux_input_source(std::uint8_t src_sa, std::uint64_t expected_aux_input_name)
  {
    (void)src_sa;
    (void)expected_aux_input_name;
    return true;
  }
  virtual bool resolve_name_from_sa(std::uint8_t sa, std::uint64_t * name_out)
  {
    (void)sa;
    if (name_out != nullptr) {
      *name_out = 0xFFFFFFFFFFFFFFFFULL;
    }
    return false;
  }
  virtual void on_preferred_assignments_changed(const std::vector<PreferredAssignmentEntry> & entries)
  {
    (void)entries;
  }
  virtual void on_change_active_mask_result(std::uint16_t mask_id, std::uint8_t error_code)
  {
    (void)mask_id;
    (void)error_code;
  }
  virtual void on_change_soft_key_mask_result(std::uint16_t mask_id, std::uint8_t error_code)
  {
    (void)mask_id;
    (void)error_code;
  }
  virtual void on_change_string_value_result(std::uint16_t object_id, std::uint8_t error_code)
  {
    (void)object_id;
    (void)error_code;
  }
  virtual void on_change_list_item_result(std::uint16_t object_id, std::uint8_t list_index, std::uint8_t error_code)
  {
    (void)object_id;
    (void)list_index;
    (void)error_code;
  }
  virtual void on_hide_show_object_result(std::uint16_t object_id, std::uint8_t error_code)
  {
    (void)object_id;
    (void)error_code;
  }
  virtual void on_hide_show_object_state(std::uint16_t object_id, bool visible)
  {
    (void)object_id;
    (void)visible;
  }
  virtual void on_enable_disable_object_result(std::uint16_t object_id, std::uint8_t error_code)
  {
    (void)object_id;
    (void)error_code;
  }
  virtual void on_select_input_object_result(std::uint16_t object_id, std::uint8_t error_code)
  {
    (void)object_id;
    (void)error_code;
  }
  virtual void on_pool_ready(const std::string & xml_path, const BuildConfig & used_cfg, bool vt_reported) {}
  virtual void on_pool_build_failed(const std::string & xml_path) {}
  virtual void on_runtime_update_result(
    bool success, const std::string & detail, std::uint32_t pending_updates,
    std::uint32_t in_progress_updates)
  {
    (void)success;
    (void)detail;
    (void)pending_updates;
    (void)in_progress_updates;
  }
  // Logging hooks implemented by VTClientNode.
  virtual void printInfo(const std::string & msg) = 0;
  virtual void printWarn(const std::string & msg) = 0;

private:
  // Low-level command/path helpers.
  void send_vt_command(
    VTProtocolCodec::Function function, const std::vector<std::uint8_t> & args = {});
  void send_change_active_mask_command();
  void send_change_soft_key_mask_command(std::uint16_t mask_id);
  std::uint16_t active_mask_id() const;
  bool send_pool_transfer_message();
  void send_end_of_object_pool_message();
  void report_vt_status_issues(const VtStatus & st);
  static double normalize_aux_value(std::uint8_t type_id, std::uint16_t value1, std::uint16_t value2);
  static std::string to_hex_u8(std::uint8_t value);
  static const char * transfer_status_text(std::uint8_t status);
  static const char * memory_status_text(std::uint8_t status);
  static std::string hardware_bits_text(std::uint8_t bits);
  static bool extract_vt_build_profile(const std::vector<std::uint8_t> & payload, BuildConfig & cfg_out);
  void ensure_pool_loaded_from_version_payload(const std::vector<std::uint8_t> & payload);
  void transition_to(SessionState next, std::uint64_t now_ms, const std::string & reason);
  void restart_or_fail(std::uint64_t now_ms, const std::string & reason);
  bool start_runtime_update(const std::string & xml_element);
  void dispatch_next_pending_runtime_update();
  void report_runtime_update_result(bool success, const std::string & detail);
  void try_send_preferred_assignment_command();

  // Runtime source/destination addresses.
  std::uint8_t local_sa_{0xFE};
  std::uint8_t vt_sa_{0x26};

  // Stateless protocol helpers and stateful pool model.
  VTProtocolCodec codec_{};
  VTPoolModel pool_model_{};
  // De-duplicate warnings for unknown VT function ids.
  std::unordered_set<std::uint8_t> warned_unparsed_functions_;
  // Active WS/pool identifiers used in VT command payloads.
  std::uint16_t vt_working_set_id_{0x0000};
  std::uint16_t vt_object_pool_id_{0x0000};
  // Optional Working Set member NAME list for PGN 65036.
  std::vector<std::uint64_t> ws_member_names_{};
  // Working Set maintenance configuration (PGN 65037 payload).
  std::uint8_t ws_version_number_{6};
  bool ws_initial_pending_{true};

  // Session state-machine runtime.
  SessionState session_state_{SessionState::Idle};
  std::uint32_t session_timeout_ms_{1000};
  std::uint32_t session_max_retries_{3};
  std::uint32_t session_retry_count_{0};
  std::uint64_t session_deadline_ms_{0};

  // Pool transfer / parse completion tracking.
  bool pool_transfer_response_received_{false};
  std::uint8_t vt_status_parsing_clear_count_{0};

  // Last VT status snapshot for change-only logging.
  bool vt_status_prev_valid_{false};
  std::uint8_t vt_status_prev_active_ws_sa_{0xFF};
  std::uint16_t vt_status_prev_data_mask_id_{0xFFFF};
  std::uint16_t vt_status_prev_soft_key_mask_id_{0xFFFF};
  std::uint8_t vt_status_prev_busy_codes_{0x00};
  std::uint8_t vt_status_prev_current_fn_{0xFF};

  // End-of-pool retry/fallback window controls.
  std::uint64_t pool_transfer_wait_started_ms_{0};
  bool end_of_pool_send_pending_{false};
  bool waiting_pool_tp_tx_complete_{false};
  std::uint32_t runtime_update_eop_inflight_{0};
  bool runtime_update_waiting_memory_{false};
  std::vector<std::uint8_t> runtime_update_pool_bytes_{};
  std::deque<std::string> pending_pool_updates_{};
  mutable std::mutex pending_updates_mutex_{};
  std::uint8_t end_of_pool_retry_count_{0};
  bool missing_fn18_wait_warned_{false};

  // Pool source and build profile settings.
  std::string pool_xml_path_{};
  std::string source_xml_declaration_{};
  std::string source_objectpool_open_tag_{};
  BuildConfig fallback_cfg_{};
  BuildConfig active_build_cfg_{};
  bool use_vt_reported_profile_{true};
  bool pool_source_configured_{false};
  struct AuxAssignmentState
  {
    std::uint16_t function_object_id{0xFFFF};
    std::uint8_t function_type_id{0xFF};
    std::uint64_t aux_input_name{0xFFFFFFFFFFFFFFFFULL};
    std::uint8_t aux_input_sa{0xFE};
    std::uint16_t aux_input_object_id{0xFFFF};
    bool assigned{false};
    bool store_as_preferred_assignment{false};
  };
  std::unordered_map<std::uint16_t, AuxAssignmentState> aux_assignments_by_function_id_{};
  std::unordered_map<std::uint64_t, std::uint16_t> aux_input_model_id_by_name_{};
  std::unordered_map<std::uint64_t, bool> aux_input_ready_by_name_{};
  std::vector<PreferredAssignmentEntry> preferred_assignments_{};
  bool preferred_assignment_pending_send_{false};
  bool preferred_assignment_inflight_{false};
  bool aux_preferred_store_override_{false};
  bool aux_n_support_{false};
};

}  // namespace ros2_isobus
