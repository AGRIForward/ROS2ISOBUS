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

#include "VTClient.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <thread>
#include <tuple>

#include "parser.h"

namespace ros2_isobus
{

namespace
{
// ISO 11783-6 / ISO 11783-7 protocol constants.
constexpr std::uint32_t kPgnVtToWorkingSet = 0x00E600;      // 58880
constexpr std::uint32_t kPgnWorkingSetToVt = 0x00E700;      // 59136
constexpr std::uint32_t kPgnWorkingSetMaster = 0x00FE0D;    // 65037
constexpr std::uint32_t kPgnWorkingSetMember = 0x00FE0C;    // 65036
constexpr std::uint8_t kTpTxStateCompleted = 1;
constexpr std::uint8_t kTpTxStateAborted = 2;
thread_local std::vector<std::uint8_t> * active_update_pool_bytes = nullptr;
thread_local bool update_parse_had_elements = false;

void update_parser_start_cb(void * data, char * el, const char ** attr)
{
  (void)data;
  (void)attr;
  if (el != nullptr) {
    update_parse_had_elements = true;
  }
}

void update_parser_end_cb(void * data, char * el)
{
  (void)data;
  (void)el;
}

void update_parser_ready_cb(char * data, int length)
{
  if (active_update_pool_bytes == nullptr || data == nullptr || length <= 0) {
    return;
  }
  active_update_pool_bytes->insert(active_update_pool_bytes->end(), data, data + length);
}

std::uint64_t swap_u64_bytes(std::uint64_t v)
{
  std::uint64_t out = 0;
  for (int i = 0; i < 8; ++i) {
    out = (out << 8) | (v & 0xFFu);
    v >>= 8;
  }
  return out;
}

}

VTClient::VTClient(std::uint8_t local_sa, std::uint8_t vt_sa)
: local_sa_(local_sa), vt_sa_(vt_sa)
{
}

void VTClient::set_session_timing(std::uint32_t timeout_ms, std::uint32_t max_retries)
{
  // Configure session state-machine timeout/retry policy.
  session_timeout_ms_ = timeout_ms;
  session_max_retries_ = max_retries;
}

void VTClient::configure_pool_source(const std::string & xml_path, const BuildConfig & fallback_cfg, bool use_vt_reported)
{
  // Configure XML pool source and fallback VT display profile.
  pool_xml_path_ = xml_path;
  fallback_cfg_ = fallback_cfg;
  active_build_cfg_ = fallback_cfg;
  use_vt_reported_profile_ = use_vt_reported;
  pool_source_configured_ = !pool_xml_path_.empty();
}

void VTClient::set_pool_identifiers(std::uint16_t working_set_id, std::uint16_t object_pool_id)
{
  // Set default WS/pool identifiers (can be overridden by parsed XML ids).
  vt_working_set_id_ = working_set_id;
  vt_object_pool_id_ = object_pool_id;
}

void VTClient::set_working_set_member_names(const std::vector<std::uint64_t> & member_names)
{
  // Define optional additional WS member NAME entries (PGN 65036).
  ws_member_names_ = member_names;
}

void VTClient::set_preferred_assignments(const std::vector<PreferredAssignmentEntry> & entries)
{
  preferred_assignments_ = entries;
  preferred_assignment_pending_send_ = !preferred_assignments_.empty();
}

std::vector<VTClient::PreferredAssignmentEntry> VTClient::preferred_assignments() const
{
  return preferred_assignments_;
}

bool VTClient::is_known_aux_source_sa(std::uint8_t sa)
{
  for (const auto & e : preferred_assignments_) {
    if (validate_aux_input_source(sa, e.aux_input_name)) {
      return true;
    }
  }
  for (const auto & kv : aux_assignments_by_function_id_) {
    if (validate_aux_input_source(sa, kv.second.aux_input_name)) {
      return true;
    }
  }
  return false;
}

void VTClient::try_send_preferred_assignment_command()
{
  if (!aux_n_support_ || !preferred_assignment_pending_send_ || preferred_assignment_inflight_) {
    return;
  }
  if (session_state_ != SessionState::Active) {
    return;
  }
  if (preferred_assignments_.empty()) {
    preferred_assignment_pending_send_ = false;
    return;
  }

  std::unordered_map<std::uint64_t, std::vector<std::pair<std::uint16_t, std::uint16_t>>> by_name;
  for (const auto & e : preferred_assignments_) {
    std::uint8_t resolved_sa = 0xFE;
    if (!validate_aux_input_name(e.aux_input_name, &resolved_sa)) {
      return;
    }
    const auto rit = aux_input_ready_by_name_.find(e.aux_input_name);
    if (rit == aux_input_ready_by_name_.end() || !rit->second) {
      return;
    }
    const auto mit = aux_input_model_id_by_name_.find(e.aux_input_name);
    if (mit == aux_input_model_id_by_name_.end()) {
      return;
    }
    by_name[e.aux_input_name].push_back({e.aux_function_object_id, e.aux_input_object_id});
  }

  std::vector<std::tuple<std::uint64_t, std::uint16_t, std::vector<std::pair<std::uint16_t, std::uint16_t>>>> groups;
  groups.reserve(by_name.size());
  for (const auto & kv : by_name) {
    const auto model_it = aux_input_model_id_by_name_.find(kv.first);
    if (model_it == aux_input_model_id_by_name_.end()) {
      continue;
    }
    groups.push_back(std::make_tuple(kv.first, model_it->second, kv.second));
  }
  if (groups.empty()) {
    return;
  }

  send_vt_command(
    VTProtocolCodec::Function::PreferredAssignmentCommand,
    codec_.build_preferred_assignment_command_args(groups));
  preferred_assignment_inflight_ = true;
  preferred_assignment_pending_send_ = false;
  printInfo(
    "VT preferred assignment command sent: input_units=" + std::to_string(groups.size()));
}


bool VTClient::load_pool_from_xml(const std::string & xml_path, const BuildConfig & cfg)
{
  // Load and normalize VT object pool model from XML.
  if (!pool_model_.load_from_xml(xml_path, cfg)) {
    printWarn("VT XML load/pool parse failed: " + xml_path);
    return false;
  }
  active_build_cfg_ = cfg;
  source_xml_declaration_ = "<?xml version=\"1.0\" encoding=\"ISO-8859-1\" standalone=\"no\"?>";
  source_objectpool_open_tag_.clear();
  {
    std::ifstream in(xml_path);
    if (in) {
      const std::string src(
        (std::istreambuf_iterator<char>(in)),
        std::istreambuf_iterator<char>());
      const auto decl_start = src.find("<?xml");
      if (decl_start != std::string::npos) {
        const auto decl_end = src.find("?>", decl_start);
        if (decl_end != std::string::npos) {
          source_xml_declaration_ = src.substr(decl_start, decl_end - decl_start + 2);
        }
      }
      const auto pool_start = src.find("<objectpool");
      if (pool_start != std::string::npos) {
        const auto pool_end = src.find('>', pool_start);
        if (pool_end != std::string::npos) {
          source_objectpool_open_tag_ = src.substr(pool_start, pool_end - pool_start + 1);
        }
      }
    }
  }

  // Prefer IDs parsed directly from XML object definitions.
  // These are the most reliable values for VT activate/request payload fields.
  if (pool_model_.working_set_id() != 0) {
    vt_working_set_id_ = pool_model_.working_set_id();
  }
  // Use first valid DataMask id from XML as default object pool id.
  for (const auto & m : pool_model_.data_masks()) {
    if (m.object_id != 0) {
      vt_object_pool_id_ = m.object_id;
      break;
    }
  }
  if (vt_working_set_id_ == 0 || vt_object_pool_id_ == 0) {
    printWarn("VT pool IDs unresolved from XML (ws_id or pool_id is 0), using fallback/default values");
  }

  printInfo(
    "VT pool parsed: " + std::to_string(pool_model_.pool_bytes().size()) + " bytes, " +
    std::to_string(pool_model_.bindings().size()) + " mapped elements, ws_id=0x" +
    [&]() {
      char b[8];
      std::snprintf(b, sizeof(b), "%04X", vt_working_set_id_);
      return std::string(b);
    }() +
    ", pool_id=0x" + [&]() {
      char b[8];
      std::snprintf(b, sizeof(b), "%04X", vt_object_pool_id_);
      return std::string(b);
    }());
  return !pool_model_.pool_bytes().empty();
}

void VTClient::send_pool_to_vt(std::uint64_t now_ms)
{
  // Send fn17 Object Pool Transfer.
  // Defer fn18 End Of Object Pool until TP transfer completion is reported by CanBridge.
  if (pool_model_.pool_bytes().empty()) {
    printWarn("VT pool not loaded, skip pool transfer");
    return;
  }
  if (!send_pool_transfer_message()) {
    printWarn("VT object pool transfer message build failed");
    return;
  }
  end_of_pool_send_pending_ = true;
  waiting_pool_tp_tx_complete_ = true;
  (void)now_ms;
}

void VTClient::notify_tp_tx_status(
  std::uint32_t pgn, std::uint8_t sa, std::uint8_t da, std::uint8_t state, std::uint64_t now_ms)
{
  if (!waiting_pool_tp_tx_complete_) {
    return;
  }
  if (pgn != kPgnWorkingSetToVt || sa != local_sa_ || da != vt_sa_) {
    return;
  }

  if (state == kTpTxStateCompleted) {
    waiting_pool_tp_tx_complete_ = false;
    if (end_of_pool_send_pending_) {
      send_end_of_object_pool_message();
      end_of_pool_send_pending_ = false;
      if (session_state_ == SessionState::WaitPoolTransferResponse) {
        session_deadline_ms_ = now_ms + session_timeout_ms_;
      }
    }
    return;
  }

  if (state == kTpTxStateAborted) {
    waiting_pool_tp_tx_complete_ = false;
    runtime_update_waiting_memory_ = false;
    runtime_update_pool_bytes_.clear();
    restart_or_fail(now_ms, "TP transfer aborted before EndOfObjectPool");
  }
}

void VTClient::request_vt_version()
{
  // Start VT capability negotiation with Get Hardware (fn C7).
  printInfo(
    "Sending VT version request to SA 0x" + [&]() {
      char b[8];
      std::snprintf(b, sizeof(b), "%02X", vt_sa_);
      return std::string(b);
    }());
  send_vt_command(VTProtocolCodec::Function::GetHardware);
}

void VTClient::request_vt_memory()
{
  // Request VT memory check for current object-pool size (fn C0).
  printInfo(
    "Sending VT memory request to SA 0x" + [&]() {
      char b[8];
      std::snprintf(b, sizeof(b), "%02X", vt_sa_);
      return std::string(b);
    }());
  const std::uint32_t required_bytes = static_cast<std::uint32_t>(pool_model_.pool_bytes().size());
  send_vt_command(
    VTProtocolCodec::Function::GetMemory,
    codec_.build_get_memory_args(required_bytes));
}

void VTClient::send_working_set_master_message(std::uint8_t member_count)
{
  msg::IsobusFrame fr;
  fr.priority = 6;
  fr.page = false;
  fr.pgn = kPgnWorkingSetMaster;
  fr.sa = local_sa_;
  fr.pf = static_cast<std::uint8_t>((fr.pgn >> 8) & 0xFF);
  fr.ps = static_cast<std::uint8_t>(fr.pgn & 0xFF);
  fr.data.fill(0xFF);
  // ISO 11783-7 Working Set Master (PGN 65037):
  // byte1=member count, remaining bytes reserved/default.
  fr.data[0] = member_count;
  send_frame(fr);
  printInfo(
    "WS Working Set Master sent: PGN 65037 (0xFE0D), member_count=" +
    std::to_string(member_count));
}

void VTClient::send_working_set_maintenance_message()
{
  // Destination-specific E700 maintenance requires a valid VT SA.
  if (vt_sa_ >= 0xFEu) {
    printWarn(
      "Skip WS maintenance (E700): invalid VT SA 0x" + to_hex_u8(vt_sa_));
    return;
  }

  msg::IsobusFrame fr;
  fr.priority = 6;
  fr.page = false;
  fr.pgn = kPgnWorkingSetToVt;
  fr.sa = local_sa_;
  fr.pf = static_cast<std::uint8_t>((fr.pgn >> 8) & 0xFF);
  fr.ps = vt_sa_;
  fr.data.fill(0xFF);
  // ISO 11783-6:2018 G.3 Working Set Maintenance (VT function channel):
  // byte1=function=0xFF, byte2 bit0=initiating (once), byte3=WS VT version.
  fr.data[0] = 0xFF;
  fr.data[1] = ws_initial_pending_ ? 0x01 : 0x00;
  fr.data[2] = ws_version_number_;
  const bool was_initial = ws_initial_pending_;
  ws_initial_pending_ = false;
  send_frame(fr);
  if (was_initial) {
    printInfo(
      "WS maintenance sent: PGN 59136 (E700), init_bit=1, version=" +
      std::to_string(fr.data[2]));
  }
}

void VTClient::send_working_set_member_messages()
{
  // Publish one PGN 65036 frame per configured WS member NAME.
  for (const auto name : ws_member_names_) {
    msg::IsobusFrame fr;
    fr.priority = 6;
    fr.page = false;
    fr.pgn = kPgnWorkingSetMember;
    fr.sa = local_sa_;
    fr.pf = static_cast<std::uint8_t>((fr.pgn >> 8) & 0xFF);
    fr.ps = static_cast<std::uint8_t>(fr.pgn & 0xFF);
    fr.data.fill(0xFF);
    // NAME in working set member PGN is transmitted LSB first.
    for (std::size_t i = 0; i < 8; ++i) {
      fr.data[i] = static_cast<std::uint8_t>((name >> (8 * i)) & 0xFFu);
    }
    send_frame(fr);
    printInfo("WS maintenance sent: PGN 65036 (Working Set Member)");
  }
}

void VTClient::send_numeric_value(std::uint16_t object_id, std::int32_t value)
{
  // Send Change Numeric Value command for mapped object id.
  send_vt_command(
    VTProtocolCodec::Function::ChangeNumericValue,
    codec_.build_change_numeric_value_args(object_id, value));
}

void VTClient::send_boolean_value(std::uint16_t object_id, bool value)
{
  // Send Change Numeric Value command in 1-byte object encoding for InputBoolean.
  send_vt_command(
    VTProtocolCodec::Function::ChangeNumericValue,
    codec_.build_change_numeric_value_u8_args(object_id, value ? 1u : 0u));
}

void VTClient::send_string_value(std::uint16_t object_id, const std::string & value)
{
  // Send Change String Value command for mapped object id.
  send_vt_command(
    VTProtocolCodec::Function::ChangeStringValue,
    codec_.build_change_string_value_args(object_id, value));
}

void VTClient::send_list_index_value(std::uint16_t object_id, std::uint8_t index)
{
  // Send Change List Item command for mapped object id.
  send_vt_command(
    VTProtocolCodec::Function::ChangeListItem,
    codec_.build_change_list_item_args(object_id, index));
}

void VTClient::send_visibility(std::uint16_t object_id, bool visible)
{
  // Send Hide/Show Object command.
  send_vt_command(
    VTProtocolCodec::Function::HideShowObject,
    codec_.build_hide_show_object_args(object_id, visible));
}

void VTClient::send_enable_disable_object(std::uint16_t object_id, bool enabled)
{
  // Send Enable/Disable Object command.
  send_vt_command(
    VTProtocolCodec::Function::EnableDisableObject,
    codec_.build_enable_disable_object_args(object_id, enabled));
}

void VTClient::send_select_input_object(std::uint16_t object_id)
{
  // Send Select Input Object command.
  send_vt_command(
    VTProtocolCodec::Function::SelectInputObjectCommand,
    codec_.build_select_input_object_args(object_id));
}

void VTClient::send_change_attribute(std::uint16_t object_id, std::uint8_t attribute_id, std::uint32_t value)
{
  // Send generic Change Attribute command with AID + value.
  send_vt_command(
    VTProtocolCodec::Function::ChangeAttribute,
    codec_.build_change_attribute_args(object_id, attribute_id, value));
}

void VTClient::send_change_child_position(
  std::uint16_t parent_object_id, std::uint16_t child_object_id, std::uint16_t x, std::uint16_t y)
{
  // Send child-object positioning command inside parent container.
  send_vt_command(
    VTProtocolCodec::Function::ChangeChildPosition,
    codec_.build_change_child_position_args(parent_object_id, child_object_id, x, y));
}

void VTClient::handle_vt_payload(
  const std::vector<std::uint8_t> & payload, std::uint64_t now_ms, std::uint8_t src_sa)
{
  // Main VT receive dispatcher for stateful session progression and events.
  if (payload.empty()) {
    return;
  }

  const std::uint8_t function = payload[0];
  if (aux_n_support_ &&
      function == static_cast<std::uint8_t>(VTProtocolCodec::Function::AuxiliaryInputMaintenanceType2)) {
    const auto mt = codec_.parse_aux_input_maintenance_type2(payload);
    if (!mt.valid) {
      printWarn("Malformed AUX Input Maintenance Type 2 message");
      return;
    }
    std::uint64_t aux_name = 0xFFFFFFFFFFFFFFFFULL;
    bool have_name = resolve_name_from_sa(src_sa, &aux_name);
    if (!have_name || !validate_aux_input_source(src_sa, aux_name)) {
      // Fallback: resolve NAME by matching source SA against known assignment/preferred NAMEs.
      for (const auto & e : preferred_assignments_) {
        if (validate_aux_input_source(src_sa, e.aux_input_name)) {
          aux_name = e.aux_input_name;
          have_name = true;
          break;
        }
      }
      if (!have_name) {
        for (const auto & kv : aux_assignments_by_function_id_) {
          if (validate_aux_input_source(src_sa, kv.second.aux_input_name)) {
            aux_name = kv.second.aux_input_name;
            have_name = true;
            break;
          }
        }
      }
    }
    if (!have_name) {
      return;
    }
    // Normalize NAME endianness against known preferred/assignment mappings.
    // Some address-book sources expose NAME byte order differently than AUX assignment payloads.
    const auto aux_name_swapped = swap_u64_bytes(aux_name);
    const bool aux_name_known =
      std::any_of(
      preferred_assignments_.begin(), preferred_assignments_.end(),
      [&](const PreferredAssignmentEntry & e) {return e.aux_input_name == aux_name;}) ||
      std::any_of(
      aux_assignments_by_function_id_.begin(), aux_assignments_by_function_id_.end(),
      [&](const auto & kv) {return kv.second.aux_input_name == aux_name;});
    if (!aux_name_known) {
      const bool swapped_known =
        std::any_of(
        preferred_assignments_.begin(), preferred_assignments_.end(),
        [&](const PreferredAssignmentEntry & e) {return e.aux_input_name == aux_name_swapped;}) ||
        std::any_of(
        aux_assignments_by_function_id_.begin(), aux_assignments_by_function_id_.end(),
        [&](const auto & kv) {return kv.second.aux_input_name == aux_name_swapped;});
      if (swapped_known) {
        aux_name = aux_name_swapped;
      }
    }
    aux_input_model_id_by_name_[aux_name] = mt.model_identification_code;
    aux_input_ready_by_name_[aux_name] = (mt.status == 0x01u);
    try_send_preferred_assignment_command();
    return;
  }

  if (aux_n_support_ &&
      function == static_cast<std::uint8_t>(VTProtocolCodec::Function::PreferredAssignmentResponse)) {
    const auto pr = codec_.parse_preferred_assignment_response(payload);
    if (!pr.valid) {
      printWarn("Malformed Preferred Assignment response");
      return;
    }
    preferred_assignment_inflight_ = false;
    if (pr.error_code_bits == 0x00u) {
      printInfo("VT preferred assignment response accepted");
    } else {
      char b[8];
      std::snprintf(b, sizeof(b), "%04X", pr.faulty_aux_function_object_id);
      printWarn(
        "VT preferred assignment response error_bits=0x" + to_hex_u8(pr.error_code_bits) +
        ", faulty_function_id=0x" + std::string(b));
    }
    return;
  }

  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::VtStatus)) {
    const auto st = codec_.parse_vt_status(payload);
    if (!st.valid) {
      printWarn("Malformed VT status payload");
      return;
    }
    on_vt_status(st);
    const bool status_changed =
      !vt_status_prev_valid_ ||
      st.active_ws_sa != vt_status_prev_active_ws_sa_ ||
      st.visible_data_alarm_mask_id != vt_status_prev_data_mask_id_ ||
      st.visible_soft_key_mask_id != vt_status_prev_soft_key_mask_id_ ||
      st.busy_codes != vt_status_prev_busy_codes_ ||
      st.current_command_function != vt_status_prev_current_fn_;
    if (status_changed) {
      printInfo(
        "VT status: active_ws_sa=0x" + to_hex_u8(st.active_ws_sa) +
        ", data_mask_id=0x" + [&]() {
          char b[8];
          std::snprintf(b, sizeof(b), "%04X", st.visible_data_alarm_mask_id);
          return std::string(b);
        }() +
        ", soft_key_mask_id=0x" + [&]() {
          char b[8];
          std::snprintf(b, sizeof(b), "%04X", st.visible_soft_key_mask_id);
          return std::string(b);
        }() +
        ", busy=0x" + to_hex_u8(st.busy_codes) +
        ", current_fn=0x" + to_hex_u8(st.current_command_function));
      vt_status_prev_valid_ = true;
      vt_status_prev_active_ws_sa_ = st.active_ws_sa;
      vt_status_prev_data_mask_id_ = st.visible_data_alarm_mask_id;
      vt_status_prev_soft_key_mask_id_ = st.visible_soft_key_mask_id;
      vt_status_prev_busy_codes_ = st.busy_codes;
      vt_status_prev_current_fn_ = st.current_command_function;
    }
    report_vt_status_issues(st);

    if (session_state_ == SessionState::Idle ||
      session_state_ == SessionState::WaitVersionResponse ||
      session_state_ == SessionState::WaitMemoryResponse ||
      session_state_ == SessionState::WaitPoolActivateResponse ||
      session_state_ == SessionState::Active) {
      try_send_preferred_assignment_command();
      // VT status is periodic keepalive; ignore outside transfer-wait path.
      return;
    }
    if (session_state_ != SessionState::WaitPoolTransferResponse) {
      printWarn(std::string("Received VT status in unexpected state: ") + session_state_name(session_state_));
      return;
    }
    if (!pool_transfer_response_received_) {
      return;
    }
    // ISO 11783-6 Annex C.2.2 c.2:
    // After EndOfPool, observe three consecutive VT Status messages where
    // parsing bit (byte 7 bit 4) is 0 before concluding transfer completed.
    const bool parsing_active = st.parsing_active;
    if (parsing_active) {
      vt_status_parsing_clear_count_ = 0;
      session_deadline_ms_ = now_ms + session_timeout_ms_;
      printInfo(
        "VT parsing status debug: parsing_active=1 -> clear_count reset to 0");
    } else if (vt_status_parsing_clear_count_ < 3u) {
      ++vt_status_parsing_clear_count_;
      session_deadline_ms_ = now_ms + session_timeout_ms_;
      printInfo(
        "VT parsing status debug: parsing_active=0 -> clear_count=" +
        std::to_string(vt_status_parsing_clear_count_) + "/3");
    } else {
      printInfo("VT parsing status debug: parsing_active=0 -> clear_count already 3/3");
    }

    if (vt_status_parsing_clear_count_ >= 3u) {
      transition_to(SessionState::WaitPoolActivateResponse, now_ms, "VT parsing complete (3x VT status)");
      // ISO 11783-6 activation path: Change Active Mask (F.34/F.35), not fn A0.
      send_change_active_mask_command();
    }
    return;
  }

  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::GetHardware)) {
    if (session_state_ != SessionState::WaitVersionResponse) {
      printWarn(
        std::string("Received VT version/get-hardware message in unexpected state: ") +
        session_state_name(session_state_));
      return;
    }
    const auto vr = codec_.parse_version_response(payload);
    if (!vr.valid) {
      restart_or_fail(now_ms, "Malformed VT version response");
      return;
    }
    printInfo(
      "VT get-hardware response: boot_time_s=" + std::to_string(vr.boot_time_s) +
      ", graphic_type=" + std::to_string(vr.graphic_type) +
      ", hardware_bits=0x" + to_hex_u8(vr.hardware_bits) +
      " (" + hardware_bits_text(vr.hardware_bits) + ")" +
      ", x_pixels=" + std::to_string(vr.x_pixels) +
      ", y_pixels=" + std::to_string(vr.y_pixels));
    // Build/reconfigure pool after logging received VT version payload.
    ensure_pool_loaded_from_version_payload(payload);
    transition_to(SessionState::RequestMemory, now_ms, "VT version response");
    request_vt_memory();
    transition_to(SessionState::WaitMemoryResponse, now_ms, "Memory request sent");
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::DeleteObjectPool)) {
    if (payload.size() < 2) {
      printWarn("Malformed VT Delete Object Pool response");
      return;
    }
    const auto err = payload[1];
    if (err == 0x00) {
      printInfo("VT Delete Object Pool response: success");
    } else {
      printWarn(
        "VT Delete Object Pool response error=0x" + to_hex_u8(err));
    }
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::GetMemory)) {
    const bool runtime_memory_response = (runtime_update_waiting_memory_ && session_state_ == SessionState::Active);
    if (session_state_ != SessionState::WaitMemoryResponse && !runtime_memory_response) {
      printWarn(
        std::string("Received VT memory response in unexpected state: ") +
        session_state_name(session_state_));
      return;
    }
    if (payload.size() < 2) {
      restart_or_fail(now_ms, "Malformed VT memory response");
      return;
    }
    const auto mr = codec_.parse_memory_response(payload);
    if (!mr.valid) {
      restart_or_fail(now_ms, "Malformed VT memory response");
      return;
    }
    printInfo(
      "VT memory response: vt_version=" + std::to_string(mr.vt_version) +
      ", status=0x" + to_hex_u8(mr.status) +
      " (" + std::string(memory_status_text(mr.status)) + ")" +
      ", value_le=" + std::to_string(mr.value_le) +
      ", payload_size=" + std::to_string(mr.payload_size));
    if (payload.size() >= 8) {
      bool reserved_ok = true;
      for (std::size_t i = 3; i < 8; ++i) {
        if (payload[i] != 0xFFu) {
          reserved_ok = false;
          break;
        }
      }
      if (!reserved_ok) {
        printWarn("VT memory response reserved bytes (4..8) are not 0xFF");
      }
    }
    if (mr.status != 0x00) {
      if (runtime_memory_response) {
        runtime_update_waiting_memory_ = false;
        runtime_update_pool_bytes_.clear();
        printWarn("VT rejected runtime pool update due to memory status");
        report_runtime_update_result(false, "memory_rejected");
        dispatch_next_pending_runtime_update();
        return;
      }
      restart_or_fail(now_ms, "VT reports insufficient/invalid memory status");
      return;
    }
    if (runtime_memory_response) {
      runtime_update_waiting_memory_ = false;
      if (runtime_update_pool_bytes_.empty()) {
        printWarn("Runtime pool update aborted: no staged pool bytes after memory response");
        report_runtime_update_result(false, "no_staged_pool_bytes");
        dispatch_next_pending_runtime_update();
        return;
      }
      send_vt_command(VTProtocolCodec::Function::ObjectPoolTransfer, runtime_update_pool_bytes_);
      runtime_update_eop_inflight_++;
      end_of_pool_send_pending_ = true;
      waiting_pool_tp_tx_complete_ = true;
      printInfo(
        "VT runtime update pool transfer started (bytes=" +
        std::to_string(runtime_update_pool_bytes_.size()) + ")");
      runtime_update_pool_bytes_.clear();
      return;
    }
    transition_to(SessionState::WaitPoolTransferResponse, now_ms, "VT memory response");
    send_pool_to_vt(now_ms);
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::EndOfObjectPool)) {
    const bool runtime_update_response = (runtime_update_eop_inflight_ > 0);
    if (session_state_ != SessionState::WaitPoolTransferResponse && !runtime_update_response) {
      printWarn(
        std::string("Received VT EndOfPool response in unexpected state: ") +
        session_state_name(session_state_));
      return;
    }
    const auto tr = codec_.parse_pool_transfer_response(payload);
    if (!tr.valid) {
      if (runtime_update_response) {
        --runtime_update_eop_inflight_;
        printWarn("Malformed VT EndOfPool response for runtime update");
        report_runtime_update_result(false, "malformed_end_of_pool_response");
        dispatch_next_pending_runtime_update();
        return;
      }
      restart_or_fail(now_ms, "Malformed VT EndOfPool response");
      return;
    }
    printInfo(
      "VT EndOfPool response: error=0x" + to_hex_u8(tr.error_codes) +
      ", parent_id=0x" + [&]() {
        char b[8];
        std::snprintf(b, sizeof(b), "%04X", tr.parent_object_id);
        return std::string(b);
      }() +
      ", object_id=0x" + [&]() {
        char b[8];
        std::snprintf(b, sizeof(b), "%04X", tr.object_id);
        return std::string(b);
      }() +
      ", object_pool_error=0x" + to_hex_u8(tr.object_pool_error_codes));
    if (tr.reserved != 0xFFu) {
      printWarn("VT EndOfPool response reserved byte 8 is not 0xFF");
    }
    if ((tr.error_codes & 0xECu) != 0u) {
      printWarn("VT EndOfPool response has reserved bits set in byte 2");
    }
    if ((tr.object_pool_error_codes & 0xF0u) != 0u) {
      printWarn("VT EndOfPool response has reserved bits set in byte 7");
    }
    if (tr.error_codes != 0x00 || tr.object_pool_error_codes != 0x00) {
      if (runtime_update_response) {
        --runtime_update_eop_inflight_;
        printWarn("VT rejected runtime pool update");
        report_runtime_update_result(false, "vt_rejected_runtime_update");
        dispatch_next_pending_runtime_update();
        return;
      }
      restart_or_fail(now_ms, "VT rejected object pool");
      return;
    }
    if (runtime_update_response) {
      --runtime_update_eop_inflight_;
      printInfo("VT EndOfPool response accepted for runtime pool update");
      report_runtime_update_result(true, "ok");
      dispatch_next_pending_runtime_update();
      return;
    }
    pool_transfer_response_received_ = true;
    end_of_pool_retry_count_ = 0;
    vt_status_parsing_clear_count_ = 0;  // Reset parsing complete counter
    printInfo("VT EndOfPool response accepted, waiting for parsing completion (3x VT status with parsing_active=0)");
    // Remain in WaitPoolTransferResponse state to collect 3x VT Status messages
    // with parsing_active=0 before sending Change Active Mask (ISO 11783-6 C.2.2/C.2.5)
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::ChangeActiveMask)) {
    if (payload.size() < 4) {
      printWarn("Malformed VT ChangeActiveMask response");
      return;
    }
    const auto mask_id = codec_.read_u16_le(payload, 1);
    const auto err = payload[3];
    on_change_active_mask_result(mask_id, err);
    if (err == 0x00) {
      printInfo(
        "VT ChangeActiveMask response ok: mask_id=0x" + [&]() {
          char b[8];
          std::snprintf(b, sizeof(b), "%04X", mask_id);
          return std::string(b);
        }());
      if (session_state_ == SessionState::WaitPoolActivateResponse) {
        session_retry_count_ = 0;
        transition_to(SessionState::Active, now_ms, "VT active mask accepted");
        try_send_preferred_assignment_command();
      }
      return;
    }
    std::string e;
    if (err & 0x01u) e += "invalid_ws_id,";
    if (err & 0x02u) e += "invalid_mask_id,";
    if (err & 0x10u) e += "other_error,";
    if (!e.empty() && e.back() == ',') e.pop_back();
    if (e.empty()) e = "unknown";
    printWarn(
      "VT ChangeActiveMask response error=0x" + to_hex_u8(err) +
      " (" + e + ")");
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::ChangeSoftKeyMask)) {
    if (payload.size() < 4) {
      printWarn("Malformed VT ChangeSoftKeyMask response");
      return;
    }
    const auto mask_id = codec_.read_u16_le(payload, 1);
    const auto err = payload[3];
    on_change_soft_key_mask_result(mask_id, err);
    if (err == 0x00) {
      printInfo(
        "VT ChangeSoftKeyMask response ok: mask_id=0x" + [&]() {
          char b[8];
          std::snprintf(b, sizeof(b), "%04X", mask_id);
          return std::string(b);
        }());
      return;
    }
    printWarn(
      "VT ChangeSoftKeyMask response error=0x" + to_hex_u8(err));
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::HideShowObject)) {
    // Handle both observed payload variants for fn160:
    // 1) state echo: bytes2..3 object id, byte4 show/hide (0=hide,1=show)
    // 2) result:     bytes2..3 object id, byte4 error code
    if (payload.size() < 4) {
      printWarn("Malformed VT HideShowObject response");
      return;
    }
    const auto object_id = codec_.read_u16_le(payload, 1);
    const auto b3 = payload[3];
    if (b3 == 0x00u || b3 == 0x01u) {
      on_hide_show_object_state(object_id, b3 == 0x01u);
      return;
    }
    const auto err = b3;
    on_hide_show_object_result(object_id, err);
    if (err != 0x00) {
      printWarn(
        "VT HideShowObject response error=0x" + to_hex_u8(err) +
        ", object_id=0x" + [&]() {
          char b[8];
          std::snprintf(b, sizeof(b), "%04X", object_id);
          return std::string(b);
        }());
    }
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::EnableDisableObject)) {
    if (payload.size() < 4) {
      printWarn("Malformed VT EnableDisableObject response");
      return;
    }
    const auto object_id = codec_.read_u16_le(payload, 1);
    const auto err = payload[3];
    on_enable_disable_object_result(object_id, err);
    if (err != 0x00) {
      printWarn(
        "VT EnableDisableObject response error=0x" + to_hex_u8(err) +
        ", object_id=0x" + [&]() {
          char b[8];
          std::snprintf(b, sizeof(b), "%04X", object_id);
          return std::string(b);
        }());
    }
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::SelectInputObjectCommand)) {
    if (payload.size() < 4) {
      printWarn("Malformed VT SelectInputObject response");
      return;
    }
    const auto object_id = codec_.read_u16_le(payload, 1);
    const auto err = payload[3];
    on_select_input_object_result(object_id, err);
    if (err != 0x00) {
      printWarn(
        "VT SelectInputObject response error=0x" + to_hex_u8(err) +
        ", object_id=0x" + [&]() {
          char b[8];
          std::snprintf(b, sizeof(b), "%04X", object_id);
          return std::string(b);
        }());
    }
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::ChangeStringValue)) {
    // ISO 11783-6: fn179 response:
    // bytes2..3 reserved, bytes4..5 object id, byte6 error code.
    if (payload.size() < 6) {
      printWarn("Malformed VT ChangeStringValue response");
      return;
    }
    const auto object_id = codec_.read_u16_le(payload, 3);
    const auto err = payload[5];
    on_change_string_value_result(object_id, err);
    if (err != 0x00) {
      printWarn(
        "VT ChangeStringValue response error=0x" + to_hex_u8(err) +
        ", object_id=0x" + [&]() {
          char b[8];
          std::snprintf(b, sizeof(b), "%04X", object_id);
          return std::string(b);
        }());
    }
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::ChangeListItem)) {
    // ISO 11783-6: fn177 response:
    // bytes2..3 object id, byte4 list index, bytes5..6 child id, byte7 error.
    if (payload.size() < 7) {
      printWarn("Malformed VT ChangeListItem response");
      return;
    }
    const auto object_id = codec_.read_u16_le(payload, 1);
    const auto list_index = payload[3];
    const auto err = payload[6];
    on_change_list_item_result(object_id, list_index, err);
    if (err != 0x00) {
      printWarn(
        "VT ChangeListItem response error=0x" + to_hex_u8(err) +
        ", object_id=0x" + [&]() {
          char b[8];
          std::snprintf(b, sizeof(b), "%04X", object_id);
          return std::string(b);
        }() +
        ", list_index=" + std::to_string(list_index));
    }
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::PointingEvent)) {
    const auto pe = codec_.parse_pointing_event(payload);
    if (!pe.valid) {
      printWarn("Malformed VT pointing event");
      return;
    }
    on_pointing_event(pe);
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::SelectInputObjectEvent) || function == static_cast<std::uint8_t>(VTProtocolCodec::Function::Esc)) {
    const auto ne = codec_.parse_navigation_event(payload);
    if (!ne.valid) {
      printWarn("Malformed VT navigation event");
      return;
    }
    on_navigation_event(ne);
    return;
  }

  if (aux_n_support_ &&
      function == static_cast<std::uint8_t>(VTProtocolCodec::Function::AuxiliaryAssignmentType2)) {
    const auto cmd = codec_.parse_aux_assignment_type2_command(payload);
    if (!cmd.valid) {
      printWarn("Malformed AUX Assignment Type 2 command");
      return;
    }

    std::uint8_t error_code_bits = 0x00u;
    std::uint8_t resolved_aux_sa = 0xFE;
    bool name_not_found = false;
    bool function_type_mismatch = false;
    if (!cmd.remove_assignment &&
        !validate_aux_input_name(cmd.aux_input_name, &resolved_aux_sa)) {
      error_code_bits |= 0x01u;  // assignment rejected: AUX input NAME not known
      name_not_found = true;
    }
    const auto it = pool_model_.by_id().find(cmd.aux_function_object_id);
    if (it == pool_model_.by_id().end()) {
      error_code_bits |= 0x02u;  // function object id invalid
    } else {
      const auto & binding = pool_model_.bindings()[it->second];
      if (binding.kind != ObjectKind::AuxFunction) {
        error_code_bits |= 0x02u;
      } else {
        if (!cmd.remove_assignment && binding.aux_function_type_id != 0xFFu &&
            binding.aux_function_type_id != cmd.assigned_input_function_type) {
          error_code_bits |= 0x01u;  // assignment rejected
          function_type_mismatch = true;
        }
        if (error_code_bits == 0x00u) {
          AuxAssignmentState st{};
          st.function_object_id = binding.object_id;
          st.function_type_id = binding.aux_function_type_id;
          st.aux_input_name = cmd.aux_input_name;
          st.aux_input_sa = resolved_aux_sa;
          st.aux_input_object_id = cmd.aux_input_object_id;
          st.store_as_preferred_assignment = cmd.store_as_preferred_assignment;
          st.assigned = !cmd.remove_assignment;
          aux_assignments_by_function_id_[binding.object_id] = st;
          if (cmd.store_as_preferred_assignment || aux_preferred_store_override_) {
            preferred_assignments_.erase(
              std::remove_if(
                preferred_assignments_.begin(), preferred_assignments_.end(),
                [&](const PreferredAssignmentEntry & e) {
                  return e.aux_function_object_id == binding.object_id;
                }),
              preferred_assignments_.end());
            if (!cmd.remove_assignment) {
              preferred_assignments_.push_back(
                PreferredAssignmentEntry{
                  cmd.aux_input_name,
                  cmd.aux_input_object_id,
                  binding.object_id});
            }
            on_preferred_assignments_changed(preferred_assignments_);
          }
          printInfo(
            "AUX assignment mapped: function_id=0x" + [&]() {
              char b[8];
              std::snprintf(b, sizeof(b), "%04X", binding.object_id);
              return std::string(b);
            }() +
            ", input_id=0x" + [&]() {
              char b[8];
              std::snprintf(b, sizeof(b), "%04X", cmd.aux_input_object_id);
              return std::string(b);
            }() +
            ", aux_name=0x" + [&]() {
              char b[24];
              std::snprintf(b, sizeof(b), "%016llX", static_cast<unsigned long long>(cmd.aux_input_name));
              return std::string(b);
            }() +
            ", resolved_sa=0x" + to_hex_u8(resolved_aux_sa));
          on_aux_assignment_value(binding, cmd);
        }
        on_aux_assignment_result(
          binding, error_code_bits == 0x00u, error_code_bits,
          (error_code_bits == 0x00u) ? "accepted" : "rejected");
        if (error_code_bits != 0x00u) {
          std::string reason;
          if (name_not_found) {
            reason += "name_not_found;";
          }
          if (function_type_mismatch) {
            reason += "function_type_mismatch;";
          }
          if (reason.empty()) {
            reason = "validation_error;";
          }
          printWarn(
            "AUX Assignment rejected: function_id=0x" + [&]() {
              char b[8];
              std::snprintf(b, sizeof(b), "%04X", cmd.aux_function_object_id);
              return std::string(b);
            }() +
            ", input_id=0x" + [&]() {
              char b[8];
              std::snprintf(b, sizeof(b), "%04X", cmd.aux_input_object_id);
              return std::string(b);
            }() +
            ", src_sa=0x" + to_hex_u8(src_sa) +
            ", resolved_sa=0x" + to_hex_u8(resolved_aux_sa) +
            ", error_bits=0x" + to_hex_u8(error_code_bits) +
            ", reason=" + reason);
        }
      }
    }

    send_vt_command(
      VTProtocolCodec::Function::AuxiliaryAssignmentType2,
      codec_.build_aux_assignment_type2_response_args(cmd.aux_function_object_id, error_code_bits));
    return;
  }

  if (aux_n_support_ &&
      function == static_cast<std::uint8_t>(VTProtocolCodec::Function::AuxiliaryInputStatusType2)) {
    const auto st = codec_.parse_aux_input_status_type2(payload);
    if (!st.valid) {
      printWarn("Malformed AUX Input Type 2 status");
      return;
    }

    bool any_matched = false;
    bool saw_name_lookup_miss = false;
    bool saw_name_sa_mismatch = false;
    for (const auto & kv : aux_assignments_by_function_id_) {
      const auto & asn = kv.second;
      if (!asn.assigned) {
        continue;
      }
      // Resolve source SA to expected AUX NAME and match against assignment mapping.
      if (!validate_aux_input_source(src_sa, asn.aux_input_name)) {
        std::uint8_t resolved_sa = 0xFE;
        if (!validate_aux_input_name(asn.aux_input_name, &resolved_sa)) {
          saw_name_lookup_miss = true;
        } else {
          saw_name_sa_mismatch = true;
        }
        continue;
      }
      if (asn.aux_input_object_id != st.input_object_id) {
        continue;
      }
      const auto fit = pool_model_.by_id().find(asn.function_object_id);
      if (fit == pool_model_.by_id().end()) {
        continue;
      }
      const auto & binding = pool_model_.bindings()[fit->second];
      if (binding.kind != ObjectKind::AuxFunction) {
        continue;
      }
      if (st.learn_mode_active) {
        continue;
      }
      const double normalized = normalize_aux_value(
        (binding.aux_function_type_id != 0xFFu) ? binding.aux_function_type_id : asn.function_type_id,
        st.value1, st.value2);
      any_matched = true;
      on_aux_input_status(binding, st, normalized);
    }
    if (!any_matched) {
      if (saw_name_lookup_miss) {
        printWarn(
          "AUX input status dropped: source SA 0x" + to_hex_u8(src_sa) +
          " could not be mapped to assignment NAME from address book");
      } else if (saw_name_sa_mismatch) {
        printWarn(
          "AUX input status dropped: source SA 0x" + to_hex_u8(src_sa) +
          " does not match assignment NAME->SA mapping");
      } else {
        printWarn(
          "AUX input status dropped: no assignment match for input_id=0x" + [&]() {
            char b[8];
            std::snprintf(b, sizeof(b), "%04X", st.input_object_id);
            return std::string(b);
          }());
      }
    }
    return;
  }

  if (payload.size() < 3) {
    if (function != static_cast<std::uint8_t>(VTProtocolCodec::Function::SoftKeyActivation) &&
        function != static_cast<std::uint8_t>(VTProtocolCodec::Function::ButtonActivation) &&
        function != static_cast<std::uint8_t>(VTProtocolCodec::Function::ChangeNumericValue) &&
        function != static_cast<std::uint8_t>(VTProtocolCodec::Function::VtChangeNumericValue) &&
        function != static_cast<std::uint8_t>(VTProtocolCodec::Function::VtChangeStringValue) &&
        warned_unparsed_functions_.insert(function).second) {
      printWarn(
        "Unhandled VT function 0x" + to_hex_u8(function) +
        " received (payload_len=" + std::to_string(payload.size()) + ")");
    }
    return;
  }

  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::SoftKeyActivation)) {
    // ISO 11783-6 softkey activation event carries object id at bytes 3..4.
    // Keep fallback to bytes 2..3 for interoperability with non-conforming VT traces.
    std::uint16_t object_id = 0;
    if (payload.size() >= 4) {
      object_id = static_cast<std::uint16_t>(payload[2]) |
        (static_cast<std::uint16_t>(payload[3]) << 8);
    } else {
      object_id = static_cast<std::uint16_t>(payload[1]) |
        (static_cast<std::uint16_t>(payload[2]) << 8);
    }
    const auto it = pool_model_.by_id().find(object_id);
    if (it == pool_model_.by_id().end()) {
      return;
    }
    const auto & binding = pool_model_.bindings()[it->second];
    const std::uint8_t activation_code = (payload.size() >= 2) ? payload[1] : 0x01u;
    const bool pressed = (activation_code != 0x00u);
    on_softkey_event(binding, pressed, activation_code);
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::ButtonActivation)) {
    // ISO 11783-6 button activation event carries object id at bytes 3..4.
    std::uint16_t object_id = 0;
    if (payload.size() >= 4) {
      object_id = static_cast<std::uint16_t>(payload[2]) |
        (static_cast<std::uint16_t>(payload[3]) << 8);
    } else {
      object_id = static_cast<std::uint16_t>(payload[1]) |
        (static_cast<std::uint16_t>(payload[2]) << 8);
    }
    const auto it = pool_model_.by_id().find(object_id);
    if (it == pool_model_.by_id().end()) {
      return;
    }
    const auto & binding = pool_model_.bindings()[it->second];
    const std::uint8_t activation_code = (payload.size() >= 2) ? payload[1] : 0x01u;
    const bool pressed = (activation_code != 0x00u);
    on_button_event(binding, pressed, activation_code);
    return;
  }

  const std::uint16_t object_id = static_cast<std::uint16_t>(payload[1]) |
    (static_cast<std::uint16_t>(payload[2]) << 8);
  const auto it = pool_model_.by_id().find(object_id);
  if (it == pool_model_.by_id().end()) {
    return;
  }
  const auto & binding = pool_model_.bindings()[it->second];

  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::VtChangeNumericValue) &&
      payload.size() >= 8) {
    std::int32_t value = 0;
    switch (binding.kind) {
      case ObjectKind::InputBoolean:
      case ObjectKind::InputList:
      case ObjectKind::OutputList:
        value = static_cast<std::int32_t>(payload[4]);
        break;
      default:
        value = static_cast<std::int32_t>(static_cast<std::uint32_t>(payload[4]) |
          (static_cast<std::uint32_t>(payload[5]) << 8) |
          (static_cast<std::uint32_t>(payload[6]) << 16) |
          (static_cast<std::uint32_t>(payload[7]) << 24));
        break;
    }
    on_numeric_event(binding, value);
    return;
  }
  if (function == static_cast<std::uint8_t>(VTProtocolCodec::Function::VtChangeStringValue) &&
      payload.size() >= 4) {
    // VTChangeStringValue (VT->ECU) fixed format:
    // [fn=0x08][obj_lo][obj_hi][len_u8][chars...]
    const std::size_t data_offset = 4;
    const std::uint8_t length = payload[3];
    const std::size_t available = payload.size() - data_offset;
    const std::size_t used = std::min<std::size_t>(length, available);
    std::string value;
    value.reserve(used);
    for (std::size_t i = 0; i < used; ++i) {
      const auto c = payload[data_offset + i];
      if (c == 0x00u || c == 0xFFu) {
        break;
      }
      value.push_back(static_cast<char>(c));
    }
    on_string_event(binding, value);
    return;
  }

  if (function != static_cast<std::uint8_t>(VTProtocolCodec::Function::SoftKeyActivation) &&
      function != static_cast<std::uint8_t>(VTProtocolCodec::Function::ButtonActivation) &&
      function != static_cast<std::uint8_t>(VTProtocolCodec::Function::ChangeNumericValue) &&
      function != static_cast<std::uint8_t>(VTProtocolCodec::Function::VtChangeNumericValue) &&
      function != static_cast<std::uint8_t>(VTProtocolCodec::Function::VtChangeStringValue) &&
      warned_unparsed_functions_.insert(function).second) {
    printWarn(
      "Unhandled VT function 0x" + to_hex_u8(function) +
      " received (payload_len=" + std::to_string(payload.size()) + ")");
  }
}

double VTClient::normalize_aux_value(std::uint8_t type_id, std::uint16_t value1, std::uint16_t value2)
{
  (void)value2;
  // Coarse normalization for application-level abstraction.
  // Keep raw fields available via dedicated AUX raw topic.
  if (type_id == 0u || type_id == 2u || type_id == 5u || type_id == 6u ||
      type_id == 7u || type_id == 8u || type_id == 11u) {
    return (value1 == 0u) ? 0.0 : 1.0;
  }
  if (type_id == 14u) {
    return static_cast<double>(value1);
  }
  if (value1 <= 0xFAFFu) {
    return (static_cast<double>(value1) * 100.0) / 64255.0;
  }
  return static_cast<double>(value1);
}

void VTClient::report_vt_status_issues(const VtStatus & st)
{
  if (!st.valid) return;
  if (st.out_of_memory) {
    printWarn("VT status indicates VT out of memory (byte7 bit7 set)");
  }
  if (st.busy_saving_to_nonvolatile) {
    printInfo("VT status: busy saving to non-volatile memory");
  }
  if (st.busy_executing_command || st.busy_executing_macro) {
    printInfo(
      "VT status: executing " +
      std::string(st.busy_executing_macro ? "macro" : "command") +
      ", function=0x" + to_hex_u8(st.current_command_function));
  }
  if (st.aux_learn_mode_active) {
    printInfo("VT status: AUX learn mode active");
  }
}

void VTClient::start_session(std::uint64_t now_ms)
{
  if (session_state_ == SessionState::Idle || session_state_ == SessionState::Active) {
  session_retry_count_ = 0;
  preferred_assignment_inflight_ = false;
  if (!preferred_assignments_.empty()) {
    preferred_assignment_pending_send_ = true;
  }
  }
  // Working Set initialization is owned by VTClientNode on AddressManager
  // ADDRESS_CLAIMED status transitions. Do not re-send WS init from here.

  pool_transfer_response_received_ = false;
  vt_status_parsing_clear_count_ = 0;
  warned_unparsed_functions_.clear();
  missing_fn18_wait_warned_ = false;
  pool_transfer_wait_started_ms_ = 0;
  end_of_pool_retry_count_ = 0;
  end_of_pool_send_pending_ = false;
  waiting_pool_tp_tx_complete_ = false;
  runtime_update_eop_inflight_ = 0;
  runtime_update_waiting_memory_ = false;
  runtime_update_pool_bytes_.clear();
  printInfo(
    "VT session start config: ws_id=0x" + [&]() {
      char b[8];
      std::snprintf(b, sizeof(b), "%04X", vt_working_set_id_);
      return std::string(b);
    }() +
    ", pool_id=0x" + [&]() {
      char b[8];
      std::snprintf(b, sizeof(b), "%04X", vt_object_pool_id_);
      return std::string(b);
    }() +
    ", pool_ready=" + (pool_model_.pool_bytes().empty() ? std::string("false") : std::string("true")));
  transition_to(SessionState::RequestVersion, now_ms, "Session start");
  request_vt_version();
  transition_to(SessionState::WaitVersionResponse, now_ms, "Version request sent");
}

void VTClient::tick_session(std::uint64_t now_ms)
{
  if (session_state_ == SessionState::Active || session_state_ == SessionState::Idle ||
      session_state_ == SessionState::Error) {
    return;
  }
  if (now_ms < session_deadline_ms_) {
    return;
  }

  switch (session_state_) {
    case SessionState::WaitVersionResponse:
      restart_or_fail(now_ms, "Timeout waiting VT version response");
      return;
    case SessionState::WaitMemoryResponse:
      restart_or_fail(now_ms, "Timeout waiting VT memory response");
      return;
    case SessionState::WaitPoolTransferResponse:
      if (end_of_pool_send_pending_) {
        printWarn(
          "Timeout waiting TP transfer completion status; sending EndOfObjectPool fallback");
        send_end_of_object_pool_message();
        end_of_pool_send_pending_ = false;
        waiting_pool_tp_tx_complete_ = false;
        session_deadline_ms_ = now_ms + session_timeout_ms_;
        return;
      }
      if (end_of_pool_retry_count_ < 3u) {
        ++end_of_pool_retry_count_;
        printWarn(
          "Timeout waiting VT EndOfPool response, retry EndOfPool " +
          std::to_string(end_of_pool_retry_count_) + "/3");
        send_end_of_object_pool_message();
        session_deadline_ms_ = now_ms + session_timeout_ms_;
        return;
      }
      restart_or_fail(now_ms, "Timeout waiting VT EndOfPool response");
      return;
    case SessionState::WaitPoolActivateResponse:
      restart_or_fail(now_ms, "Timeout waiting VT pool activation response");
      return;
    default:
      return;
  }
}

const char * VTClient::session_state_name(SessionState s)
{
  switch (s) {
    case SessionState::Idle: return "Idle";
    case SessionState::RequestVersion: return "RequestVersion";
    case SessionState::WaitVersionResponse: return "WaitVersionResponse";
    case SessionState::RequestMemory: return "RequestMemory";
    case SessionState::WaitMemoryResponse: return "WaitMemoryResponse";
    case SessionState::WaitPoolTransferResponse: return "WaitPoolTransferResponse";
    case SessionState::WaitPoolActivateResponse: return "WaitPoolActivateResponse";
    case SessionState::Active: return "Active";
    case SessionState::Error: return "Error";
    default: return "Unknown";
  }
}

void VTClient::send_vt_command(VTProtocolCodec::Function function, const std::vector<std::uint8_t> & args)
{
  // VT command carriage rule:
  // - <= 8 bytes total payload -> PGN 0xE700 single frame
  // - > 8 bytes total payload  -> PGN 0xE700 TP payload
  // (ISO 11783 transport profile on top of J1939 framing).
  const auto payload = codec_.build_command_payload(function, args);
  const std::size_t total_len = payload.size();
  if (total_len > 8) {
    // Fallback: oversized VT command payload via TP channel.
    msg::IsobusTpFrame tp;
    tp.priority = 6;
    tp.page = false;
    tp.pgn = kPgnWorkingSetToVt;
    tp.sa = local_sa_;
    tp.pf = static_cast<std::uint8_t>((tp.pgn >> 8) & 0xFF);
    tp.ps = vt_sa_;
    tp.data = payload;
    send_tp_frame(tp);
    printWarn("VT command payload exceeded 8 bytes, sent via TP");
    return;
  }

  msg::IsobusFrame fr;
  fr.priority = 6;
  fr.page = false;
  fr.pgn = kPgnWorkingSetToVt;
  fr.sa = local_sa_;
  fr.pf = static_cast<std::uint8_t>((fr.pgn >> 8) & 0xFF);
  fr.ps = vt_sa_;
  fr.data.fill(0xFF);
  for (std::size_t i = 0; i < payload.size(); ++i) {
    fr.data[i] = payload[i];
  }
  send_frame(fr);
}

void VTClient::send_change_active_mask_command()
{
  const std::uint16_t mask_id = active_mask_id();
  send_vt_command(
    VTProtocolCodec::Function::ChangeActiveMask,
    codec_.build_change_active_mask_args(vt_working_set_id_, mask_id));
  printInfo(
    "VT ChangeActiveMask sent: ws_id=0x" + [&]() {
      char b[8];
      std::snprintf(b, sizeof(b), "%04X", vt_working_set_id_);
      return std::string(b);
    }() +
    ", mask_id=0x" + [&]() {
      char b[8];
      std::snprintf(b, sizeof(b), "%04X", mask_id);
      return std::string(b);
    }());
}

std::uint16_t VTClient::active_mask_id() const
{
  // Prefer first valid DataMask id parsed from XML pool. Fallback to configured pool id.
  for (const auto & m : pool_model_.data_masks()) {
    if (m.object_id != 0) {
      return m.object_id;
    }
  }
  return vt_object_pool_id_;
}

bool VTClient::send_pool_transfer_message()
{
  msg::IsobusTpFrame tp;
  tp.priority = 6;
  tp.page = false;
  tp.pgn = kPgnWorkingSetToVt;
  tp.sa = local_sa_;
  tp.pf = static_cast<std::uint8_t>((tp.pgn >> 8) & 0xFF);
  tp.ps = vt_sa_;

  // ISO 11783-6:2018 Annex C.2.3:
  // [0]=fn17, [1..]=object pool records.
  tp.data = codec_.build_command_payload(
    VTProtocolCodec::Function::ObjectPoolTransfer, pool_model_.pool_bytes());

  send_tp_frame(tp);
  printInfo(
    "VT object pool sent (bytes=" + std::to_string(tp.data.size()) +
    ", vt_sa=0x" + to_hex_u8(vt_sa_) +
    ", ws_id=0x" + [&]() {
      char b[8];
      std::snprintf(b, sizeof(b), "%04X", vt_working_set_id_);
      return std::string(b);
    }() +
    ", pool_id=0x" + [&]() {
      char b[8];
      std::snprintf(b, sizeof(b), "%04X", vt_object_pool_id_);
      return std::string(b);
    }() + ")");
  return true;
}

void VTClient::send_end_of_object_pool_message()
{
  msg::IsobusFrame fr;
  fr.priority = 6;
  fr.page = false;
  fr.pgn = kPgnWorkingSetToVt;
  fr.sa = local_sa_;
  fr.pf = static_cast<std::uint8_t>((fr.pgn >> 8) & 0xFF);
  fr.ps = vt_sa_;
  fr.data.fill(0xFF);
  // Annex C.2.4 End of Object Pool message (function 18).
  const auto payload = codec_.build_command_payload(VTProtocolCodec::Function::EndOfObjectPool);
  fr.data[0] = payload[0];
  send_frame(fr);
  pool_transfer_response_received_ = false;
  vt_status_parsing_clear_count_ = 0;
  printInfo("VT End of Object Pool sent");
}

bool VTClient::send_change_active_mask_by_id(std::uint16_t mask_id)
{
  if (mask_id == 0) {
    printWarn("Cannot send ChangeActiveMask with unresolved id (0)");
    return false;
  }
  send_vt_command(
    VTProtocolCodec::Function::ChangeActiveMask,
    codec_.build_change_active_mask_args(vt_working_set_id_, mask_id));
  return true;
}

bool VTClient::send_change_soft_key_mask_by_id(std::uint16_t mask_id)
{
  if (mask_id == 0) {
    printWarn("Cannot send ChangeSoftKeyMask with unresolved id (0)");
    return false;
  }
  send_change_soft_key_mask_command(mask_id);
  return true;
}

bool VTClient::apply_xml_update(const std::string & xml_element)
{
  // Only one runtime pool-transfer sequence may be active at a time:
  // GetMemory -> fn11 -> TP complete -> fn12 -> fn12 response.
  if (
    runtime_update_waiting_memory_ || waiting_pool_tp_tx_complete_ ||
    end_of_pool_send_pending_ || runtime_update_eop_inflight_ > 0)
  {
    {
      std::lock_guard<std::mutex> lock(pending_updates_mutex_);
      pending_pool_updates_.push_back(xml_element);
    }
    report_runtime_update_result(false, "queued");
    return true;
  }
  return start_runtime_update(xml_element);
}

bool VTClient::start_runtime_update(const std::string & xml_element)
{
  if (xml_element.empty()) {
    printWarn("VT update parse failed: empty XML");
    report_runtime_update_result(false, "empty_xml");
    return false;
  }

  std::string xml;
  if (xml_element.find("<objectpool") != std::string::npos) {
    xml = xml_element;
  } else {
    // Wrap single element using original loaded XML header.
    std::string objectpool_open = source_objectpool_open_tag_;
    if (objectpool_open.empty()) {
      objectpool_open =
        "<objectpool dimension=\"" + std::to_string(active_build_cfg_.vt_dimension) +
        "\" sk_width=\"" + std::to_string(active_build_cfg_.vt_softkey_width) +
        "\" sk_height=\"" + std::to_string(active_build_cfg_.vt_softkey_height) +
        "\" std_bitmap_path=\"images\\\" fix_bitmap_path=\"images\\\">";
      printWarn("VT update using fallback objectpool header (source header unavailable)");
    }
    xml = source_xml_declaration_ + "\n" + objectpool_open + "\n" + xml_element + "\n</objectpool>\n";
  }

  FILE * tf = std::tmpfile();
  if (tf == nullptr) {
    printWarn("VT update parse failed: cannot create temp file");
    report_runtime_update_result(false, "tempfile_create_failed");
    return false;
  }
  const auto written = std::fwrite(xml.data(), 1, xml.size(), tf);
  if (written != xml.size()) {
    std::fclose(tf);
    printWarn("VT update parse failed: temp file write failed");
    report_runtime_update_result(false, "tempfile_write_failed");
    return false;
  }
  std::rewind(tf);

  std::vector<std::uint8_t> update_pool_bytes;
  active_update_pool_bytes = &update_pool_bytes;
  update_parse_had_elements = false;
  parse(
    tf,
    &update_parser_start_cb,
    &update_parser_end_cb,
    &update_parser_ready_cb,
    active_build_cfg_.vt_dimension,
    active_build_cfg_.vt_softkey_width,
    active_build_cfg_.vt_softkey_height,
    active_build_cfg_.vt_colors);
  active_update_pool_bytes = nullptr;
  std::fclose(tf);

  if (!update_parse_had_elements || update_pool_bytes.empty()) {
    printWarn("VT update parse failed: pooledit_parser produced empty pool bytes");
    report_runtime_update_result(false, "parser_empty_output");
    return false;
  }

  runtime_update_pool_bytes_ = std::move(update_pool_bytes);
  runtime_update_waiting_memory_ = true;
  const std::uint32_t required_bytes = static_cast<std::uint32_t>(runtime_update_pool_bytes_.size());
  send_vt_command(
    VTProtocolCodec::Function::GetMemory,
    codec_.build_get_memory_args(required_bytes));
  printInfo(
    "VT runtime update memory request sent (bytes=" +
    std::to_string(required_bytes) + "), waiting memory response before fn11");
  return true;
}

void VTClient::dispatch_next_pending_runtime_update()
{
  std::string next_update;
  {
    std::lock_guard<std::mutex> lock(pending_updates_mutex_);
    if (!pending_pool_updates_.empty()) {
      next_update = pending_pool_updates_.front();
      pending_pool_updates_.pop_front();
    }
  }
  if (!next_update.empty()) {
    (void)start_runtime_update(next_update);
  }
}

void VTClient::report_runtime_update_result(bool success, const std::string & detail)
{
  std::uint32_t pending_updates = 0;
  {
    std::lock_guard<std::mutex> lock(pending_updates_mutex_);
    pending_updates = static_cast<std::uint32_t>(pending_pool_updates_.size());
  }
  const bool in_progress =
    runtime_update_waiting_memory_ || waiting_pool_tp_tx_complete_ ||
    end_of_pool_send_pending_ || runtime_update_eop_inflight_ > 0;
  on_runtime_update_result(success, detail, pending_updates, in_progress ? 1u : 0u);
}

void VTClient::send_change_soft_key_mask_command(std::uint16_t mask_id)
{
  send_vt_command(
    VTProtocolCodec::Function::ChangeSoftKeyMask,
    codec_.build_change_soft_key_mask_args(vt_working_set_id_, mask_id));
  printInfo(
    "VT ChangeSoftKeyMask sent: ws_id=0x" + [&]() {
      char b[8];
      std::snprintf(b, sizeof(b), "%04X", vt_working_set_id_);
      return std::string(b);
    }() +
    ", mask_id=0x" + [&]() {
      char b[8];
      std::snprintf(b, sizeof(b), "%04X", mask_id);
      return std::string(b);
    }());
}

std::string VTClient::to_hex_u8(std::uint8_t value)
{
  char b[8];
  std::snprintf(b, sizeof(b), "%02X", value);
  return std::string(b);
}

const char * VTClient::transfer_status_text(std::uint8_t status)
{
  switch (status) {
    case 0x00: return "accepted";
    case 0x01: return "busy_try_later";
    case 0x02: return "out_of_memory";
    case 0x03: return "format_not_supported";
    case 0x04: return "pool_not_acceptable";
    default: return "unknown_transfer_status";
  }
}

const char * VTClient::memory_status_text(std::uint8_t status)
{
  // ISO 11783-6:2018 D.3 Get Memory status codes.
  switch (status) {
    case 0x00: return "there_can_be_enough_memory";
    case 0x01: return "not_enough_memory";
    default: return "unknown_memory_status";
  }
}

std::string VTClient::hardware_bits_text(std::uint8_t bits)
{
  std::string out;
  auto add = [&](const char * s) {
    if (!out.empty()) {
      out += ",";
    }
    out += s;
  };
  if (bits == 0) {
    return "none";
  }
  if (bits & (1u << 0)) add("touchscreen");
  if (bits & (1u << 1)) add("pointing_device");
  if (bits & (1u << 2)) add("audio_multi_frequency");
  if (bits & (1u << 3)) add("audio_adjustable_volume");
  if (bits & (1u << 4)) add("simultaneous_softkeys");
  if (bits & (1u << 5)) add("simultaneous_buttons");
  if (bits & (1u << 6)) add("drag_reporting");
  if (bits & (1u << 7)) add("drag_intermediate_coords");
  if (out.empty()) {
    out = "unmapped_bits";
  }
  return out;
}

bool VTClient::extract_vt_build_profile(const std::vector<std::uint8_t> & payload, BuildConfig & cfg_out)
{
  VTProtocolCodec codec;
  // ISO 11783-6:2018 D.9 Get Hardware response:
  // byte3 graphic type, bytes5..6 X pixels, bytes7..8 Y pixels.
  if (payload.size() < 8 || payload[0] != static_cast<std::uint8_t>(VTProtocolCodec::Function::GetHardware)) return false;
  const auto x_pixels = static_cast<int>(codec.read_u16_le(payload, 4));
  const auto y_pixels = static_cast<int>(codec.read_u16_le(payload, 6));
  if (x_pixels <= 0 || y_pixels <= 0) return false;

  int colors = 2;
  switch (payload[2]) {
    case 0: colors = 2; break;
    case 1: colors = 16; break;
    case 2: colors = 256; break;
    default: return false;
  }

  const int dim = std::min(x_pixels, y_pixels);
  if (dim < 100) return false;

  // Soft key dimensions are not included in D.9; keep fallback values.
  const int sk_w = cfg_out.vt_softkey_width;
  const int sk_h = cfg_out.vt_softkey_height;

  cfg_out.vt_dimension = dim;
  cfg_out.vt_softkey_width = sk_w;
  cfg_out.vt_softkey_height = sk_h;
  cfg_out.vt_colors = colors;
  return true;
}

void VTClient::ensure_pool_loaded_from_version_payload(const std::vector<std::uint8_t> & payload)
{
  if (!pool_model_.pool_bytes().empty() || !pool_source_configured_) {
    return;
  }
  BuildConfig runtime_cfg = fallback_cfg_;
  bool vt_reported = false;
  if (use_vt_reported_profile_ && extract_vt_build_profile(payload, runtime_cfg)) {
    vt_reported = true;
  }
  if (load_pool_from_xml(pool_xml_path_, runtime_cfg)) {
    on_pool_ready(pool_xml_path_, runtime_cfg, vt_reported);
  } else {
    on_pool_build_failed(pool_xml_path_);
  }
}

void VTClient::transition_to(SessionState next, std::uint64_t now_ms, const std::string & reason)
{
  session_state_ = next;
  session_deadline_ms_ = now_ms + session_timeout_ms_;
  if (next == SessionState::WaitPoolTransferResponse) {
    pool_transfer_wait_started_ms_ = now_ms;
    missing_fn18_wait_warned_ = false;
  }
  printInfo(std::string("VT session -> ") + session_state_name(next) + " (" + reason + ")");
}

void VTClient::restart_or_fail(std::uint64_t now_ms, const std::string & reason)
{
  if (session_retry_count_ < session_max_retries_) {
    ++session_retry_count_;
    printWarn(reason + ", retry " + std::to_string(session_retry_count_) + "/" +
             std::to_string(session_max_retries_));
    start_session(now_ms);
    return;
  }
  session_state_ = SessionState::Error;
  printWarn(reason + ", retries exhausted");
}

}  // namespace ros2_isobus
