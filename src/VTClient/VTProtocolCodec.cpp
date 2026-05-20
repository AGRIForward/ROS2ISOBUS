#include "VTProtocolCodec.hpp"

#include <algorithm>

namespace ros2_isobus
{

std::uint16_t VTProtocolCodec::read_u16_le(
  const std::vector<std::uint8_t> & payload, std::size_t offset) const
{
  // Decode little-endian uint16 from payload with bounds guard.
  if (payload.size() < offset + 2) {
    return 0;
  }
  return static_cast<std::uint16_t>(payload[offset]) |
         (static_cast<std::uint16_t>(payload[offset + 1]) << 8);
}

std::uint32_t VTProtocolCodec::read_u32_le(
  const std::vector<std::uint8_t> & payload, std::size_t offset) const
{
  // Decode little-endian uint32 from payload with bounds guard.
  if (payload.size() < offset + 4) {
    return 0;
  }
  return static_cast<std::uint32_t>(payload[offset]) |
         (static_cast<std::uint32_t>(payload[offset + 1]) << 8) |
         (static_cast<std::uint32_t>(payload[offset + 2]) << 16) |
         (static_cast<std::uint32_t>(payload[offset + 3]) << 24);
}

VersionResponse VTProtocolCodec::parse_version_response(const std::vector<std::uint8_t> & payload) const
{
  VersionResponse out{};
  // ISO 11783-6:2018 D.9 Get Hardware response:
  // b1=function(0xC7), b2=boot time, b3=graphic type, b4=hardware,
  // b5..b6=X pixels (LE16), b7..b8=Y pixels (LE16).
  if (payload.size() < 8 || payload[0] != static_cast<std::uint8_t>(Function::GetHardware)) {
    return out;
  }
  out.valid = true;
  out.boot_time_s = payload[1];
  out.graphic_type = payload[2];
  out.hardware_bits = payload[3];
  out.x_pixels = read_u16_le(payload, 4);
  out.y_pixels = read_u16_le(payload, 6);
  return out;
}

MemoryResponse VTProtocolCodec::parse_memory_response(const std::vector<std::uint8_t> & payload) const
{
  MemoryResponse out{};
  // ISO 11783-6:2018 D.3 Get Memory response:
  // b1=function(0xC0), b2=VT version, b3=status, b4..b8 reserved.
  if (payload.size() < 3 || payload[0] != static_cast<std::uint8_t>(Function::GetMemory)) {
    return out;
  }
  out.valid = true;
  out.vt_version = payload[1];
  out.status = payload[2];
  out.payload_size = payload.size();
  out.value_le = 0;
  return out;
}

PoolTransferResponse VTProtocolCodec::parse_pool_transfer_response(const std::vector<std::uint8_t> & payload) const
{
  PoolTransferResponse out{};
  // ISO 11783-6:2018 Annex C.2.5 End Of Object Pool response (fn18):
  // b2=error bitmap, b3..b4=parent object id, b5..b6=object id,
  // b7=object-pool error code, b8=reserved.
  if (payload.size() < 2 || payload[0] != static_cast<std::uint8_t>(Function::EndOfObjectPool)) {
    return out;
  }
  if (payload.size() < 8) {
    return out;
  }
  out.valid = true;
  out.error_codes = payload[1];
  out.parent_object_id = read_u16_le(payload, 2);
  out.object_id = read_u16_le(payload, 4);
  out.object_pool_error_codes = payload[6];
  out.reserved = payload[7];
  return out;
}

VtStatus VTProtocolCodec::parse_vt_status(const std::vector<std::uint8_t> & payload) const
{
  VtStatus out{};
  // ISO 11783-6 VT Status (8 bytes):
  // b2=active WS SA, b3..b4=data/alarm mask id, b5..b6=softkey mask id,
  // b7=busy/status bits, b8=current command function.
  if (payload.size() < 8 || payload[0] != static_cast<std::uint8_t>(Function::VtStatus)) {
    return out;
  }
  out.valid = true;
  out.active_ws_sa = payload[1];
  out.visible_data_alarm_mask_id = read_u16_le(payload, 2);
  out.visible_soft_key_mask_id = read_u16_le(payload, 4);
  out.busy_codes = payload[6];
  out.raw_status_byte7 = payload[6];
  out.current_command_function = payload[7];

  out.busy_updating_visible_mask = (out.busy_codes & 0x01u) != 0u;
  out.busy_saving_to_nonvolatile = (out.busy_codes & 0x02u) != 0u;
  out.busy_executing_command = (out.busy_codes & 0x04u) != 0u;
  out.busy_executing_macro = (out.busy_codes & 0x08u) != 0u;
  out.parsing_active = (out.busy_codes & 0x10u) != 0u;
  out.aux_learn_mode_active = (out.busy_codes & 0x40u) != 0u;
  out.out_of_memory = (out.busy_codes & 0x80u) != 0u;
  return out;
}

PointingEvent VTProtocolCodec::parse_pointing_event(const std::vector<std::uint8_t> & payload) const
{
  PointingEvent out{};
  // ISO 11783-6 Pointing Event (fn2):
  // b2..b3=object id, b4..b5=x, b6..b7=y, b8=event code.
  if (payload.size() < 8 || payload[0] != static_cast<std::uint8_t>(Function::PointingEvent)) {
    return out;
  }
  out.valid = true;
  out.object_id = read_u16_le(payload, 1);
  out.x = static_cast<std::int32_t>(static_cast<std::int16_t>(read_u16_le(payload, 3)));
  out.y = static_cast<std::int32_t>(static_cast<std::int16_t>(read_u16_le(payload, 5)));
  out.event_code = payload[7];
  return out;
}

NavigationEvent VTProtocolCodec::parse_navigation_event(const std::vector<std::uint8_t> & payload) const
{
  NavigationEvent out{};
  // Navigation-like VT events:
  // - fn3 Select Input Object Event
  // - fn4 ESC event
  if (payload.empty()) {
    return out;
  }
  if (payload[0] == static_cast<std::uint8_t>(Function::SelectInputObjectEvent)) {
    if (payload.size() < 3) {
      return out;
    }
    out.valid = true;
    out.function_code = static_cast<std::uint8_t>(Function::SelectInputObjectEvent);
    out.object_id = read_u16_le(payload, 1);
    return out;
  }
  if (payload[0] == static_cast<std::uint8_t>(Function::Esc)) {
    out.valid = true;
    out.function_code = static_cast<std::uint8_t>(Function::Esc);
    out.object_id = 0xFFFF;
    return out;
  }
  return out;
}

VtAuxInputStatus VTProtocolCodec::parse_aux_input_status_type2(const std::vector<std::uint8_t> & payload) const
{
  VtAuxInputStatus out{};
  // ISO 11783-6:2018 Annex J.7.9 Auxiliary Input Type 2 Status:
  // b1=0x26, b2..b3=input object id, b4..b5=value1, b6..b7=value2, b8=operating state.
  if (payload.size() < 8 || payload[0] != static_cast<std::uint8_t>(Function::AuxiliaryInputStatusType2)) {
    return out;
  }
  out.valid = true;
  out.input_object_id = read_u16_le(payload, 1);
  out.value1 = read_u16_le(payload, 3);
  out.value2 = read_u16_le(payload, 5);
  out.operating_state_bits = payload[7];
  out.learn_mode_active = (out.operating_state_bits & 0x01u) != 0u;
  out.input_activated_in_learn_mode = (out.operating_state_bits & 0x02u) != 0u;
  out.locked = (out.operating_state_bits & 0x04u) != 0u;
  out.interaction_detected = (out.operating_state_bits & 0x08u) != 0u;
  return out;
}

VtAuxAssignmentCommand VTProtocolCodec::parse_aux_assignment_type2_command(
  const std::vector<std::uint8_t> & payload) const
{
  VtAuxAssignmentCommand out{};
  // ISO 11783-6:2018 Annex J.7.5 Auxiliary Assignment Type 2 command:
  // b1=0x24
  // b2..b9 = AUX input NAME (LE64 on bus),
  // b10    = assignment attributes:
  //          bits 0..4 assigned input function type (Table J.5),
  //          bit 7 store-as-preferred-assignment
  // b11..b12=input object id, b13..b14=function object id.
  if (payload.size() < 14 || payload[0] != static_cast<std::uint8_t>(Function::AuxiliaryAssignmentType2)) {
    return out;
  }
  out.valid = true;
  out.aux_input_name = 0ULL;
  for (std::size_t i = 0; i < 8; ++i) {
    out.aux_input_name |= (static_cast<std::uint64_t>(payload[1 + i]) << (8 * i));
  }
  // StoreAsPreferredAssignment flag as used in current VT/CANoe interoperability:
  // bit7=1 => store as preferred assignment, bit7=0 => do not store.
  out.store_as_preferred_assignment = (payload[9] & 0x80u) != 0u;
  out.assigned_input_function_type = payload[9] & 0x1Fu;
  out.aux_input_object_id = read_u16_le(payload, 10);
  out.aux_function_object_id = read_u16_le(payload, 12);
  out.remove_assignment =
    (out.aux_input_name == 0xFFFFFFFFFFFFFFFFULL) ||
    (out.aux_input_object_id == 0xFFFFu) ||
    (out.assigned_input_function_type == 0x1Fu);
  return out;
}

VtAuxInputMaintenance VTProtocolCodec::parse_aux_input_maintenance_type2(
  const std::vector<std::uint8_t> & payload) const
{
  VtAuxInputMaintenance out{};
  // ISO 11783-6:2018 Annex J.7.10 Auxiliary Input Type 2 Maintenance:
  // b1=0x25, b2..b3=model identification code, b4=status.
  if (payload.size() < 4 || payload[0] != static_cast<std::uint8_t>(Function::AuxiliaryInputMaintenanceType2)) {
    return out;
  }
  out.valid = true;
  out.model_identification_code = read_u16_le(payload, 1);
  out.status = payload[3];
  return out;
}

VtPreferredAssignmentResponse VTProtocolCodec::parse_preferred_assignment_response(
  const std::vector<std::uint8_t> & payload) const
{
  VtPreferredAssignmentResponse out{};
  // ISO 11783-6:2018 Annex J.7.8 Preferred Assignment response:
  // b1=0x23, b2=error code bits, b3..b4=faulty function object id.
  if (payload.size() < 4 || payload[0] != static_cast<std::uint8_t>(Function::PreferredAssignmentResponse)) {
    return out;
  }
  out.valid = true;
  out.error_code_bits = payload[1];
  out.faulty_aux_function_object_id = read_u16_le(payload, 2);
  return out;
}

std::vector<std::uint8_t> VTProtocolCodec::build_get_memory_args(std::uint32_t required_bytes) const
{
  // ISO 11783-6:2018 D.2 Get Memory request args:
  // b2=FF, b3..b6=required memory bytes (LE32), b7..b8=FF.
  std::vector<std::uint8_t> args;
  args.reserve(7);
  args.push_back(0xFF);
  args.push_back(static_cast<std::uint8_t>(required_bytes & 0xFFu));
  args.push_back(static_cast<std::uint8_t>((required_bytes >> 8) & 0xFFu));
  args.push_back(static_cast<std::uint8_t>((required_bytes >> 16) & 0xFFu));
  args.push_back(static_cast<std::uint8_t>((required_bytes >> 24) & 0xFFu));
  args.push_back(0xFF);
  args.push_back(0xFF);
  return args;
}

std::vector<std::uint8_t> VTProtocolCodec::build_change_numeric_value_args(
  std::uint16_t object_id, std::int32_t value) const
{
  // ISO 11783-6 Annex F.22 Change Numeric Value payload:
  // object id (LE16), reserved/padding byte, value (LE32).
  std::vector<std::uint8_t> args;
  args.reserve(7);
  args.push_back(static_cast<std::uint8_t>(object_id & 0xFF));
  args.push_back(static_cast<std::uint8_t>((object_id >> 8) & 0xFF));
  args.push_back(0xFF);
  args.push_back(static_cast<std::uint8_t>(value & 0xFF));
  args.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
  args.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
  args.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));
  return args;
}

std::vector<std::uint8_t> VTProtocolCodec::build_change_numeric_value_u8_args(
  std::uint16_t object_id, std::uint8_t value) const
{
  // ISO 11783-6 Annex F.22 Change Numeric Value for 1-byte objects
  // (e.g. Input Boolean): value in byte 5, bytes 6..8 set to FF.
  std::vector<std::uint8_t> args;
  args.reserve(7);
  args.push_back(static_cast<std::uint8_t>(object_id & 0xFF));
  args.push_back(static_cast<std::uint8_t>((object_id >> 8) & 0xFF));
  args.push_back(0xFF);
  args.push_back(value);
  args.push_back(0xFF);
  args.push_back(0xFF);
  args.push_back(0xFF);
  return args;
}

std::vector<std::uint8_t> VTProtocolCodec::build_change_string_value_args(
  std::uint16_t object_id, const std::string & value) const
{
  // ISO 11783-6:2018 F.24 Change String Value:
  // object id (LE16), string length in bytes (LE16), then string bytes.
  const std::uint16_t length = static_cast<std::uint16_t>(
    std::min<std::size_t>(value.size(), 0xFFFFu));
  std::vector<std::uint8_t> args;
  args.reserve(4 + length);
  args.push_back(static_cast<std::uint8_t>(object_id & 0xFF));
  args.push_back(static_cast<std::uint8_t>((object_id >> 8) & 0xFF));
  args.push_back(static_cast<std::uint8_t>(length & 0xFF));
  args.push_back(static_cast<std::uint8_t>((length >> 8) & 0xFF));
  args.insert(args.end(), value.begin(), value.begin() + static_cast<std::ptrdiff_t>(length));
  return args;
}

std::vector<std::uint8_t> VTProtocolCodec::build_change_list_item_args(
  std::uint16_t object_id, std::uint8_t index) const
{
  // ISO 11783-6 Annex F command payload: object id + selected list index.
  std::vector<std::uint8_t> args;
  args.reserve(3);
  args.push_back(static_cast<std::uint8_t>(object_id & 0xFF));
  args.push_back(static_cast<std::uint8_t>((object_id >> 8) & 0xFF));
  args.push_back(index);
  return args;
}

std::vector<std::uint8_t> VTProtocolCodec::build_hide_show_object_args(
  std::uint16_t object_id, bool visible) const
{
  // ISO 11783-6 Annex F command payload: object id + visible flag.
  std::vector<std::uint8_t> args;
  args.reserve(3);
  args.push_back(static_cast<std::uint8_t>(object_id & 0xFF));
  args.push_back(static_cast<std::uint8_t>((object_id >> 8) & 0xFF));
  args.push_back(visible ? 1u : 0u);
  return args;
}

std::vector<std::uint8_t> VTProtocolCodec::build_enable_disable_object_args(
  std::uint16_t object_id, bool enabled) const
{
  // ISO 11783-6 Annex F command payload: object id + enabled flag.
  return build_hide_show_object_args(object_id, enabled);
}

std::vector<std::uint8_t> VTProtocolCodec::build_select_input_object_args(std::uint16_t object_id) const
{
  // ISO 11783-6 Annex F command payload: select object, trailing reserved byte.
  std::vector<std::uint8_t> args;
  args.reserve(3);
  args.push_back(static_cast<std::uint8_t>(object_id & 0xFF));
  args.push_back(static_cast<std::uint8_t>((object_id >> 8) & 0xFF));
  args.push_back(0x00);
  return args;
}

std::vector<std::uint8_t> VTProtocolCodec::build_change_attribute_args(
  std::uint16_t object_id, std::uint8_t attribute_id, std::uint32_t value) const
{
  // ISO 11783-6 Annex F command payload: object id + AID + value (LE32).
  std::vector<std::uint8_t> args;
  args.reserve(7);
  args.push_back(static_cast<std::uint8_t>(object_id & 0xFF));
  args.push_back(static_cast<std::uint8_t>((object_id >> 8) & 0xFF));
  args.push_back(attribute_id);
  args.push_back(static_cast<std::uint8_t>(value & 0xFFu));
  args.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFu));
  args.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFFu));
  args.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFFu));
  return args;
}

std::vector<std::uint8_t> VTProtocolCodec::build_change_child_position_args(
  std::uint16_t parent_object_id, std::uint16_t child_object_id, std::uint16_t x, std::uint16_t y) const
{
  // ISO 11783-6 Annex F command payload for child positioning inside parent object.
  std::vector<std::uint8_t> args;
  args.reserve(8);
  args.push_back(static_cast<std::uint8_t>(parent_object_id & 0xFF));
  args.push_back(static_cast<std::uint8_t>((parent_object_id >> 8) & 0xFF));
  args.push_back(static_cast<std::uint8_t>(child_object_id & 0xFF));
  args.push_back(static_cast<std::uint8_t>((child_object_id >> 8) & 0xFF));
  args.push_back(static_cast<std::uint8_t>(x & 0xFF));
  args.push_back(static_cast<std::uint8_t>((x >> 8) & 0xFF));
  args.push_back(static_cast<std::uint8_t>(y & 0xFF));
  args.push_back(static_cast<std::uint8_t>((y >> 8) & 0xFF));
  return args;
}

std::vector<std::uint8_t> VTProtocolCodec::build_change_active_mask_args(
  std::uint16_t ws_id, std::uint16_t mask_id) const
{
  // ISO 11783-6:2018 F.34/F.35 Change Active Mask:
  // ws id, mask id, then reserved bytes.
  std::vector<std::uint8_t> args;
  args.reserve(7);
  args.push_back(static_cast<std::uint8_t>(ws_id & 0xFF));
  args.push_back(static_cast<std::uint8_t>((ws_id >> 8) & 0xFF));
  args.push_back(static_cast<std::uint8_t>(mask_id & 0xFF));
  args.push_back(static_cast<std::uint8_t>((mask_id >> 8) & 0xFF));
  args.push_back(0xFF);
  args.push_back(0xFF);
  args.push_back(0xFF);
  return args;
}

std::vector<std::uint8_t> VTProtocolCodec::build_change_soft_key_mask_args(
  std::uint16_t ws_id, std::uint16_t mask_id) const
{
  // ISO 11783-6 Annex F command payload is structurally equivalent to active-mask form.
  return build_change_active_mask_args(ws_id, mask_id);
}

std::vector<std::uint8_t> VTProtocolCodec::build_aux_assignment_type2_response_args(
  std::uint16_t aux_function_object_id, std::uint8_t error_code_bits) const
{
  // ISO 11783-6:2018 Annex J.7.6 Auxiliary Assignment Type 2 response args:
  // b2..b3=function object id, b4=error bitmap, b5..b8=FF.
  std::vector<std::uint8_t> args;
  args.reserve(7);
  args.push_back(static_cast<std::uint8_t>(aux_function_object_id & 0xFFu));
  args.push_back(static_cast<std::uint8_t>((aux_function_object_id >> 8) & 0xFFu));
  args.push_back(error_code_bits);
  args.push_back(0xFFu);
  args.push_back(0xFFu);
  args.push_back(0xFFu);
  args.push_back(0xFFu);
  return args;
}

std::vector<std::uint8_t> VTProtocolCodec::build_preferred_assignment_command_args(
  const std::vector<std::tuple<std::uint64_t, std::uint16_t, std::vector<std::pair<std::uint16_t, std::uint16_t>>>> &
    groups) const
{
  // ISO 11783-6:2018 Annex J.7.7 Preferred Assignment command args:
  // b2=number of input units; repeated:
  //   b3..b10 NAME (LE64), b11..b12 model id, b13 function-count,
  //   then per function: function object id + input object id (LE16 each).
  std::vector<std::uint8_t> args;
  args.reserve(1 + groups.size() * 16);
  args.push_back(static_cast<std::uint8_t>(std::min<std::size_t>(255U, groups.size())));
  for (const auto & g : groups) {
    const auto aux_name = std::get<0>(g);
    const auto model_id = std::get<1>(g);
    const auto & pairs = std::get<2>(g);
    for (std::size_t i = 0; i < 8; ++i) {
      args.push_back(static_cast<std::uint8_t>((aux_name >> (8 * i)) & 0xFFu));
    }
    args.push_back(static_cast<std::uint8_t>(model_id & 0xFFu));
    args.push_back(static_cast<std::uint8_t>((model_id >> 8) & 0xFFu));
    args.push_back(static_cast<std::uint8_t>(std::min<std::size_t>(255U, pairs.size())));
    for (const auto & p : pairs) {
      args.push_back(static_cast<std::uint8_t>(p.first & 0xFFu));
      args.push_back(static_cast<std::uint8_t>((p.first >> 8) & 0xFFu));
      args.push_back(static_cast<std::uint8_t>(p.second & 0xFFu));
      args.push_back(static_cast<std::uint8_t>((p.second >> 8) & 0xFFu));
    }
  }
  return args;
}

std::vector<std::uint8_t> VTProtocolCodec::build_command_payload(
  Function function, const std::vector<std::uint8_t> & args) const
{
  // Final VT command payload format:
  // byte1=function id, bytes2..N=command arguments.
  std::vector<std::uint8_t> payload;
  payload.reserve(1 + args.size());
  payload.push_back(static_cast<std::uint8_t>(function));
  payload.insert(payload.end(), args.begin(), args.end());
  return payload;
}

}  // namespace ros2_isobus
