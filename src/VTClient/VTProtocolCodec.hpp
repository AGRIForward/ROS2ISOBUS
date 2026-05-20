#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "VTTypes.hpp"

namespace ros2_isobus
{

/*
 *
 * VTProtocolCodec centralizes ISO 11783-6 VT payload handling:
 *  - Function id catalog (enum Function)
 *  - Decode VT->WS payloads to typed structs
 *  - Build WS->VT command argument payloads
 *  - Build final command payload [function + args]
 *
 * This class is transport-agnostic. CAN/TP frame carriage is handled by VTClient.
 *
 */
class VTProtocolCodec
{
public:
  // VT function identifiers (ISO 11783-6).
  enum class Function : std::uint8_t
  {
    SoftKeyActivation = 0x00,        // ISO 11783-6:2018 Annex H.2
    ButtonActivation = 0x01,         // ISO 11783-6:2018 Annex H.4
    PointingEvent = 0x02,            // ISO 11783-6:2018 Annex H.6
    SelectInputObjectEvent = 0x03,   // ISO 11783-6:2018 Annex H.8
    Esc = 0x04,                      // ISO 11783-6:2018 Annex H.10
    VtChangeNumericValue = 0x05,     // ISO 11783-6 VT->ECU change numeric value event
    VtChangeStringValue = 0x08,      // ISO 11783-6 VT->ECU change string value event

    ObjectPoolTransfer = 0x11,       // ISO 11783-6:2018 Annex C.2.3
    EndOfObjectPool = 0x12,          // ISO 11783-6:2018 Annex C.2.4/C.2.5

    HideShowObject = 0xA0,           // ISO 11783-6:2018 Annex F.2
    EnableDisableObject = 0xA1,      // ISO 11783-6:2018 Annex F.4
    SelectInputObjectCommand = 0xA2, // ISO 11783-6:2018 Annex F.6
    ChangeNumericValue = 0xA8,       // ISO 11783-6:2018 Annex F.22
    ChangeActiveMask = 0xAD,         // ISO 11783-6:2018 Annex F.34/F.35
    ChangeSoftKeyMask = 0xAE,        // ISO 11783-6:2018 Annex F.36
    ChangeAttribute = 0xAF,          // ISO 11783-6:2018 Annex F.38
    ChangeListItem = 0xB1,           // ISO 11783-6:2018 Annex F.42
    DeleteObjectPool = 0xB2,         // ISO 11783-6:2018 Annex F.44
    ChangeStringValue = 0xB3,        // ISO 11783-6:2018 Annex F.24
    ChangeChildPosition = 0xB4,      // ISO 11783-6:2018 Annex F.16

    GetMemory = 0xC0,                // ISO 11783-6:2018 D.2 (req) / D.3 (resp)
    GetHardware = 0xC7,              // ISO 11783-6:2018 D.9 (resp)
    AuxiliaryAssignmentType2 = 0x24, // ISO 11783-6:2018 Annex J.7.5/J.7.6
    AuxiliaryInputMaintenanceType2 = 0x23, // ISO 11783-6:2018 Annex J.7.10 (35dec)
    AuxiliaryInputStatusType2 = 0x26,// ISO 11783-6:2018 Annex J.7.9
    PreferredAssignmentCommand = 0x22, // ISO 11783-6:2018 Annex J.7.7
    PreferredAssignmentResponse = 0x22, // ISO 11783-6:2018 Annex J.7.8 (same function id as command)

    VtStatus = 0xFE                  // ISO 11783-6:2018 Annex H.20 (VT Status message)
  };

  // Little-endian scalar readers with bounds guard.
  std::uint16_t read_u16_le(const std::vector<std::uint8_t> & payload, std::size_t offset) const;
  std::uint32_t read_u32_le(const std::vector<std::uint8_t> & payload, std::size_t offset) const;

  // Decode VT->WS payloads to typed structures.
  VersionResponse parse_version_response(const std::vector<std::uint8_t> & payload) const;
  MemoryResponse parse_memory_response(const std::vector<std::uint8_t> & payload) const;
  PoolTransferResponse parse_pool_transfer_response(const std::vector<std::uint8_t> & payload) const;
  VtStatus parse_vt_status(const std::vector<std::uint8_t> & payload) const;
  PointingEvent parse_pointing_event(const std::vector<std::uint8_t> & payload) const;
  NavigationEvent parse_navigation_event(const std::vector<std::uint8_t> & payload) const;
  VtAuxInputStatus parse_aux_input_status_type2(const std::vector<std::uint8_t> & payload) const;
  VtAuxAssignmentCommand parse_aux_assignment_type2_command(const std::vector<std::uint8_t> & payload) const;
  VtAuxInputMaintenance parse_aux_input_maintenance_type2(const std::vector<std::uint8_t> & payload) const;
  VtPreferredAssignmentResponse parse_preferred_assignment_response(const std::vector<std::uint8_t> & payload) const;

  // Build WS->VT command arguments (function byte excluded).
  std::vector<std::uint8_t> build_get_memory_args(std::uint32_t required_bytes) const;
  std::vector<std::uint8_t> build_change_numeric_value_args(std::uint16_t object_id, std::int32_t value) const;
  std::vector<std::uint8_t> build_change_numeric_value_u8_args(
    std::uint16_t object_id, std::uint8_t value) const;
  std::vector<std::uint8_t> build_change_string_value_args(std::uint16_t object_id, const std::string & value) const;
  std::vector<std::uint8_t> build_change_list_item_args(std::uint16_t object_id, std::uint8_t index) const;
  std::vector<std::uint8_t> build_hide_show_object_args(std::uint16_t object_id, bool visible) const;
  std::vector<std::uint8_t> build_enable_disable_object_args(std::uint16_t object_id, bool enabled) const;
  std::vector<std::uint8_t> build_select_input_object_args(std::uint16_t object_id) const;
  std::vector<std::uint8_t> build_change_attribute_args(
    std::uint16_t object_id, std::uint8_t attribute_id, std::uint32_t value) const;
  std::vector<std::uint8_t> build_change_child_position_args(
    std::uint16_t parent_object_id, std::uint16_t child_object_id, std::uint16_t x, std::uint16_t y) const;
  std::vector<std::uint8_t> build_change_active_mask_args(std::uint16_t ws_id, std::uint16_t mask_id) const;
  std::vector<std::uint8_t> build_change_soft_key_mask_args(std::uint16_t ws_id, std::uint16_t mask_id) const;
  std::vector<std::uint8_t> build_aux_assignment_type2_response_args(
    std::uint16_t aux_function_object_id, std::uint8_t error_code_bits) const;
  std::vector<std::uint8_t> build_preferred_assignment_command_args(
    const std::vector<std::tuple<std::uint64_t, std::uint16_t, std::vector<std::pair<std::uint16_t, std::uint16_t>>>> &
      groups) const;

  // Build final VT command payload [function + args].
  std::vector<std::uint8_t> build_command_payload(Function function, const std::vector<std::uint8_t> & args = {}) const;
};

}  // namespace ros2_isobus
