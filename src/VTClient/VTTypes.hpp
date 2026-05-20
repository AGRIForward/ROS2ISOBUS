#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_isobus
{

// Parsed XML object category used for dynamic ROS topic binding.
enum class ObjectKind
{
  Container,
  Button,
  SoftKey,
  AuxFunction,
  InputString,
  InputNumber,
  StringVariable,
  InputList,
  OutputList,
  NumberVariable,
  InputBoolean
};

// Object-level XML binding:
// - original tag/name from pool XML
// - ROS topic token generated from name
// - VT object id and parent id for VT command addressing.
struct ObjectBinding
{
  ObjectKind kind{ObjectKind::Button};
  std::string element_tag;
  std::string element_name;
  std::string topic_token;
  std::uint16_t object_id{0};
  std::uint16_t parent_object_id{0};
  std::uint8_t aux_function_type_id{0xFF};
};

// DataMask / SoftKeyMask binding used by mask control topics.
struct MaskBinding
{
  std::string element_name;
  std::string topic_token;
  std::uint16_t object_id{0};
};

// VT display profile used when transforming PoolEdit XML to object-pool bytes.
// Values correspond to parser transform inputs (dimension, softkey size, palette depth).
struct BuildConfig
{
  int vt_dimension{200};
  int vt_softkey_width{60};
  int vt_softkey_height{32};
  int vt_colors{256};
};

// ISO 11783-6 D.9 Get Hardware response (decoded).
struct VersionResponse
{
  bool valid{false};
  std::uint8_t boot_time_s{0xFF};
  std::uint8_t graphic_type{0xFF};
  std::uint8_t hardware_bits{0xFF};
  std::uint16_t x_pixels{0};
  std::uint16_t y_pixels{0};
};

// ISO 11783-6 Annex C.2.5 End Of Object Pool response (decoded).
struct PoolTransferResponse
{
  bool valid{false};
  std::uint8_t error_codes{0xFF};
  std::uint16_t parent_object_id{0xFFFF};
  std::uint16_t object_id{0xFFFF};
  std::uint8_t object_pool_error_codes{0xFF};
  std::uint8_t reserved{0xFF};
};

// ISO 11783-6 D.3 Get Memory response (decoded).
struct MemoryResponse
{
  bool valid{false};
  std::uint8_t vt_version{0xFF};
  std::uint8_t status{0xFF};
  std::uint32_t value_le{0};
  std::size_t payload_size{0};
};

// ISO 11783-6 VT Status message (decoded).
struct VtStatus
{
  bool valid{false};
  std::uint8_t active_ws_sa{0xFF};
  std::uint16_t visible_data_alarm_mask_id{0xFFFF};
  std::uint16_t visible_soft_key_mask_id{0xFFFF};
  std::uint8_t busy_codes{0x00};
  bool parsing_active{false};
  bool busy_updating_visible_mask{false};
  bool busy_saving_to_nonvolatile{false};
  bool busy_executing_command{false};
  bool busy_executing_macro{false};
  bool aux_learn_mode_active{false};
  bool out_of_memory{false};
  std::uint8_t current_command_function{0xFF};
  std::uint8_t raw_status_byte7{0xFF};
};

// ISO 11783-6 Pointing Event message (decoded).
struct PointingEvent
{
  bool valid{false};
  std::uint16_t object_id{0};
  std::int32_t x{0};
  std::int32_t y{0};
  std::uint8_t event_code{0xFF};
};

// ISO 11783-6 navigation-related VT event:
// Select Input Object Event or ESC.
struct NavigationEvent
{
  bool valid{false};
  std::uint8_t function_code{0xFF};
  std::uint16_t object_id{0xFFFF};
};

// ISO 11783-6:2018 Annex J.7.9 Auxiliary Input Type 2 Status (decoded).
struct VtAuxInputStatus
{
  bool valid{false};
  std::uint16_t input_object_id{0xFFFF};
  std::uint16_t value1{0xFFFF};
  std::uint16_t value2{0xFFFF};
  std::uint8_t operating_state_bits{0x00};
  bool learn_mode_active{false};
  bool input_activated_in_learn_mode{false};
  bool locked{false};
  bool interaction_detected{false};
};

// ISO 11783-6:2018 Annex J.7.5 Auxiliary Assignment Type 2 command (decoded).
struct VtAuxAssignmentCommand
{
  bool valid{false};
  std::uint64_t aux_input_name{0xFFFFFFFFFFFFFFFFULL};
  bool store_as_preferred_assignment{false};
  std::uint8_t assigned_input_function_type{0x1F};
  std::uint16_t aux_input_object_id{0xFFFF};
  std::uint16_t aux_function_object_id{0xFFFF};
  bool remove_assignment{false};
};

// ISO 11783-6:2018 Annex J.7.8 Preferred Assignment response (decoded).
struct VtPreferredAssignmentResponse
{
  bool valid{false};
  std::uint8_t error_code_bits{0x00};
  std::uint16_t faulty_aux_function_object_id{0xFFFF};
};

// ISO 11783-6:2018 Annex J.7.10 Auxiliary Input Type 2 Maintenance (decoded).
struct VtAuxInputMaintenance
{
  bool valid{false};
  std::uint16_t model_identification_code{0xFFFF};
  std::uint8_t status{0xFF};  // 0=Initializing, 1=Ready
};

// Output bundle from VTPoolModel XML load/build pipeline.
struct PoolLoadResult
{
  bool success{false};
  std::vector<std::uint8_t> pool_bytes{};
  std::vector<ObjectBinding> bindings{};
  std::vector<MaskBinding> data_masks{};
  std::vector<MaskBinding> soft_key_masks{};
  std::unordered_map<std::uint16_t, std::size_t> by_id{};
  std::unordered_map<std::string, std::size_t> numeric_by_token{};
  std::uint16_t working_set_id{0};
};

}  // namespace ros2_isobus
