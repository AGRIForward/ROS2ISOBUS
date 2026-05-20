#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "VTTypes.hpp"

namespace ros2_isobus
{

/*
 *
 * VTPoolModel owns VT object-pool model state:
 *  - XML -> object-pool byte stream conversion via pooledit parser
 *  - object id/name/topic bindings
 *  - DataMask / SoftKeyMask maps
 *  - fast lookup tables (by object id, numeric token)
 *
 * VTClient uses this as the single source of truth for pool data.
 *
 */
class VTPoolModel
{
public:
  // Load pool model from XML and rebuild all lookup maps.
  bool load_from_xml(const std::string & xml_path, const BuildConfig & cfg);
  // Clear all model state.
  void clear();

  // Access parsed/canonical model state.
  const std::vector<std::uint8_t> & pool_bytes() const { return pool_bytes_; }
  const std::vector<ObjectBinding> & bindings() const { return bindings_; }
  const std::vector<MaskBinding> & data_masks() const { return data_masks_; }
  const std::vector<MaskBinding> & soft_key_masks() const { return soft_key_masks_; }
  const std::unordered_map<std::uint16_t, std::size_t> & by_id() const { return by_id_; }
  const std::unordered_map<std::string, std::size_t> & numeric_by_token() const { return numeric_by_token_; }
  std::uint16_t working_set_id() const { return working_set_id_; }

  // Utility helpers shared by parser/model build.
  static std::string sanitize_topic_token(const std::string & text);
  static std::string attr_value(const char ** attr, const char * key);

  // Register one object binding during XML parse.
  static void add_binding(
    std::vector<ObjectBinding> & bindings,
    std::unordered_map<std::string, int> & topic_name_counts,
    ObjectKind kind, const std::string & tag, const std::string & name, std::uint16_t object_id,
    std::uint16_t parent_object_id);

  // Register one DataMask/SoftKeyMask binding during XML parse.
  static void add_mask_binding(
    std::vector<MaskBinding> & out,
    std::unordered_map<std::string, int> & topic_name_counts,
    const std::string & name, std::uint16_t object_id);

  // Rebuild lookup tables from binding vectors.
  static void rebuild_lookup(
    const std::vector<ObjectBinding> & bindings,
    std::unordered_map<std::uint16_t, std::size_t> & by_id,
    std::unordered_map<std::string, std::size_t> & numeric_by_token);

private:
  // Move temporary parse result into persistent instance state.
  void apply_load_result(PoolLoadResult && r);
  // XML parser callbacks.
  static void parser_start_cb(void * data, char * el, const char ** attr);
  static void parser_end_cb(void * data, char * el);
  static void parser_ready_cb(char * data, int length);

  // Canonical pool model state.
  std::vector<std::uint8_t> pool_bytes_{};
  std::vector<ObjectBinding> bindings_{};
  std::vector<MaskBinding> data_masks_{};
  std::vector<MaskBinding> soft_key_masks_{};
  // Fast lookup maps used by VTClientNode topic/event routing.
  std::unordered_map<std::uint16_t, std::size_t> by_id_{};
  std::unordered_map<std::string, std::size_t> numeric_by_token_{};
  // Parsed <workingset id="..."> identifier from XML.
  std::uint16_t working_set_id_{0};
};

}  // namespace ros2_isobus
