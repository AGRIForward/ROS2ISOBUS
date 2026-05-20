#include "VTPoolModel.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <exception>

#include "parser.h"

namespace ros2_isobus
{

namespace
{
std::uint8_t parse_aux_function_type_id(const std::string & text)
{
  // Align topic/meta mapping with parser AUX-N output:
  // boolean -> 0x02 (Boolean non-latching), analog -> 0x01 (Analogue).
  if (text.empty()) {
    return 0xFFu;
  }
  if (text == "boolean") {
    return 0x02u;
  }
  if (text == "analog") {
    return 0x01u;
  }
  try {
    const auto v = std::stoul(text, nullptr, 0);
    if (v <= 0x1Fu) {
      return static_cast<std::uint8_t>(v);
    }
  } catch (const std::exception &) {
  }
  return 0xFFu;
}

struct ParseContext
{
  PoolLoadResult result{};
  std::unordered_map<std::string, int> topic_name_counts{};
  std::vector<std::string> parse_element_stack{};
  std::vector<std::uint16_t> parse_id_stack{};
};

thread_local ParseContext * active_context = nullptr;
}  // namespace

void VTPoolModel::clear()
{
  // Reset full model state before a new XML load attempt.
  pool_bytes_.clear();
  bindings_.clear();
  data_masks_.clear();
  soft_key_masks_.clear();
  by_id_.clear();
  numeric_by_token_.clear();
  working_set_id_ = 0;
}

void VTPoolModel::apply_load_result(PoolLoadResult && r)
{
  // Commit parsed model snapshot atomically to this instance.
  pool_bytes_ = std::move(r.pool_bytes);
  bindings_ = std::move(r.bindings);
  data_masks_ = std::move(r.data_masks);
  soft_key_masks_ = std::move(r.soft_key_masks);
  by_id_ = std::move(r.by_id);
  numeric_by_token_ = std::move(r.numeric_by_token);
  working_set_id_ = r.working_set_id;
}

bool VTPoolModel::load_from_xml(const std::string & xml_path, const BuildConfig & cfg)
{
  // Build object pool bytes from PoolEdit XML using legacy parser backend.
  // Parser callbacks populate bindings/masks/lookup structures in ParseContext.
  PoolLoadResult out{};
  FILE * file = std::fopen(xml_path.c_str(), "r");
  if (file == nullptr) {
    clear();
    return false;
  }

  ParseContext ctx{};
  active_context = &ctx;
  parse(
    file, &VTPoolModel::parser_start_cb, &VTPoolModel::parser_end_cb, &VTPoolModel::parser_ready_cb,
    cfg.vt_dimension, cfg.vt_softkey_width, cfg.vt_softkey_height, cfg.vt_colors);
  active_context = nullptr;
  std::fclose(file);

  rebuild_lookup(ctx.result.bindings, ctx.result.by_id, ctx.result.numeric_by_token);
  ctx.result.success = !ctx.result.pool_bytes.empty();
  apply_load_result(std::move(ctx.result));
  return !pool_bytes_.empty();
}

std::string VTPoolModel::sanitize_topic_token(const std::string & text)
{
  // Convert XML name to stable ROS topic token [a-z0-9_].
  if (text.empty()) {
    return "unnamed";
  }

  std::string out;
  out.reserve(text.size());
  for (char c : text) {
    const auto uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc)) {
      out.push_back(static_cast<char>(std::tolower(uc)));
    } else {
      out.push_back('_');
    }
  }

  while (!out.empty() && out.front() == '_') {
    out.erase(out.begin());
  }
  while (!out.empty() && out.back() == '_') {
    out.pop_back();
  }
  if (out.empty()) {
    out = "unnamed";
  }
  return out;
}

std::string VTPoolModel::attr_value(const char ** attr, const char * key)
{
  // Read XML attribute value by key from expat-style key/value array.
  for (std::size_t i = 0; attr[i] != nullptr && attr[i + 1] != nullptr; i += 2) {
    if (std::string(attr[i]) == key) {
      return std::string(attr[i + 1]);
    }
  }
  return "";
}

void VTPoolModel::add_binding(
  std::vector<ObjectBinding> & bindings,
  std::unordered_map<std::string, int> & topic_name_counts,
  ObjectKind kind, const std::string & tag, const std::string & name, std::uint16_t object_id,
  std::uint16_t parent_object_id)
{
  // Register one object binding with deduplicated topic token.
  ObjectBinding b;
  b.kind = kind;
  b.element_tag = tag;
  b.element_name = name.empty() ? "unnamed" : name;

  const std::string base = sanitize_topic_token(b.element_name);
  int & count = topic_name_counts[base];
  if (count == 0) {
    b.topic_token = base;
  } else {
    b.topic_token = base + "_" + std::to_string(count);
  }
  ++count;

  b.object_id = object_id;
  b.parent_object_id = parent_object_id;
  b.aux_function_type_id = 0xFFu;
  bindings.push_back(std::move(b));
}

void VTPoolModel::add_mask_binding(
  std::vector<MaskBinding> & out,
  std::unordered_map<std::string, int> & topic_name_counts,
  const std::string & name, std::uint16_t object_id)
{
  // Register one mask binding with deduplicated topic token.
  MaskBinding b;
  b.element_name = name.empty() ? "unnamed" : name;
  const std::string base = sanitize_topic_token(b.element_name);
  int & count = topic_name_counts[base];
  if (count == 0) {
    b.topic_token = base;
  } else {
    b.topic_token = base + "_" + std::to_string(count);
  }
  ++count;
  b.object_id = object_id;
  out.push_back(std::move(b));
}

void VTPoolModel::rebuild_lookup(
  const std::vector<ObjectBinding> & bindings,
  std::unordered_map<std::uint16_t, std::size_t> & by_id,
  std::unordered_map<std::string, std::size_t> & numeric_by_token)
{
  // Rebuild fast lookup maps from canonical binding vector.
  by_id.clear();
  numeric_by_token.clear();

  for (std::size_t i = 0; i < bindings.size(); ++i) {
    const auto & b = bindings[i];
    if (b.object_id != 0) {
      by_id[b.object_id] = i;
    }
    if (b.kind == ObjectKind::NumberVariable) {
      numeric_by_token[b.topic_token] = i;
    }
  }
}

void VTPoolModel::parser_start_cb(void * data, char * el, const char ** attr)
{
  // XML start-element callback used during pool model extraction.
  (void)data;
  if (active_context == nullptr || el == nullptr || attr == nullptr) {
    return;
  }

  const std::string tag(el);
  std::uint16_t object_id = 0;
  const std::string id_text = attr_value(attr, "id");
  if (!id_text.empty()) {
    try {
      object_id = static_cast<std::uint16_t>(std::stoul(id_text));
    } catch (const std::exception &) {
      object_id = 0;
    }
  }
  const std::uint16_t parent_object_id =
    active_context->parse_id_stack.empty() ? 0 : active_context->parse_id_stack.back();
  active_context->parse_element_stack.push_back(tag);
  active_context->parse_id_stack.push_back(object_id);

  // Object category mapping from PoolEdit XML tags to runtime binding kind.
  if (tag == "workingset") {
    active_context->result.working_set_id = object_id;
    return;
  }
  if (tag == "datamask") {
    const std::string name = attr_value(attr, "name");
    add_mask_binding(active_context->result.data_masks, active_context->topic_name_counts, name, object_id);
    return;
  }
  if (tag == "softkeymask") {
    const std::string name = attr_value(attr, "name");
    add_mask_binding(active_context->result.soft_key_masks, active_context->topic_name_counts, name, object_id);
    return;
  }

  const std::string name = attr_value(attr, "name");
  if (tag == "button") {
    add_binding(
      active_context->result.bindings, active_context->topic_name_counts, ObjectKind::Button, tag,
      name, object_id, parent_object_id);
  } else if (tag == "key") {
    add_binding(
      active_context->result.bindings, active_context->topic_name_counts, ObjectKind::SoftKey, tag,
      name, object_id, parent_object_id);
  } else if (tag == "container") {
    add_binding(
      active_context->result.bindings, active_context->topic_name_counts, ObjectKind::Container, tag,
      name, object_id, parent_object_id);
  } else if (tag == "stringvariable") {
    add_binding(
      active_context->result.bindings, active_context->topic_name_counts, ObjectKind::StringVariable, tag,
      name, object_id, parent_object_id);
  } else if (tag == "inputstring") {
    add_binding(
      active_context->result.bindings, active_context->topic_name_counts, ObjectKind::InputString, tag,
      name, object_id, parent_object_id);
  } else if (tag == "inputnumber") {
    add_binding(
      active_context->result.bindings, active_context->topic_name_counts, ObjectKind::InputNumber, tag,
      name, object_id, parent_object_id);
  } else if (tag == "inputlist") {
    add_binding(
      active_context->result.bindings, active_context->topic_name_counts, ObjectKind::InputList, tag,
      name, object_id, parent_object_id);
  } else if (tag == "outputlist") {
    add_binding(
      active_context->result.bindings, active_context->topic_name_counts, ObjectKind::OutputList, tag,
      name, object_id, parent_object_id);
  } else if (tag == "numbervariable") {
    add_binding(
      active_context->result.bindings, active_context->topic_name_counts, ObjectKind::NumberVariable, tag,
      name, object_id, parent_object_id);
  } else if (tag == "inputboolean") {
    add_binding(
      active_context->result.bindings, active_context->topic_name_counts, ObjectKind::InputBoolean, tag,
      name, object_id, parent_object_id);
  } else if (tag == "auxiliaryfunction") {
    add_binding(
      active_context->result.bindings, active_context->topic_name_counts, ObjectKind::AuxFunction, tag,
      name, object_id, parent_object_id);
    if (!active_context->result.bindings.empty()) {
      active_context->result.bindings.back().aux_function_type_id =
        parse_aux_function_type_id(attr_value(attr, "function_type"));
    }
  }
}

void VTPoolModel::parser_end_cb(void * data, char * el)
{
  // XML end-element callback keeps parent/object stacks aligned.
  (void)data;
  (void)el;
  if (active_context == nullptr || active_context->parse_element_stack.empty() ||
      active_context->parse_id_stack.empty()) {
    return;
  }
  active_context->parse_element_stack.pop_back();
  active_context->parse_id_stack.pop_back();
}

void VTPoolModel::parser_ready_cb(char * data, int length)
{
  // Raw object pool record stream produced by parser backend.
  if (active_context == nullptr || data == nullptr || length <= 0) {
    return;
  }
  active_context->result.pool_bytes.insert(active_context->result.pool_bytes.end(), data, data + length);
}

}  // namespace ros2_isobus
