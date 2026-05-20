VT Client Interface Helper (`vt_client_interface.hpp`)
======================================================

Purpose
- Provides high-level C++ wrappers for nodes that communicate with `vt_client_node`.
- Hides raw ROS topic publish/subscribe details behind typed classes and methods.

Header
- `include/ros2_isobus/vt_client_interface.hpp`

Classes
- `ros2_isobus::VTContainer`
  - Container visibility control only.
  - Topics: `ISOBUS/vt/container/<name_token>/visible/(set|value|result)`.
  - Methods: `set_visible(bool)`, `visible()`, `visible_result()`.
- `ros2_isobus::VTNumber`
  - Number value set/get via `ISOBUS/vt/number/<name_token>/(set|value)`.
- `ros2_isobus::VTInputNumber`
  - Input-number value set/get via `ISOBUS/vt/input_number/<name_token>/(set|value)`.
  - Enable control: `ISOBUS/vt/input_number/<name_token>/enabled/(set|value|result)`.
  - Methods: `set(double)`, `value()`, `on_change(...)`, `set_enabled(bool)`, `enabled()`, `enabled_result()`.
- `ros2_isobus::VTString`
  - String value set/get via `ISOBUS/vt/string/<name_token>/(set|value)`.
- `ros2_isobus::VTInputString`
  - Input-string value set/get via `ISOBUS/vt/input_string/<name_token>/(set|value)`.
  - Enable control: `ISOBUS/vt/input_string/<name_token>/enabled/(set|value|result)`.
  - Methods: `set(string)`, `value()`, `on_change(...)`, `set_enabled(bool)`, `enabled()`, `enabled_result()`.
- `ros2_isobus::VTInputBoolean`
  - Input-boolean value set/get via `ISOBUS/vt/input_bool/<name_token>/(set|value)`.
  - Enable control: `ISOBUS/vt/input_bool/<name_token>/enabled/(set|value|result)`.
  - Methods: `set(bool)`, `value()`, `on_change(...)`, `set_enabled(bool)`, `enabled()`, `enabled_result()`.
- `ros2_isobus::VTList`
  - List index set/get via `ISOBUS/vt/list/<name_token>/(set|value)`.
- `ros2_isobus::VTButton`
  - Event input via `ISOBUS/vt/event/button/<name_token>`.
  - Enable control: `ISOBUS/vt/button/<name_token>/enabled/(set|value|result)`.
  - Callback registration:
    - `on_event(std::function<void(bool)>)` for press/release.
    - `on_pressed(std::function<void()>)` for press only (`code=0x01`).
    - `on_released(std::function<void()>)` for release only (`code=0x00`).
- `ros2_isobus::VTSoftkey`
  - Event input via `ISOBUS/vt/event/softkey/<name_token>`.
  - Same callback API as `VTButton`.
- `ros2_isobus::VTWorkingSet`
  - Working-set level communication:
    - active/softkey mask set/value/result
    - status, session state
    - pointing/navigation events
    - diagnostics
    - runtime pool updates (`send_pool_update_xml(...)`)
    - runtime pool update result callback (`on_updated(...)`)
  - Supports both:
    - named trigger topics (`trigger_active_mask("mask_name")`)

Pool Creation And XML Usage
1. Create/edit the VT object pool with PoolEdit.
2. Export XML from PoolEdit (`Export XML`).
3. Configure that XML path as `vt_client_node` parameter `xml_file`.
4. Use XML `name` values as wrapper `name_token` constructor arguments.

Modeling Recommendation
- Prefer `numbervariable` / `stringvariable` objects for general data flow:
  - Input object references variable.
  - Output object references the same variable.
- You can also use input objects directly without variables via:
  - `VTInputNumber`
  - `VTInputString`
  - `VTInputBoolean`
- Direct input wrappers are especially useful when you need runtime `enabled` control.

Usage Example
```cpp
#include "ros2_isobus/vt_client_interface.hpp"

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node"),
             label_(*this, "label0"),
             panel_(*this, "main_panel"),
             ok_button_(*this, "ok_button"),
             ws_(*this) {
    ok_button_.on_pressed([this]() {
      label_.set("OK");
      panel_.set_visible(true);
    });

    ws_.trigger_active_mask("main_mask");
    ws_.on_updated([this](const ros2_isobus::msg::VTUpdateResult & r) {
      RCLCPP_INFO(
        get_logger(),
        "update: success=%s detail=%s pending=%u in_progress=%u",
        r.success ? "true" : "false",
        r.detail.c_str(),
        static_cast<unsigned>(r.pending_updates),
        static_cast<unsigned>(r.in_progress_updates));
    });
  }

private:
  ros2_isobus::VTString label_;
  ros2_isobus::VTContainer panel_;
  ros2_isobus::VTButton ok_button_;
  ros2_isobus::VTWorkingSet ws_;
};
```

Input Wrapper Example
```cpp
#include "ros2_isobus/vt_client_interface.hpp"

class VtInputDemo : public rclcpp::Node {
public:
  VtInputDemo() : Node("vt_input_demo"),
                  in_num_(*this, "counter_input"),
                  in_str_(*this, "text_input"),
                  in_bool_(*this, "bool_input") {
    in_num_.on_change([this](double v) {
      RCLCPP_INFO(get_logger(), "counter_input=%.0f", v);
    });
    in_str_.on_change([this](const std::string & s) {
      RCLCPP_INFO(get_logger(), "text_input=%s", s.c_str());
    });
    in_bool_.on_change([this](bool b) {
      RCLCPP_INFO(get_logger(), "bool_input=%s", b ? "true" : "false");
    });

    // Example: disable all input fields
    in_num_.set_enabled(false);
    in_str_.set_enabled(false);
    in_bool_.set_enabled(false);
  }

private:
  ros2_isobus::VTInputNumber in_num_;
  ros2_isobus::VTInputString in_str_;
  ros2_isobus::VTInputBoolean in_bool_;
};
```

Pool Update Example (`send_pool_update_xml`)
```cpp
#include "ros2_isobus/vt_client_interface.hpp"

class VtPoolUpdateDemo : public rclcpp::Node {
public:
  VtPoolUpdateDemo() : Node("vt_pool_update_demo"), ws_(*this) {
    ws_.on_updated([this](const ros2_isobus::msg::VTUpdateResult & r) {
      RCLCPP_INFO(
        get_logger(),
        "pool update: success=%s detail=%s pending=%u in_progress=%u",
        r.success ? "true" : "false",
        r.detail.c_str(),
        static_cast<unsigned>(r.pending_updates),
        static_cast<unsigned>(r.in_progress_updates));
    });

    // Update one object (single XML element).
    ws_.send_pool_update_xml(
      "<polygon height=\"200\" id=\"290\" name=\"link3\" polygon_type=\"convex\" use=\"mask\" width=\"200\">"
      "<fillattributes fill_colour=\"yellow\" fill_type=\"fillcolour\" id=\"291\" name=\"yellow\" role=\"fill_attributes\" use=\"mask\"/>"
      "<lineattributes id=\"292\" line_art=\"1111111111111111\" line_colour=\"black\" line_width=\"1\" name=\"black1\" role=\"line_attributes\" use=\"mask\"/>"
      "<point pos_x=\"90\" pos_y=\"87\"/>"
      "<point pos_x=\"140\" pos_y=\"72\"/>"
      "<point pos_x=\"148\" pos_y=\"97\"/>"
      "<point pos_x=\"97\" pos_y=\"112\"/>"
      "</polygon>");

    // Recommended: group related updates into one payload.
    ws_.send_pool_update_xml(
      "<objectpool>"
      "<polygon height=\"200\" id=\"287\" name=\"link2\" polygon_type=\"convex\" use=\"mask\" width=\"200\">"
      "<fillattributes fill_colour=\"yellow\" fill_type=\"fillcolour\" id=\"288\" name=\"yello\" role=\"fill_attributes\" use=\"mask\"/>"
      "<lineattributes id=\"289\" line_art=\"1111111111111111\" line_colour=\"black\" line_width=\"1\" name=\"black1\" role=\"line_attributes\" use=\"mask\"/>"
      "<point pos_x=\"30\" pos_y=\"68\"/>"
      "<point pos_x=\"36\" pos_y=\"53\"/>"
      "<point pos_x=\"94\" pos_y=\"80\"/>"
      "<point pos_x=\"88\" pos_y=\"95\"/>"
      "</polygon>"
      "<polygon height=\"200\" id=\"290\" name=\"link3\" polygon_type=\"convex\" use=\"mask\" width=\"200\">"
      "<fillattributes fill_colour=\"yellow\" fill_type=\"fillcolour\" id=\"291\" name=\"yellow\" role=\"fill_attributes\" use=\"mask\"/>"
      "<lineattributes id=\"292\" line_art=\"1111111111111111\" line_colour=\"black\" line_width=\"1\" name=\"black1\" role=\"line_attributes\" use=\"mask\"/>"
      "<point pos_x=\"90\" pos_y=\"87\"/>"
      "<point pos_x=\"140\" pos_y=\"72\"/>"
      "<point pos_x=\"148\" pos_y=\"97\"/>"
      "<point pos_x=\"97\" pos_y=\"112\"/>"
      "</polygon>"
      "</objectpool>");
  }

private:
  ros2_isobus::VTWorkingSet ws_;
};
```

Notes
- Each object class instance is bound to one XML `name_token`.
- Values/results are cached internally and protected with mutexes.
- This helper assumes `vt_client_node` is running and has created matching topics from loaded XML.
- `VTWorkingSet::send_pool_update_xml(...)` can update one or many objects in one XML payload.
- You can call `send_pool_update_xml(...)` multiple times; updates are serialized internally by `vt_client_node`.
- Recommended: put all related object changes into one XML payload when possible.
