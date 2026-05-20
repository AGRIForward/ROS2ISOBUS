/*
 *  This file is part of ROS2ISOBUS
 *
 *  Copyright 2026 Juha Backman / Natural Resources Institute Finland
 *
 *  ROS2ISOBUS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or (at your option) any later version.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_isobus/msg/command_result.hpp"
#include "ros2_isobus/msg/vt_navigation_event.hpp"
#include "ros2_isobus/msg/vt_pointing_event.hpp"
#include "ros2_isobus/msg/vt_session_state.hpp"
#include "ros2_isobus/msg/vt_status.hpp"
#include "ros2_isobus/msg/vt_update_result.hpp"
#include "ros2_isobus/msg/vt_aux_assignment.hpp"
#include "ros2_isobus/msg/vt_aux_input_raw.hpp"
#include "ros2_isobus/msg/vt_aux_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace ros2_isobus
{

// VTContainer wraps container visibility control:
// - publishes:  ISOBUS/vt/container/<name_token>/visible/set
// - subscribes: ISOBUS/vt/container/<name_token>/visible/value|result
class VTContainer
{
public:
  // Construct container wrapper for one XML name token.
  VTContainer(rclcpp::Node & node, std::string name_token)
  : node_(node), name_token_(std::move(name_token)), base_topic_("ISOBUS/vt/container/" + name_token_)
  {
    visible_set_pub_ = node_.create_publisher<std_msgs::msg::Bool>(base_topic_ + "/visible/set", rclcpp::QoS(10));
    visible_value_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
      base_topic_ + "/visible/value", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        visible_ = msg->data;
      });
    visible_result_sub_ = node_.create_subscription<ros2_isobus::msg::CommandResult>(
      base_topic_ + "/visible/result", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::CommandResult::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        visible_result_ = *msg;
      });
  }

  // Return the XML token used in this wrapper's topic paths.
  const std::string & name_token() const { return name_token_; }

  // Publish visible=true/false request to VT.
  void set_visible(bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    visible_set_pub_->publish(msg);
  }

  // Latest visible state mirrored from VT value topic.
  bool visible() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return visible_;
  }

  // Latest command result for visible set command.
  ros2_isobus::msg::CommandResult visible_result() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return visible_result_;
  }

private:
  rclcpp::Node & node_;
  std::string name_token_;
  std::string base_topic_;
  mutable std::mutex mutex_;

  bool visible_{false};
  ros2_isobus::msg::CommandResult visible_result_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visible_set_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visible_value_sub_;
  rclcpp::Subscription<ros2_isobus::msg::CommandResult>::SharedPtr visible_result_sub_;
};

// VTNumber wraps NumberVariable value I/O:
// - publishes:  ISOBUS/vt/number/<token>/set
// - subscribes: ISOBUS/vt/number/<token>/value
class VTNumber
{
public:
  // Construct NumberVariable wrapper for one XML name token.
  VTNumber(rclcpp::Node & node, std::string name_token)
  : node_(node), name_token_(std::move(name_token))
  {
    set_pub_ = node_.create_publisher<std_msgs::msg::Float64>(
      "ISOBUS/vt/number/" + name_token_ + "/set", rclcpp::QoS(10));
    value_sub_ = node_.create_subscription<std_msgs::msg::Float64>(
      "ISOBUS/vt/number/" + name_token_ + "/value", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        std::vector<std::function<void(double)>> callbacks_copy;
        double value_copy = 0.0;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          value_ = msg->data;
          value_copy = value_;
          callbacks_copy = on_change_callbacks_;
        }
        for (const auto & cb : callbacks_copy) {
          cb(value_copy);
        }
      });
  }

  // Return the XML token used in this wrapper's topic paths.
  const std::string & name_token() const { return name_token_; }

  // Publish numeric value set request.
  void set(double value)
  {
    std_msgs::msg::Float64 msg;
    msg.data = value;
    set_pub_->publish(msg);
  }

  // Latest numeric value received from VT.
  double value() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

  // Register callback called on each incoming value update.
  void on_change(const std::function<void(double)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_change_callbacks_.push_back(callback);
  }

private:
  rclcpp::Node & node_;
  std::string name_token_;
  mutable std::mutex mutex_;
  double value_{0.0};
  std::vector<std::function<void(double)>> on_change_callbacks_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr set_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr value_sub_;
};

// VTInputNumber wraps Input Number value and enabled control:
// - value:    ISOBUS/vt/input_number/<token>/set|value
// - enabled:  ISOBUS/vt/input_number/<token>/enabled/set|value|result
class VTInputNumber
{
public:
  // Construct Input Number wrapper for one XML name token.
  VTInputNumber(rclcpp::Node & node, std::string name_token)
  : node_(node), name_token_(std::move(name_token))
  {
    set_pub_ = node_.create_publisher<std_msgs::msg::Float64>(
      "ISOBUS/vt/input_number/" + name_token_ + "/set", rclcpp::QoS(10));
    value_sub_ = node_.create_subscription<std_msgs::msg::Float64>(
      "ISOBUS/vt/input_number/" + name_token_ + "/value", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        std::vector<std::function<void(double)>> callbacks_copy;
        double value_copy = 0.0;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          value_ = msg->data;
          value_copy = value_;
          callbacks_copy = on_change_callbacks_;
        }
        for (const auto & cb : callbacks_copy) {
          cb(value_copy);
        }
      });
    enabled_set_pub_ = node_.create_publisher<std_msgs::msg::Bool>(
      "ISOBUS/vt/input_number/" + name_token_ + "/enabled/set", rclcpp::QoS(10));
    enabled_value_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
      "ISOBUS/vt/input_number/" + name_token_ + "/enabled/value", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_ = msg->data;
      });
    enabled_result_sub_ = node_.create_subscription<ros2_isobus::msg::CommandResult>(
      "ISOBUS/vt/input_number/" + name_token_ + "/enabled/result", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::CommandResult::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_result_ = *msg;
      });
  }

  // Return the XML token used in this wrapper's topic paths.
  const std::string & name_token() const { return name_token_; }

  // Publish input number value set request.
  void set(double value)
  {
    std_msgs::msg::Float64 msg;
    msg.data = value;
    set_pub_->publish(msg);
  }

  // Latest input number value received from VT.
  double value() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

  // Register callback called on each incoming input value update.
  void on_change(const std::function<void(double)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_change_callbacks_.push_back(callback);
  }

  // Publish enabled=true/false request for this input object.
  void set_enabled(bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    enabled_set_pub_->publish(msg);
  }

  // Latest enabled state mirrored from VT value topic.
  bool enabled() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
  }

  // Latest command result for enabled set command.
  ros2_isobus::msg::CommandResult enabled_result() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_result_;
  }

private:
  rclcpp::Node & node_;
  std::string name_token_;
  mutable std::mutex mutex_;
  double value_{0.0};
  bool enabled_{false};
  ros2_isobus::msg::CommandResult enabled_result_;
  std::vector<std::function<void(double)>> on_change_callbacks_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr set_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr value_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_set_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_value_sub_;
  rclcpp::Subscription<ros2_isobus::msg::CommandResult>::SharedPtr enabled_result_sub_;
};

// VTString wraps StringVariable value I/O:
// - publishes:  ISOBUS/vt/string/<token>/set
// - subscribes: ISOBUS/vt/string/<token>/value
class VTString
{
public:
  // Construct StringVariable wrapper for one XML name token.
  VTString(rclcpp::Node & node, std::string name_token)
  : node_(node), name_token_(std::move(name_token))
  {
    set_pub_ = node_.create_publisher<std_msgs::msg::String>(
      "ISOBUS/vt/string/" + name_token_ + "/set", rclcpp::QoS(10));
    value_sub_ = node_.create_subscription<std_msgs::msg::String>(
      "ISOBUS/vt/string/" + name_token_ + "/value", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::vector<std::function<void(const std::string &)>> callbacks_copy;
        std::string value_copy;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          value_ = msg->data;
          value_copy = value_;
          callbacks_copy = on_change_callbacks_;
        }
        for (const auto & cb : callbacks_copy) {
          cb(value_copy);
        }
      });
  }

  // Return the XML token used in this wrapper's topic paths.
  const std::string & name_token() const { return name_token_; }

  // Publish string value set request.
  void set(const std::string & value)
  {
    std_msgs::msg::String msg;
    msg.data = value;
    set_pub_->publish(msg);
  }

  // Latest string value received from VT.
  std::string value() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

  // Register callback called on each incoming value update.
  void on_change(const std::function<void(const std::string &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_change_callbacks_.push_back(callback);
  }

private:
  rclcpp::Node & node_;
  std::string name_token_;
  mutable std::mutex mutex_;
  std::string value_;
  std::vector<std::function<void(const std::string &)>> on_change_callbacks_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr value_sub_;
};

// VTInputString wraps Input String value and enabled control:
// - value:    ISOBUS/vt/input_string/<token>/set|value
// - enabled:  ISOBUS/vt/input_string/<token>/enabled/set|value|result
class VTInputString
{
public:
  // Construct Input String wrapper for one XML name token.
  VTInputString(rclcpp::Node & node, std::string name_token)
  : node_(node), name_token_(std::move(name_token))
  {
    set_pub_ = node_.create_publisher<std_msgs::msg::String>(
      "ISOBUS/vt/input_string/" + name_token_ + "/set", rclcpp::QoS(10));
    value_sub_ = node_.create_subscription<std_msgs::msg::String>(
      "ISOBUS/vt/input_string/" + name_token_ + "/value", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::vector<std::function<void(const std::string &)>> callbacks_copy;
        std::string value_copy;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          value_ = msg->data;
          value_copy = value_;
          callbacks_copy = on_change_callbacks_;
        }
        for (const auto & cb : callbacks_copy) {
          cb(value_copy);
        }
      });
    enabled_set_pub_ = node_.create_publisher<std_msgs::msg::Bool>(
      "ISOBUS/vt/input_string/" + name_token_ + "/enabled/set", rclcpp::QoS(10));
    enabled_value_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
      "ISOBUS/vt/input_string/" + name_token_ + "/enabled/value", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_ = msg->data;
      });
    enabled_result_sub_ = node_.create_subscription<ros2_isobus::msg::CommandResult>(
      "ISOBUS/vt/input_string/" + name_token_ + "/enabled/result", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::CommandResult::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_result_ = *msg;
      });
  }

  // Return the XML token used in this wrapper's topic paths.
  const std::string & name_token() const { return name_token_; }

  // Publish input string value set request.
  void set(const std::string & value)
  {
    std_msgs::msg::String msg;
    msg.data = value;
    set_pub_->publish(msg);
  }

  // Latest input string value received from VT.
  std::string value() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

  // Register callback called on each incoming input value update.
  void on_change(const std::function<void(const std::string &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_change_callbacks_.push_back(callback);
  }

  // Publish enabled=true/false request for this input object.
  void set_enabled(bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    enabled_set_pub_->publish(msg);
  }

  // Latest enabled state mirrored from VT value topic.
  bool enabled() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
  }

  // Latest command result for enabled set command.
  ros2_isobus::msg::CommandResult enabled_result() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_result_;
  }

private:
  rclcpp::Node & node_;
  std::string name_token_;
  mutable std::mutex mutex_;
  std::string value_;
  bool enabled_{false};
  ros2_isobus::msg::CommandResult enabled_result_;
  std::vector<std::function<void(const std::string &)>> on_change_callbacks_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr value_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_set_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_value_sub_;
  rclcpp::Subscription<ros2_isobus::msg::CommandResult>::SharedPtr enabled_result_sub_;
};

// VTInputBoolean wraps Input Boolean value and enabled control:
// - value:    ISOBUS/vt/input_bool/<token>/set|value
// - enabled:  ISOBUS/vt/input_bool/<token>/enabled/set|value|result
class VTInputBoolean
{
public:
  // Construct Input Boolean wrapper for one XML name token.
  VTInputBoolean(rclcpp::Node & node, std::string name_token)
  : node_(node), name_token_(std::move(name_token))
  {
    set_pub_ = node_.create_publisher<std_msgs::msg::Bool>(
      "ISOBUS/vt/input_bool/" + name_token_ + "/set", rclcpp::QoS(10));
    value_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
      "ISOBUS/vt/input_bool/" + name_token_ + "/value", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::vector<std::function<void(bool)>> callbacks_copy;
        bool value_copy = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          value_ = msg->data;
          value_copy = value_;
          callbacks_copy = on_change_callbacks_;
        }
        for (const auto & cb : callbacks_copy) {
          cb(value_copy);
        }
      });
    enabled_set_pub_ = node_.create_publisher<std_msgs::msg::Bool>(
      "ISOBUS/vt/input_bool/" + name_token_ + "/enabled/set", rclcpp::QoS(10));
    enabled_value_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
      "ISOBUS/vt/input_bool/" + name_token_ + "/enabled/value", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_ = msg->data;
      });
    enabled_result_sub_ = node_.create_subscription<ros2_isobus::msg::CommandResult>(
      "ISOBUS/vt/input_bool/" + name_token_ + "/enabled/result", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::CommandResult::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_result_ = *msg;
      });
  }

  // Return the XML token used in this wrapper's topic paths.
  const std::string & name_token() const { return name_token_; }

  // Publish input boolean value set request.
  void set(bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    set_pub_->publish(msg);
  }

  // Latest input boolean value received from VT.
  bool value() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

  // Register callback called on each incoming input value update.
  void on_change(const std::function<void(bool)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_change_callbacks_.push_back(callback);
  }

  // Publish enabled=true/false request for this input object.
  void set_enabled(bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    enabled_set_pub_->publish(msg);
  }

  // Latest enabled state mirrored from VT value topic.
  bool enabled() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
  }

  // Latest command result for enabled set command.
  ros2_isobus::msg::CommandResult enabled_result() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_result_;
  }

private:
  rclcpp::Node & node_;
  std::string name_token_;
  mutable std::mutex mutex_;
  bool value_{false};
  bool enabled_{false};
  ros2_isobus::msg::CommandResult enabled_result_;
  std::vector<std::function<void(bool)>> on_change_callbacks_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr set_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr value_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_set_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_value_sub_;
  rclcpp::Subscription<ros2_isobus::msg::CommandResult>::SharedPtr enabled_result_sub_;
};

// VTList wraps Input/Output list index I/O:
// - publishes:  ISOBUS/vt/list/<token>/set
// - subscribes: ISOBUS/vt/list/<token>/value
class VTList
{
public:
  // Construct list wrapper for one XML name token.
  VTList(rclcpp::Node & node, std::string name_token)
  : node_(node), name_token_(std::move(name_token))
  {
    set_pub_ = node_.create_publisher<std_msgs::msg::Int32>(
      "ISOBUS/vt/list/" + name_token_ + "/set", rclcpp::QoS(10));
    value_sub_ = node_.create_subscription<std_msgs::msg::Int32>(
      "ISOBUS/vt/list/" + name_token_ + "/value", rclcpp::QoS(10),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        value_ = msg->data;
      });
  }

  // Return the XML token used in this wrapper's topic paths.
  const std::string & name_token() const { return name_token_; }

  // Publish list index set request.
  void set(std::int32_t value)
  {
    std_msgs::msg::Int32 msg;
    msg.data = value;
    set_pub_->publish(msg);
  }

  // Latest list index received from VT.
  std::int32_t value() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

private:
  rclcpp::Node & node_;
  std::string name_token_;
  mutable std::mutex mutex_;
  std::int32_t value_{0};
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr set_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr value_sub_;
};

// VTButton wraps button events and optional enabled control:
// - event:     ISOBUS/vt/event/button/<token> (UInt8 activation code)
// - enabled:   ISOBUS/vt/button/<token>/enabled/set|value|result
// Event callbacks:
// - on_event(bool): all button events (pressed/released/held mapped to bool)
// - on_pressed(): activation code 0x01
// - on_released(): activation code 0x00
class VTButton
{
public:
  // Construct button wrapper for one XML name token.
  VTButton(rclcpp::Node & node, std::string name_token)
  : node_(node), name_token_(std::move(name_token))
  {
    event_sub_ = node_.create_subscription<std_msgs::msg::UInt8>(
      "ISOBUS/vt/event/button/" + name_token_, rclcpp::QoS(10),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        const bool pressed = (msg->data != 0x00u);
        std::vector<std::function<void(bool)>> callbacks_copy;
        std::vector<std::function<void()>> pressed_copy;
        std::vector<std::function<void()>> released_copy;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_pressed_ = pressed;
          callbacks_copy = on_event_callbacks_;
          if (msg->data == 0x01u) {
            pressed_copy = on_pressed_callbacks_;
          } else if (msg->data == 0x00u) {
            released_copy = on_released_callbacks_;
          }
        }
        for (const auto & cb : callbacks_copy) {
          cb(pressed);
        }
        for (const auto & cb : pressed_copy) {
          cb();
        }
        for (const auto & cb : released_copy) {
          cb();
        }
      });
    enabled_set_pub_ = node_.create_publisher<std_msgs::msg::Bool>(
      "ISOBUS/vt/button/" + name_token_ + "/enabled/set", rclcpp::QoS(10));
    enabled_value_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
      "ISOBUS/vt/button/" + name_token_ + "/enabled/value", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_ = msg->data;
      });
    enabled_result_sub_ = node_.create_subscription<ros2_isobus::msg::CommandResult>(
      "ISOBUS/vt/button/" + name_token_ + "/enabled/result", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::CommandResult::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_result_ = *msg;
      });
  }

  // Return the XML token used in this wrapper's topic paths.
  const std::string & name_token() const { return name_token_; }

  // Register callback for all button activation events (0/1/2 mapped to bool).
  void on_event(const std::function<void(bool)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_event_callbacks_.push_back(callback);
  }

  // Register callback for activation code 0x01 (pressed).
  void on_pressed(const std::function<void()> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_pressed_callbacks_.push_back(callback);
  }

  // Register callback for activation code 0x00 (released).
  void on_released(const std::function<void()> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_released_callbacks_.push_back(callback);
  }

  // Latest pressed state (true for non-zero activation code).
  bool pressed() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_pressed_;
  }

  // Publish enabled=true/false request for this button object.
  void set_enabled(bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    enabled_set_pub_->publish(msg);
  }

  // Latest enabled state mirrored from VT value topic.
  bool enabled() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
  }

  // Latest command result for enabled set command.
  ros2_isobus::msg::CommandResult enabled_result() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_result_;
  }

private:
  rclcpp::Node & node_;
  std::string name_token_;
  mutable std::mutex mutex_;
  bool last_pressed_{false};
  bool enabled_{false};
  ros2_isobus::msg::CommandResult enabled_result_;
  std::vector<std::function<void(bool)>> on_event_callbacks_;
  std::vector<std::function<void()>> on_pressed_callbacks_;
  std::vector<std::function<void()>> on_released_callbacks_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr event_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_set_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_value_sub_;
  rclcpp::Subscription<ros2_isobus::msg::CommandResult>::SharedPtr enabled_result_sub_;
};

// VTSoftkey wraps softkey events:
// - event: ISOBUS/vt/event/softkey/<token> (UInt8 activation code)
// Event callbacks follow the same semantics as VTButton.
class VTSoftkey
{
public:
  // Construct softkey wrapper for one XML name token.
  VTSoftkey(rclcpp::Node & node, std::string name_token)
  : node_(node), name_token_(std::move(name_token))
  {
    event_sub_ = node_.create_subscription<std_msgs::msg::UInt8>(
      "ISOBUS/vt/event/softkey/" + name_token_, rclcpp::QoS(10),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        const bool pressed = (msg->data != 0x00u);
        std::vector<std::function<void(bool)>> callbacks_copy;
        std::vector<std::function<void()>> pressed_copy;
        std::vector<std::function<void()>> released_copy;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_pressed_ = pressed;
          callbacks_copy = on_event_callbacks_;
          if (msg->data == 0x01u) {
            pressed_copy = on_pressed_callbacks_;
          } else if (msg->data == 0x00u) {
            released_copy = on_released_callbacks_;
          }
        }
        for (const auto & cb : callbacks_copy) {
          cb(pressed);
        }
        for (const auto & cb : pressed_copy) {
          cb();
        }
        for (const auto & cb : released_copy) {
          cb();
        }
      });
  }

  // Return the XML token used in this wrapper's topic paths.
  const std::string & name_token() const { return name_token_; }

  // Register callback for all softkey activation events (0/1/2 mapped to bool).
  void on_event(const std::function<void(bool)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_event_callbacks_.push_back(callback);
  }

  // Register callback for activation code 0x01 (pressed).
  void on_pressed(const std::function<void()> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_pressed_callbacks_.push_back(callback);
  }

  // Register callback for activation code 0x00 (released).
  void on_released(const std::function<void()> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_released_callbacks_.push_back(callback);
  }

  // Latest pressed state (true for non-zero activation code).
  bool pressed() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_pressed_;
  }

private:
  rclcpp::Node & node_;
  std::string name_token_;
  mutable std::mutex mutex_;
  bool last_pressed_{false};
  std::vector<std::function<void(bool)>> on_event_callbacks_;
  std::vector<std::function<void()>> on_pressed_callbacks_;
  std::vector<std::function<void()>> on_released_callbacks_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr event_sub_;
};

// VTAuxFunction<T> wraps AUX-N function telemetry and assignment topics:
// - input/value:        ISOBUS/vt/aux/<token>/input/value (Float64 normalized)
// - input/raw:          ISOBUS/vt/aux/<token>/input/raw
// - assignment/value:   ISOBUS/vt/aux/<token>/assignment/value
// - assignment/result:  ISOBUS/vt/aux/<token>/assignment/result
// Template TValue controls typed conversion for input/value callback:
// - bool, integer and floating-point variants are supported.
template<typename TValue = double>
class VTAuxFunction
{
public:
  // Construct AUX function wrapper for one XML name token.
  VTAuxFunction(rclcpp::Node & node, std::string name_token)
  : node_(node), name_token_(std::move(name_token)),
    base_topic_("ISOBUS/vt/aux/" + name_token_)
  {
    input_value_sub_ = node_.create_subscription<std_msgs::msg::Float64>(
      base_topic_ + "/input/value", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        std::vector<std::function<void(TValue)>> callbacks;
        const TValue typed = convert_value(msg->data);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          input_value_ = typed;
          input_value_raw_ = msg->data;
          callbacks = on_input_callbacks_;
        }
        for (const auto & cb : callbacks) {
          cb(typed);
        }
      });
    input_raw_sub_ = node_.create_subscription<ros2_isobus::msg::VTAuxInputRaw>(
      base_topic_ + "/input/raw", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::VTAuxInputRaw::SharedPtr msg) {
        std::vector<std::function<void(const ros2_isobus::msg::VTAuxInputRaw &)>> callbacks;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          input_raw_ = *msg;
          callbacks = on_input_raw_callbacks_;
        }
        for (const auto & cb : callbacks) {
          cb(*msg);
        }
      });
    assignment_value_sub_ = node_.create_subscription<ros2_isobus::msg::VTAuxAssignment>(
      base_topic_ + "/assignment/value", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::VTAuxAssignment::SharedPtr msg) {
        std::vector<std::function<void(const ros2_isobus::msg::VTAuxAssignment &)>> callbacks;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          assignment_ = *msg;
          callbacks = on_assignment_callbacks_;
        }
        for (const auto & cb : callbacks) {
          cb(*msg);
        }
      });
    assignment_result_sub_ = node_.create_subscription<ros2_isobus::msg::CommandResult>(
      base_topic_ + "/assignment/result", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::CommandResult::SharedPtr msg) {
        std::vector<std::function<void(const ros2_isobus::msg::CommandResult &)>> callbacks;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          assignment_result_ = *msg;
          callbacks = on_assignment_result_callbacks_;
        }
        for (const auto & cb : callbacks) {
          cb(*msg);
        }
      });
  }

  // Return the XML token used in this wrapper's topic paths.
  const std::string & name_token() const { return name_token_; }

  // Latest typed AUX input value.
  TValue input_value() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return input_value_;
  }
  // Latest raw floating-point AUX input value.
  double input_value_raw() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return input_value_raw_;
  }
  // Latest raw AUX input status payload.
  ros2_isobus::msg::VTAuxInputRaw input_raw() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return input_raw_;
  }
  // Latest AUX assignment data for this function token.
  ros2_isobus::msg::VTAuxAssignment assignment() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return assignment_;
  }
  // Latest AUX assignment command result.
  ros2_isobus::msg::CommandResult assignment_result() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return assignment_result_;
  }

  // Register callback for typed AUX input updates.
  void on_input(const std::function<void(TValue)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_input_callbacks_.push_back(callback);
  }
  // Register callback for raw AUX input payload updates.
  void on_input_raw(const std::function<void(const ros2_isobus::msg::VTAuxInputRaw &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_input_raw_callbacks_.push_back(callback);
  }
  // Register callback for AUX assignment value updates.
  void on_assignment(const std::function<void(const ros2_isobus::msg::VTAuxAssignment &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_assignment_callbacks_.push_back(callback);
  }
  // Register callback for AUX assignment result updates.
  void on_assignment_result(const std::function<void(const ros2_isobus::msg::CommandResult &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_assignment_result_callbacks_.push_back(callback);
  }

private:
  // Convert normalized AUX numeric value to requested wrapper type.
  static TValue convert_value(double raw)
  {
    if constexpr (std::is_same_v<TValue, bool>) {
      return static_cast<bool>(raw);
    } else if constexpr (std::is_integral_v<TValue>) {
      return static_cast<TValue>(raw);
    } else {
      return static_cast<TValue>(raw);
    }
  }

  rclcpp::Node & node_;
  std::string name_token_;
  std::string base_topic_;
  mutable std::mutex mutex_;

  TValue input_value_{};
  double input_value_raw_{0.0};
  ros2_isobus::msg::VTAuxInputRaw input_raw_;
  ros2_isobus::msg::VTAuxAssignment assignment_;
  ros2_isobus::msg::CommandResult assignment_result_;

  std::vector<std::function<void(TValue)>> on_input_callbacks_;
  std::vector<std::function<void(const ros2_isobus::msg::VTAuxInputRaw &)>> on_input_raw_callbacks_;
  std::vector<std::function<void(const ros2_isobus::msg::VTAuxAssignment &)>> on_assignment_callbacks_;
  std::vector<std::function<void(const ros2_isobus::msg::CommandResult &)>> on_assignment_result_callbacks_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr input_value_sub_;
  rclcpp::Subscription<ros2_isobus::msg::VTAuxInputRaw>::SharedPtr input_raw_sub_;
  rclcpp::Subscription<ros2_isobus::msg::VTAuxAssignment>::SharedPtr assignment_value_sub_;
  rclcpp::Subscription<ros2_isobus::msg::CommandResult>::SharedPtr assignment_result_sub_;
};

// VTWorkingSet wraps working-set level VT control and diagnostics:
// - status/state/diagnostics subscriptions
// - pointing/navigation event subscriptions
// - runtime pool update publisher
// - active/softkey mask trigger helpers
//
// Typical usage:
// 1) Construct once per node.
// 2) Register callbacks with on_status/on_state/on_updated/... .
// 3) Call trigger_active_mask(...) and send_pool_update_xml(...) as needed.
class VTWorkingSet
{
public:
  // Construct working-set wrapper and subscribe VT status/event channels.
  explicit VTWorkingSet(rclcpp::Node & node)
  : node_(node)
  {
    status_sub_ = node_.create_subscription<ros2_isobus::msg::VTStatus>(
      "ISOBUS/vt/status", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::VTStatus::SharedPtr msg) {
        std::vector<std::function<void(const ros2_isobus::msg::VTStatus &)>> callbacks;
        std::vector<std::function<void(const ros2_isobus::msg::VTStatus &)>> timeout_callbacks;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          status_ = *msg;
          callbacks = status_callbacks_;
          if (msg->status_timeout) {
            timeout_callbacks = status_timeout_callbacks_;
          }
        }
        for (const auto & cb : callbacks) {
          cb(*msg);
        }
        for (const auto & cb : timeout_callbacks) {
          cb(*msg);
        }
      });

    state_sub_ = node_.create_subscription<ros2_isobus::msg::VTSessionState>(
      "ISOBUS/vt/session/state", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::VTSessionState::SharedPtr msg) {
        std::vector<std::function<void(const ros2_isobus::msg::VTSessionState &)>> callbacks;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          state_ = *msg;
          callbacks = state_callbacks_;
        }
        for (const auto & cb : callbacks) {
          cb(*msg);
        }
      });

    diagnostics_sub_ = node_.create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "ISOBUS/vt/diagnostics", rclcpp::QoS(10),
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        std::vector<std::function<void(const diagnostic_msgs::msg::DiagnosticArray &)>> callbacks;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          diagnostics_ = *msg;
          callbacks = diagnostics_callbacks_;
        }
        for (const auto & cb : callbacks) {
          cb(*msg);
        }
      });

    pointing_sub_ = node_.create_subscription<ros2_isobus::msg::VTPointingEvent>(
      "ISOBUS/vt/event/pointing", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::VTPointingEvent::SharedPtr msg) {
        std::vector<std::function<void(const ros2_isobus::msg::VTPointingEvent &)>> callbacks;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pointing_ = *msg;
          callbacks = pointing_callbacks_;
        }
        for (const auto & cb : callbacks) {
          cb(*msg);
        }
      });

    navigation_sub_ = node_.create_subscription<ros2_isobus::msg::VTNavigationEvent>(
      "ISOBUS/vt/event/navigation", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::VTNavigationEvent::SharedPtr msg) {
        std::vector<std::function<void(const ros2_isobus::msg::VTNavigationEvent &)>> callbacks;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          navigation_ = *msg;
          callbacks = navigation_callbacks_;
        }
        for (const auto & cb : callbacks) {
          cb(*msg);
        }
      });

    update_result_sub_ = node_.create_subscription<ros2_isobus::msg::VTUpdateResult>(
      "ISOBUS/vt/update_result", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::VTUpdateResult::SharedPtr msg) {
        std::vector<std::function<void(const ros2_isobus::msg::VTUpdateResult &)>> callbacks;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          update_result_ = *msg;
          callbacks = update_callbacks_;
        }
        for (const auto & cb : callbacks) {
          cb(*msg);
        }
      });

    aux_status_sub_ = node_.create_subscription<ros2_isobus::msg::VTAuxStatus>(
      "ISOBUS/vt/aux/status", rclcpp::QoS(10),
      [this](const ros2_isobus::msg::VTAuxStatus::SharedPtr msg) {
        std::vector<std::function<void(const ros2_isobus::msg::VTAuxStatus &)>> callbacks;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          aux_status_ = *msg;
          callbacks = aux_status_callbacks_;
        }
        for (const auto & cb : callbacks) {
          cb(*msg);
        }
      });
  }

  // Trigger active mask by XML token name.
  void trigger_active_mask(const std::string & mask_name)
  {
    auto pub = get_empty_publisher("ISOBUS/vt/active_mask/" + mask_name + "/set");
    std_msgs::msg::Empty msg;
    pub->publish(msg);
  }

  // Trigger softkey mask by XML token name.
  void trigger_softkey_mask(const std::string & mask_name)
  {
    auto pub = get_empty_publisher("ISOBUS/vt/softkey_mask/" + mask_name + "/set");
    std_msgs::msg::Empty msg;
    pub->publish(msg);
  }

  // Send runtime pool update XML to VTClient update channel.
  // Payload may be a single element or full <objectpool> document.
  void send_pool_update_xml(const std::string & xml)
  {
    std_msgs::msg::String msg;
    msg.data = xml;
    get_update_publisher()->publish(msg);
  }

  // Latest VT status snapshot.
  ros2_isobus::msg::VTStatus status() const { std::lock_guard<std::mutex> lock(mutex_); return status_; }
  // Latest VT session state snapshot.
  ros2_isobus::msg::VTSessionState state() const { std::lock_guard<std::mutex> lock(mutex_); return state_; }
  // Latest VT diagnostics snapshot.
  diagnostic_msgs::msg::DiagnosticArray diagnostics() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return diagnostics_;
  }
  // Latest pointing event snapshot.
  ros2_isobus::msg::VTPointingEvent pointing() const { std::lock_guard<std::mutex> lock(mutex_); return pointing_; }
  // Latest navigation event snapshot.
  ros2_isobus::msg::VTNavigationEvent navigation() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return navigation_;
  }
  // Latest runtime update result snapshot.
  ros2_isobus::msg::VTUpdateResult update_result() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return update_result_;
  }
  // Latest AUX status snapshot.
  ros2_isobus::msg::VTAuxStatus aux_status() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return aux_status_;
  }

  // Register callback for each incoming VT status message.
  void on_status(const std::function<void(const ros2_isobus::msg::VTStatus &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_callbacks_.push_back(callback);
  }

  // Register callback for VT status messages flagged as timeout.
  void on_status_timeout(const std::function<void(const ros2_isobus::msg::VTStatus &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_timeout_callbacks_.push_back(callback);
  }

  // Register callback for VT session state updates.
  void on_state(const std::function<void(const ros2_isobus::msg::VTSessionState &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_callbacks_.push_back(callback);
  }

  // Register callback for VT diagnostics updates.
  void on_diagnostics(const std::function<void(const diagnostic_msgs::msg::DiagnosticArray &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    diagnostics_callbacks_.push_back(callback);
  }

  // Register callback for VT pointing events.
  void on_pointing(const std::function<void(const ros2_isobus::msg::VTPointingEvent &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pointing_callbacks_.push_back(callback);
  }

  // Register callback for VT navigation events.
  void on_navigation(const std::function<void(const ros2_isobus::msg::VTNavigationEvent &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    navigation_callbacks_.push_back(callback);
  }

  // Register callback for VT runtime update result events.
  void on_updated(const std::function<void(const ros2_isobus::msg::VTUpdateResult &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    update_callbacks_.push_back(callback);
  }
  // Register callback for VT AUX status updates.
  void on_aux_status(const std::function<void(const ros2_isobus::msg::VTAuxStatus &)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    aux_status_callbacks_.push_back(callback);
  }

private:
  // Create/reuse cached Empty publisher for named trigger topics.
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr get_empty_publisher(const std::string & topic)
  {
    auto it = named_empty_set_pubs_.find(topic);
    if (it != named_empty_set_pubs_.end()) {
      return it->second;
    }
    auto pub = node_.create_publisher<std_msgs::msg::Empty>(topic, rclcpp::QoS(10));
    named_empty_set_pubs_[topic] = pub;
    return pub;
  }

  // Create/reuse XML runtime update publisher.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get_update_publisher()
  {
    if (!update_pub_) {
      update_pub_ = node_.create_publisher<std_msgs::msg::String>("ISOBUS/vt/update", rclcpp::QoS(10));
    }
    return update_pub_;
  }

  rclcpp::Node & node_;
  mutable std::mutex mutex_;

  ros2_isobus::msg::VTStatus status_;
  ros2_isobus::msg::VTSessionState state_;
  diagnostic_msgs::msg::DiagnosticArray diagnostics_;
  ros2_isobus::msg::VTPointingEvent pointing_;
  ros2_isobus::msg::VTNavigationEvent navigation_;
  ros2_isobus::msg::VTUpdateResult update_result_;
  ros2_isobus::msg::VTAuxStatus aux_status_;

  std::vector<std::function<void(const ros2_isobus::msg::VTStatus &)>> status_callbacks_;
  std::vector<std::function<void(const ros2_isobus::msg::VTStatus &)>> status_timeout_callbacks_;
  std::vector<std::function<void(const ros2_isobus::msg::VTSessionState &)>> state_callbacks_;
  std::vector<std::function<void(const diagnostic_msgs::msg::DiagnosticArray &)>> diagnostics_callbacks_;
  std::vector<std::function<void(const ros2_isobus::msg::VTPointingEvent &)>> pointing_callbacks_;
  std::vector<std::function<void(const ros2_isobus::msg::VTNavigationEvent &)>> navigation_callbacks_;
  std::vector<std::function<void(const ros2_isobus::msg::VTUpdateResult &)>> update_callbacks_;
  std::vector<std::function<void(const ros2_isobus::msg::VTAuxStatus &)>> aux_status_callbacks_;


  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> named_empty_set_pubs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr update_pub_;

  rclcpp::Subscription<ros2_isobus::msg::VTStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<ros2_isobus::msg::VTSessionState>::SharedPtr state_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<ros2_isobus::msg::VTPointingEvent>::SharedPtr pointing_sub_;
  rclcpp::Subscription<ros2_isobus::msg::VTNavigationEvent>::SharedPtr navigation_sub_;
  rclcpp::Subscription<ros2_isobus::msg::VTUpdateResult>::SharedPtr update_result_sub_;
  rclcpp::Subscription<ros2_isobus::msg::VTAuxStatus>::SharedPtr aux_status_sub_;
};

}  // namespace ros2_isobus
