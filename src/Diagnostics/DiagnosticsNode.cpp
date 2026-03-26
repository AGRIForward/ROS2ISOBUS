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

#include "DiagnosticsNode.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <sstream>

#include "ros2_isobus/topics.hpp"

namespace ros2_isobus
{

std::uint8_t DiagnosticsROS2::clamp_u8(int v)
{
  if (v < 0) return 0U;
  if (v > 255) return 255U;
  return static_cast<std::uint8_t>(v);
}

static std::uint16_t clamp_u16(int v)
{
  if (v < 0) return 0U;
  if (v > 65535) return 65535U;
  return static_cast<std::uint16_t>(v);
}

std::vector<std::uint8_t> DiagnosticsROS2::parse_byte_list(const std::string & s)
{
  std::vector<std::uint8_t> out;
  std::string norm;
  norm.reserve(s.size());
  for (char c : s) norm.push_back((c == ';' || c == '|') ? ',' : c);
  std::stringstream ss(norm);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(
      std::remove_if(item.begin(), item.end(), [](unsigned char ch) { return std::isspace(ch) != 0; }),
      item.end());
    if (item.empty()) continue;
    try {
      out.push_back(clamp_u8(std::stoi(item, nullptr, 0)));
    } catch (...) {
    }
  }
  return out;
}

DiagnosticsROS2::DiagnosticsROS2(const rclcpp::NodeOptions & options)
: rclcpp::Node("diagnostics_node", options)
{
  // ROS2 wrapper for ISO 11783-12 diagnostics core.
  // Parameter parsing and transport bindings are handled here,
  // while Diagnostics base class contains the protocol behavior.
  load_config();

  tx_pub_ = create_publisher<msg::IsobusFrame>(kBusTxTopic, rclcpp::QoS(100));
  tx_tp_pub_ = create_publisher<msg::IsobusTpFrame>(kBusTxTpTopic, rclcpp::QoS(10));
  rx_sub_ = create_subscription<msg::IsobusFrame>(
    kBusRxTopic, rclcpp::QoS(100),
    [this](const msg::IsobusFrame::SharedPtr fr) {
      if (fr) handleMsg(*fr);
    });

  if (use_address_manager_sa_) {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    addr_sub_ = create_subscription<msg::IsobusAddressStatus>(
      kAddressManagerStatus, qos,
      [this](const msg::IsobusAddressStatus::SharedPtr msg) {
        if (!msg || msg->sa > 0xFDU) return;
        setSourceAddress(msg->sa);
      });
  }

  timer_ = create_wall_timer(
    std::chrono::milliseconds(dm1PeriodMs()),
    [this]() { run(); });

  printInfo("Diagnostics node ready (ISO 11783-12 Annex B minimum set)");
}

void DiagnosticsROS2::load_config()
{
  // Read all diagnostics parameters from ROS2 and map them to
  // Diagnostics::Config used by ISO 11783-12 message generation.
  Diagnostics::Config cfg;
  cfg.source_address = clamp_u8(declare_parameter<int>("source_address", 0xFE));
  cfg.priority = clamp_u8(declare_parameter<int>("priority", 6));
  cfg.respond_to_requests = declare_parameter<bool>("respond_to_requests", true);
  cfg.nack_unsupported_request = declare_parameter<bool>("nack_unsupported_request", true);
  cfg.periodic_dm1_enabled = declare_parameter<bool>("periodic_dm1_enabled", true);
  cfg.dm3_clear_dm2 = declare_parameter<bool>("dm3_clear_dm2", true);
  cfg.dm1_period_ms = static_cast<std::uint32_t>(std::max<int>(10, declare_parameter<int>("dm1_period_ms", 1000)));
  cfg.dm1_byte1 = clamp_u8(declare_parameter<int>("dm1_byte1", 0xFF));
  cfg.dm1_byte2 = clamp_u8(declare_parameter<int>("dm1_byte2", 0xFF));
  cfg.dm2_byte1 = clamp_u8(declare_parameter<int>("dm2_byte1", 0xFF));
  cfg.dm2_byte2 = clamp_u8(declare_parameter<int>("dm2_byte2", 0xFF));

  cfg.ecu_part_number = declare_parameter<std::string>("ecu_part_number", "ECU-PN-UNSET");
  cfg.ecu_serial_number = declare_parameter<std::string>("ecu_serial_number", "ECU-SN-UNSET");
  cfg.ecu_location = declare_parameter<std::string>("ecu_location", "UNSET");
  cfg.ecu_type = declare_parameter<std::string>("ecu_type", "UNSET");
  cfg.ecu_manufacturer_name = declare_parameter<std::string>("ecu_manufacturer_name", "UNSET");
  cfg.ecu_hardware_id = declare_parameter<std::string>("ecu_hardware_id", "UNSET");
  cfg.software_ident_fields = declare_parameter<std::vector<std::string>>(
    "software_ident_fields", std::vector<std::string>{"ROS2ISOBUS#Diagnostics#1.0"});
  cfg.diagnostic_protocol_id = clamp_u8(declare_parameter<int>("diagnostic_protocol_id", 0x00));

  cfg.compliance_test_lab_id = clamp_u16(declare_parameter<int>("compliance_test_lab_id", 0x18));
  cfg.compliance_cert_year = std::max<int>(0, declare_parameter<int>("compliance_cert_year", 2026));
  cfg.compliance_cert_revision = clamp_u8(declare_parameter<int>("compliance_cert_revision", 1));
  cfg.compliance_lab_type = clamp_u8(declare_parameter<int>("compliance_lab_type", 1));
  cfg.compliance_reference_number = clamp_u16(declare_parameter<int>("compliance_reference_number", 0));
  cfg.compliance_message_revision = clamp_u8(declare_parameter<int>("compliance_message_revision", 1));
  cfg.compliance_type_byte4 = clamp_u8(declare_parameter<int>("compliance_type_byte4", 0));
  cfg.compliance_type_byte5 = clamp_u8(declare_parameter<int>("compliance_type_byte5", 0));

  cfg.product_ident_code = declare_parameter<std::string>("product_ident_code", "UNSET-CODE");
  cfg.product_ident_brand = declare_parameter<std::string>("product_ident_brand", "UNSET-BRAND");
  cfg.product_ident_model = declare_parameter<std::string>("product_ident_model", "UNSET-MODEL");

  const auto ids = declare_parameter<std::vector<std::int64_t>>("cf_functionality_ids", std::vector<std::int64_t>{});
  const auto gens = declare_parameter<std::vector<std::int64_t>>(
    "cf_functionality_generations", std::vector<std::int64_t>{});
  const auto options =
    declare_parameter<std::vector<std::string>>("cf_functionality_options", std::vector<std::string>{});
  const std::size_t fn_count = std::min(ids.size(), gens.size());
  cfg.functionalities.reserve(fn_count);
  for (std::size_t i = 0; i < fn_count; ++i) {
    Diagnostics::FunctionalityEntry e;
    e.id = clamp_u8(static_cast<int>(ids[i]));
    e.generation = clamp_u8(static_cast<int>(gens[i]));
    if (i < options.size()) e.options = parse_byte_list(options[i]);
    cfg.functionalities.push_back(e);
  }

  auto parse_dtcs = [this](const std::string & prefix, std::vector<Diagnostics::Dtc> & out) {
    const auto spn = declare_parameter<std::vector<std::int64_t>>(prefix + "_spn", std::vector<std::int64_t>{});
    const auto fmi = declare_parameter<std::vector<std::int64_t>>(prefix + "_fmi", std::vector<std::int64_t>{});
    const auto oc = declare_parameter<std::vector<std::int64_t>>(prefix + "_oc", std::vector<std::int64_t>{});
    const auto cm = declare_parameter<std::vector<std::int64_t>>(prefix + "_cm", std::vector<std::int64_t>{});
    const std::size_t n = std::min({spn.size(), fmi.size(), oc.size(), cm.size()});
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      Diagnostics::Dtc d;
      d.spn = static_cast<std::uint32_t>(std::max<std::int64_t>(0, std::min<std::int64_t>(spn[i], 0x7FFFF)));
      d.fmi = static_cast<std::uint8_t>(std::max<std::int64_t>(0, std::min<std::int64_t>(fmi[i], 31)));
      d.oc = static_cast<std::uint8_t>(std::max<std::int64_t>(0, std::min<std::int64_t>(oc[i], 127)));
      d.cm = static_cast<std::uint8_t>(cm[i] ? 1U : 0U);
      out.push_back(d);
    }
  };
  parse_dtcs("active_dtc", cfg.active_dtcs);
  parse_dtcs("previously_active_dtc", cfg.previously_active_dtcs);

  use_address_manager_sa_ = declare_parameter<bool>("use_address_manager_sa", true);
  setConfig(cfg);
}

void DiagnosticsROS2::sendFrame(const msg::IsobusFrame & frame)
{
  if (tx_pub_) tx_pub_->publish(frame);
}

void DiagnosticsROS2::sendTpFrame(const msg::IsobusTpFrame & frame)
{
  if (tx_tp_pub_) tx_tp_pub_->publish(frame);
}

void DiagnosticsROS2::printInfo(const std::string & msg)
{
  RCLCPP_INFO(get_logger(), "%s", msg.c_str());
}

void DiagnosticsROS2::printWarn(const std::string & msg)
{
  RCLCPP_WARN(get_logger(), "%s", msg.c_str());
}

}  // namespace ros2_isobus

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_isobus::DiagnosticsROS2>());
  rclcpp::shutdown();
  return 0;
}
