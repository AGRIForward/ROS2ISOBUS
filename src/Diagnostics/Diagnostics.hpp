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

#include <cstdint>
#include <string>
#include <vector>

#include "ros2_isobus/msg/isobus_frame.hpp"
#include "ros2_isobus/msg/isobus_tp_frame.hpp"

namespace ros2_isobus
{

/*
 *
 * Diagnostics implements ISO 11783-12 (2019) Annex B diagnostic flow:
 *  - Responds to PGN 59904 requests for Annex B diagnostic information messages
 *  - Publishes DM1 according to configured timing/state-change behavior
 *  - Encodes DM/identification/functionality payloads from configured diagnostics data
 *
 */
class Diagnostics
{
public:
  // Diagnostic trouble code structure used in DM1/DM2 payloads
  // (ISO 11783-12:2019, Annex B.6 / B.7; SAE J1939-73 compatible encoding).
  struct Dtc
  {
    std::uint32_t spn{0};
    std::uint8_t fmi{0};
    std::uint8_t oc{0};
    std::uint8_t cm{0};
  };

  struct FunctionalityEntry
  {
    std::uint8_t id{0};
    std::uint8_t generation{0};
    std::vector<std::uint8_t> options;
  };

  struct Config
  {
    std::uint8_t source_address{0xFE};
    std::uint8_t priority{6};
    bool respond_to_requests{true};
    bool nack_unsupported_request{true};
    bool periodic_dm1_enabled{true};
    bool dm3_clear_dm2{true};
    std::uint32_t dm1_period_ms{1000};
    std::uint8_t dm1_byte1{0xFF};
    std::uint8_t dm1_byte2{0xFF};
    std::uint8_t dm2_byte1{0xFF};
    std::uint8_t dm2_byte2{0xFF};

    std::string ecu_part_number{"ECU-PN-UNSET"};
    std::string ecu_serial_number{"ECU-SN-UNSET"};
    std::string ecu_location{"UNSET"};
    std::string ecu_type{"UNSET"};
    std::string ecu_manufacturer_name{"UNSET"};
    std::string ecu_hardware_id{"UNSET"};
    std::vector<std::string> software_ident_fields{"ROS2ISOBUS#Diagnostics#1.0"};
    std::uint8_t diagnostic_protocol_id{0};

    // ISO 11783-7 A.29.4: 11-bit certification laboratory ID.
    std::uint16_t compliance_test_lab_id{0x18};
    int compliance_cert_year{2026};
    // ISO 11783-7 A.29.2: 5-bit protocol revision.
    std::uint8_t compliance_cert_revision{1};
    // ISO 11783-7 A.29.3: 2-bit laboratory type (01 = AEF certified laboratory).
    std::uint8_t compliance_lab_type{1};
    // ISO 11783-7 A.29.18: 16-bit certification reference number.
    std::uint16_t compliance_reference_number{0};
    // ISO 11783-7 A.29.19: message revision bit (0=2nd ed format, 1=3rd+).
    std::uint8_t compliance_message_revision{1};
    // Certification type bitfields (bytes 4..5 in B.27).
    std::uint8_t compliance_type_byte4{0};
    std::uint8_t compliance_type_byte5{0};

    std::string product_ident_code{"UNSET-CODE"};
    std::string product_ident_brand{"UNSET-BRAND"};
    std::string product_ident_model{"UNSET-MODEL"};
    std::vector<FunctionalityEntry> functionalities;
    std::vector<Dtc> active_dtcs;
    std::vector<Dtc> previously_active_dtcs;
  };

  Diagnostics() = default;
  virtual ~Diagnostics() = default;

  // Load diagnostics behavior and content configuration
  // (ISO 11783-12:2019, clause 6 + Annex B message set).
  void setConfig(const Config & cfg);
  void setSourceAddress(std::uint8_t sa);
  std::uint32_t dm1PeriodMs() const { return cfg_.dm1_period_ms; }

  void run(); // Periodic DM1 processing (ISO 11783-12:2019, Annex B.6 transmission rules).
  void handleMsg(const msg::IsobusFrame & fr); // Handle PGN 59904 requests for Annex B messages.

protected:
  virtual void sendFrame(const msg::IsobusFrame & frame) = 0;
  virtual void sendTpFrame(const msg::IsobusTpFrame & frame) = 0;
  virtual void printInfo(const std::string & msg) = 0;
  virtual void printWarn(const std::string & msg) = 0;

private:
  static bool isValidSA(std::uint8_t sa);
  static std::uint8_t clampU8(int v);
  static bool sameDtcs(const std::vector<Dtc> & a, const std::vector<Dtc> & b);
  static std::vector<std::uint8_t> buildAsciiStarPayload(const std::vector<std::string> & fields);
  static std::array<std::uint8_t, 4> encodeDtc(const Dtc & d);

  std::vector<std::uint8_t> buildEcuIdentPayload() const;
  std::vector<std::uint8_t> buildSwIdentPayload() const;
  std::vector<std::uint8_t> buildDiagProtocolPayload() const;
  std::vector<std::uint8_t> buildIsobusCompliancePayload() const;
  std::vector<std::uint8_t> buildProductIdentPayload() const;
  std::vector<std::uint8_t> buildFunctionalitiesPayload() const;
  std::vector<std::uint8_t> buildDmPayload(const std::vector<Dtc> & dtcs, std::uint8_t b1, std::uint8_t b2) const;
  void sendAck(std::uint8_t da, std::uint8_t control, std::uint32_t requested_pgn);
  void sendPayload(std::uint32_t pgn, std::uint8_t da, const std::vector<std::uint8_t> & payload);

  Config cfg_{};
  std::vector<Dtc> last_dm1_snapshot_;
  bool last_dm1_has_faults_{false};
};

}  // namespace ros2_isobus
