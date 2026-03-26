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

#include "Diagnostics.hpp"

#include <algorithm>
#include <array>

namespace ros2_isobus
{
namespace
{
// ISO 11783-3:2018 network management / request / acknowledgement PGNs.
static constexpr std::uint32_t PGN_ACKM = 0xE800;
static constexpr std::uint32_t PGN_REQUEST = 0xEA00;
// ISO 11783-12:2019 Annex B diagnostic information message PGNs.
static constexpr std::uint32_t PGN_ECU_IDENT = 0xFDC5;
static constexpr std::uint32_t PGN_SW_IDENT = 0xFEDA;
static constexpr std::uint32_t PGN_DIAG_PROTOCOL = 0xFD32;
static constexpr std::uint32_t PGN_DM1 = 0xFECA;
static constexpr std::uint32_t PGN_DM2 = 0xFECB;
static constexpr std::uint32_t PGN_DM3 = 0xFECC;
static constexpr std::uint32_t PGN_ISOBUS_COMPLIANCE_CERT = 0xFD42;
static constexpr std::uint32_t PGN_CF_FUNCTIONALITIES = 0xFC8E;
static constexpr std::uint32_t PGN_PRODUCT_IDENT = 0xFEEB;

static constexpr std::uint8_t ACK_CONTROL_ACK = 0x00;
static constexpr std::uint8_t ACK_CONTROL_NACK = 0x01;
}  // namespace

void Diagnostics::setConfig(const Config & cfg)
{
  cfg_ = cfg;
  cfg_.source_address = clampU8(cfg_.source_address);
  cfg_.priority = clampU8(cfg_.priority);
  cfg_.dm1_byte1 = clampU8(cfg_.dm1_byte1);
  cfg_.dm1_byte2 = clampU8(cfg_.dm1_byte2);
  cfg_.dm2_byte1 = clampU8(cfg_.dm2_byte1);
  cfg_.dm2_byte2 = clampU8(cfg_.dm2_byte2);
  if (cfg_.dm1_period_ms < 10U) cfg_.dm1_period_ms = 10U;
  if (cfg_.compliance_cert_year < 0) cfg_.compliance_cert_year = 0;
  last_dm1_has_faults_ = !cfg_.active_dtcs.empty();
  last_dm1_snapshot_ = cfg_.active_dtcs;
}

void Diagnostics::setSourceAddress(std::uint8_t sa)
{
  cfg_.source_address = clampU8(sa);
}

bool Diagnostics::isValidSA(std::uint8_t sa)
{
  return sa <= 0xFDU;
}

std::uint8_t Diagnostics::clampU8(int v)
{
  if (v < 0) return 0U;
  if (v > 255) return 255U;
  return static_cast<std::uint8_t>(v);
}

bool Diagnostics::sameDtcs(const std::vector<Dtc> & a, const std::vector<Dtc> & b)
{
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].spn != b[i].spn || a[i].fmi != b[i].fmi || a[i].oc != b[i].oc || a[i].cm != b[i].cm) {
      return false;
    }
  }
  return true;
}

std::vector<std::uint8_t> Diagnostics::buildAsciiStarPayload(const std::vector<std::string> & fields)
{
  std::vector<std::uint8_t> payload;
  for (std::size_t i = 0; i < fields.size(); ++i) {
    payload.insert(payload.end(), fields[i].begin(), fields[i].end());
    if (i + 1U < fields.size()) payload.push_back(static_cast<std::uint8_t>('*'));
  }
  if (payload.size() < 8U) payload.resize(8U, 0xFF);
  return payload;
}

std::vector<std::uint8_t> Diagnostics::buildEcuIdentPayload() const
{
  // ISO 11783-12:2019, Annex B.1 ECU identification information (ASCII '*' delimiter).
  return buildAsciiStarPayload({
    cfg_.ecu_part_number,
    cfg_.ecu_serial_number,
    cfg_.ecu_location,
    cfg_.ecu_type,
    cfg_.ecu_manufacturer_name,
    cfg_.ecu_hardware_id});
}

std::vector<std::uint8_t> Diagnostics::buildSwIdentPayload() const
{
  // ISO 11783-12:2019, Annex B.2 software identification:
  // byte 1 = number of fields, byte 2..n = '*' separated software identification string.
  std::vector<std::uint8_t> payload;
  payload.push_back(clampU8(static_cast<int>(cfg_.software_ident_fields.size())));
  for (std::size_t i = 0; i < cfg_.software_ident_fields.size(); ++i) {
    payload.insert(payload.end(), cfg_.software_ident_fields[i].begin(), cfg_.software_ident_fields[i].end());
    if (i + 1U < cfg_.software_ident_fields.size()) payload.push_back(static_cast<std::uint8_t>('*'));
  }
  if (payload.size() < 8U) payload.resize(8U, 0xFF);
  return payload;
}

std::vector<std::uint8_t> Diagnostics::buildDiagProtocolPayload() const
{
  // ISO 11783-12:2019, Annex B.5 diagnostic protocol identification:
  // byte 1 identifies supported protocol, bytes 2..8 reserved (0xFF).
  return std::vector<std::uint8_t>{cfg_.diagnostic_protocol_id, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
}

std::vector<std::uint8_t> Diagnostics::buildIsobusCompliancePayload() const
{
  // ISO 11783-12:2019 Annex B.3 delegates PGN 0xFD42 payload format to ISO 11783-7 B.27.
  // Pack CCID fields from certificate-aligned parameters:
  //   - Year (A.29.1, 6-bit, 2000 offset)
  //   - Revision (A.29.2, 5-bit, split across byte1/byte2)
  //   - Lab type (A.29.3, 2-bit)
  //   - Lab ID (A.29.4, 11-bit)
  //   - Reference number (A.29.18, 16-bit little-endian)
  //   - Message revision bit (A.29.19)
  int year_off = cfg_.compliance_cert_year - 2000;
  if (year_off < 0) year_off = 0;
  if (year_off > 63) year_off = 63;
  const std::uint8_t year6 = static_cast<std::uint8_t>(year_off) & 0x3FU;

  const std::uint8_t rev5 = cfg_.compliance_cert_revision & 0x1FU;
  const std::uint8_t rev_b1_2 = rev5 & 0x03U;
  const std::uint8_t rev_b3 = static_cast<std::uint8_t>((rev5 >> 2U) & 0x01U);
  const std::uint8_t rev_b4_5 = static_cast<std::uint8_t>((rev5 >> 3U) & 0x03U);

  const std::uint8_t lab_type2 = cfg_.compliance_lab_type & 0x03U;
  const std::uint16_t lab_id11 = static_cast<std::uint16_t>(cfg_.compliance_test_lab_id & 0x07FFU);
  const std::uint8_t lab_id_lsb3 = static_cast<std::uint8_t>(lab_id11 & 0x0007U);
  const std::uint8_t lab_id_msb8 = static_cast<std::uint8_t>((lab_id11 >> 3U) & 0x00FFU);

  const std::uint8_t b1 = static_cast<std::uint8_t>((rev_b1_2 << 6U) | year6);
  const std::uint8_t b2 = static_cast<std::uint8_t>(
    (lab_id_lsb3 << 5U) | (rev_b4_5 << 3U) | (lab_type2 << 1U) | rev_b3);
  const std::uint8_t b3 = lab_id_msb8;
  const std::uint8_t b4 = cfg_.compliance_type_byte4;
  const std::uint8_t b5 = cfg_.compliance_type_byte5;
  const std::uint8_t b6 = static_cast<std::uint8_t>((cfg_.compliance_message_revision & 0x01U) << 7U);
  const std::uint16_t ref = cfg_.compliance_reference_number;

  return std::vector<std::uint8_t>{
    b1,
    b2,
    b3,
    b4,
    b5,
    b6,
    static_cast<std::uint8_t>(ref & 0x00FFU),
    static_cast<std::uint8_t>((ref >> 8U) & 0x00FFU)};
}

std::vector<std::uint8_t> Diagnostics::buildProductIdentPayload() const
{
  // ISO 11783-12:2019, Annex B.10 product identification (ASCII '*' delimiter).
  return buildAsciiStarPayload({cfg_.product_ident_code, cfg_.product_ident_brand, cfg_.product_ident_model});
}

std::vector<std::uint8_t> Diagnostics::buildFunctionalitiesPayload() const
{
  // ISO 11783-12:2019, Annex B.9 control function functionalities.
  std::vector<std::uint8_t> payload;
  payload.push_back(0xFF);
  payload.push_back(clampU8(static_cast<int>(cfg_.functionalities.size())));
  for (const auto & e : cfg_.functionalities) {
    payload.push_back(e.id);
    payload.push_back(e.generation);
    payload.push_back(clampU8(static_cast<int>(e.options.size())));
    payload.insert(payload.end(), e.options.begin(), e.options.end());
  }
  if (payload.size() < 8U) payload.resize(8U, 0xFF);
  return payload;
}

std::array<std::uint8_t, 4> Diagnostics::encodeDtc(const Dtc & d)
{
  return {
    static_cast<std::uint8_t>(d.spn & 0xFFU),
    static_cast<std::uint8_t>((d.spn >> 8U) & 0xFFU),
    static_cast<std::uint8_t>(((d.spn >> 16U) & 0x07U) << 5U | (d.fmi & 0x1FU)),
    static_cast<std::uint8_t>((d.cm ? 0x80U : 0x00U) | (d.oc & 0x7FU))
  };
}

std::vector<std::uint8_t> Diagnostics::buildDmPayload(
  const std::vector<Dtc> & dtcs, std::uint8_t b1, std::uint8_t b2) const
{
  // ISO 11783-12:2019, Annex B.6/B.7 DM1/DM2 payload:
  // bytes 1..2 lamp/status bytes, then repeating DTC tuples.
  std::vector<std::uint8_t> payload{b1, b2};
  if (dtcs.empty()) {
    payload.insert(payload.end(), {0x00, 0x00, 0x00, 0x00});
    payload.resize(8U, 0xFFU);
    return payload;
  }
  for (const auto & d : dtcs) {
    const auto bytes = encodeDtc(d);
    payload.insert(payload.end(), bytes.begin(), bytes.end());
  }
  if (payload.size() < 8U) payload.resize(8U, 0xFFU);
  return payload;
}

void Diagnostics::sendAck(std::uint8_t da, std::uint8_t control, std::uint32_t requested_pgn)
{
  // ISO 11783-3:2018 PGN 59392 Acknowledgement (ACK/NACK).
  msg::IsobusFrame fr;
  fr.priority = cfg_.priority;
  fr.page = false;
  fr.pgn = PGN_ACKM;
  fr.sa = cfg_.source_address;
  fr.pf = static_cast<std::uint8_t>((PGN_ACKM >> 8U) & 0xFFU);
  fr.ps = da;
  fr.data.fill(0xFF);
  fr.data[0] = control;
  fr.data[5] = static_cast<std::uint8_t>(requested_pgn & 0xFFU);
  fr.data[6] = static_cast<std::uint8_t>((requested_pgn >> 8U) & 0xFFU);
  fr.data[7] = static_cast<std::uint8_t>((requested_pgn >> 16U) & 0xFFU);
  sendFrame(fr);
}

void Diagnostics::sendPayload(std::uint32_t pgn, std::uint8_t da, const std::vector<std::uint8_t> & payload)
{
  // ISO 11783-3:2018 frame formatting:
  // PDU1 => PS=DA, PDU2 => PS=group extension (PGN LSB).
  const std::uint8_t pf = static_cast<std::uint8_t>((pgn >> 8U) & 0xFFU);
  const bool pdu1 = (pf < 240U);
  const std::uint8_t ps = pdu1 ? da : static_cast<std::uint8_t>(pgn & 0xFFU);
  if (payload.size() <= 8U) {
    msg::IsobusFrame fr;
    fr.priority = cfg_.priority;
    fr.page = false;
    fr.pgn = pgn;
    fr.sa = cfg_.source_address;
    fr.pf = pf;
    fr.ps = ps;
    fr.data.fill(0xFF);
    for (std::size_t i = 0; i < payload.size(); ++i) fr.data[i] = payload[i];
    sendFrame(fr);
    return;
  }
  msg::IsobusTpFrame tp;
  tp.priority = cfg_.priority;
  tp.page = false;
  tp.pgn = pgn;
  tp.sa = cfg_.source_address;
  tp.pf = pf;
  tp.ps = ps;
  tp.data = payload;
  sendTpFrame(tp);
}

void Diagnostics::run()
{
  // ISO 11783-12:2019 Annex B.6:
  // periodic DM1 transmission while active faults exist, and update on state/content change.
  if (!cfg_.periodic_dm1_enabled || !isValidSA(cfg_.source_address)) return;
  const bool has_faults = !cfg_.active_dtcs.empty();
  const bool dtc_changed = !sameDtcs(cfg_.active_dtcs, last_dm1_snapshot_);
  if (!has_faults && !last_dm1_has_faults_ && !dtc_changed) return;
  sendPayload(PGN_DM1, 0xFFU, buildDmPayload(cfg_.active_dtcs, cfg_.dm1_byte1, cfg_.dm1_byte2));
  last_dm1_snapshot_ = cfg_.active_dtcs;
  last_dm1_has_faults_ = has_faults;
}

void Diagnostics::handleMsg(const msg::IsobusFrame & fr)
{
  // ISO 11783-12:2019 clause 6 + Annex B:
  // reply to PGN 59904 requests for supported diagnostics messages.
  if (!cfg_.respond_to_requests || !isValidSA(cfg_.source_address)) return;
  const std::uint8_t pf = fr.pf ? fr.pf : static_cast<std::uint8_t>((fr.pgn >> 8U) & 0xFFU);
  const std::uint32_t pgn_from_pf = static_cast<std::uint32_t>(pf) << 8U;
  if (fr.pgn != PGN_REQUEST && pgn_from_pf != PGN_REQUEST) return;
  if (fr.sa == cfg_.source_address) return;
  const bool global = (fr.ps == 0xFFU);
  const bool specific = (fr.ps == cfg_.source_address);
  if (!global && !specific) return;

  const std::uint32_t req_pgn =
    static_cast<std::uint32_t>(fr.data[0]) |
    (static_cast<std::uint32_t>(fr.data[1]) << 8U) |
    (static_cast<std::uint32_t>(fr.data[2]) << 16U);

  switch (req_pgn) {
    case PGN_ECU_IDENT:
      sendPayload(PGN_ECU_IDENT, fr.sa, buildEcuIdentPayload());
      return;
    case PGN_SW_IDENT:
      sendPayload(PGN_SW_IDENT, fr.sa, buildSwIdentPayload());
      return;
    case PGN_DIAG_PROTOCOL:
      sendPayload(PGN_DIAG_PROTOCOL, fr.sa, buildDiagProtocolPayload());
      return;
    case PGN_DM1:
      sendPayload(PGN_DM1, fr.sa, buildDmPayload(cfg_.active_dtcs, cfg_.dm1_byte1, cfg_.dm1_byte2));
      return;
    case PGN_DM2:
      sendPayload(PGN_DM2, fr.sa, buildDmPayload(cfg_.previously_active_dtcs, cfg_.dm2_byte1, cfg_.dm2_byte2));
      return;
    case PGN_DM3:
      if (cfg_.dm3_clear_dm2) {
        cfg_.previously_active_dtcs.clear();
        if (specific) sendAck(fr.sa, ACK_CONTROL_ACK, PGN_DM3);
      } else if (specific) {
        sendAck(fr.sa, ACK_CONTROL_NACK, PGN_DM3);
      }
      return;
    case PGN_ISOBUS_COMPLIANCE_CERT:
      sendPayload(PGN_ISOBUS_COMPLIANCE_CERT, fr.sa, buildIsobusCompliancePayload());
      return;
    case PGN_CF_FUNCTIONALITIES:
      sendPayload(PGN_CF_FUNCTIONALITIES, fr.sa, buildFunctionalitiesPayload());
      return;
    case PGN_PRODUCT_IDENT:
      sendPayload(PGN_PRODUCT_IDENT, fr.sa, buildProductIdentPayload());
      return;
    default:
      break;
  }

  if (specific && cfg_.nack_unsupported_request) {
    sendAck(fr.sa, ACK_CONTROL_NACK, req_pgn);
  }
}

}  // namespace ros2_isobus
