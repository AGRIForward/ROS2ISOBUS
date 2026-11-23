#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "Diagnostics.hpp"

namespace
{
constexpr std::uint32_t kRequestPgn = 0xEA00;

class CapturingDiagnostics : public ros2_isobus::Diagnostics
{
public:
  std::vector<ros2_isobus::msg::IsobusFrame> frames;
  std::vector<ros2_isobus::msg::IsobusTpFrame> tp_frames;

protected:
  void sendFrame(const ros2_isobus::msg::IsobusFrame & frame) override { frames.push_back(frame); }
  void sendTpFrame(const ros2_isobus::msg::IsobusTpFrame & frame) override { tp_frames.push_back(frame); }
  void printInfo(const std::string &) override {}
  void printWarn(const std::string &) override {}
};

ros2_isobus::msg::IsobusFrame request(std::uint32_t requested_pgn)
{
  ros2_isobus::msg::IsobusFrame frame;
  frame.pgn = kRequestPgn;
  frame.pf = 0xEA;
  frame.ps = 0x1C;
  frame.sa = 0x80;
  frame.data.fill(0xFF);
  frame.data[0] = static_cast<std::uint8_t>(requested_pgn & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>((requested_pgn >> 8U) & 0xFFU);
  frame.data[2] = static_cast<std::uint8_t>((requested_pgn >> 16U) & 0xFFU);
  return frame;
}

TEST(DiagnosticsIso11783_12, ProductIdentificationUsesFc8dAndTrailingDelimiter)
{
  CapturingDiagnostics diagnostics;
  ros2_isobus::Diagnostics::Config config;
  config.source_address = 0x1C;
  config.product_ident_code = "CODE";
  config.product_ident_brand = "BRAND";
  config.product_ident_model = "MODEL";
  diagnostics.setConfig(config);

  diagnostics.handleMsg(request(0xFC8D));

  ASSERT_EQ(diagnostics.tp_frames.size(), 1U);
  EXPECT_EQ(diagnostics.tp_frames[0].pgn, 0xFC8DU);
  const std::string payload(
    diagnostics.tp_frames[0].data.begin(), diagnostics.tp_frames[0].data.end());
  EXPECT_EQ(payload, "CODE*BRAND*MODEL*");
}

TEST(DiagnosticsIso11783_12, IdentificationPayloadsEndInDelimiter)
{
  CapturingDiagnostics diagnostics;
  ros2_isobus::Diagnostics::Config config;
  config.source_address = 0x1C;
  config.ecu_part_number = "P";
  config.ecu_serial_number = "S";
  config.ecu_location = "L";
  config.ecu_type = "T";
  config.ecu_manufacturer_name = "M";
  config.ecu_hardware_id = "H";
  config.software_ident_fields = {"SW1", "SW2"};
  diagnostics.setConfig(config);

  diagnostics.handleMsg(request(0xFDC5));
  diagnostics.handleMsg(request(0xFEDA));

  ASSERT_EQ(diagnostics.tp_frames.size(), 2U);
  EXPECT_EQ(diagnostics.tp_frames[0].data.back(), static_cast<std::uint8_t>('*'));
  EXPECT_EQ(diagnostics.tp_frames[1].data[0], 2U);
  EXPECT_EQ(diagnostics.tp_frames[1].data.back(), static_cast<std::uint8_t>('*'));
}

TEST(DiagnosticsIso11783_12, FunctionalityTrailingZeroOptionsAreOmitted)
{
  CapturingDiagnostics diagnostics;
  ros2_isobus::Diagnostics::Config config;
  config.source_address = 0x1C;
  config.functionalities = {
    {0x00, 0x01, {}},
    {0x10, 0x02, {0xFF, 0x00, 0x01, 0x00, 0x00}}};
  diagnostics.setConfig(config);

  diagnostics.handleMsg(request(0xFC8E));

  ASSERT_EQ(diagnostics.tp_frames.size(), 1U);
  const std::vector<std::uint8_t> expected{
    0xFF, 0x02, 0x00, 0x01, 0x00, 0x10, 0x02, 0x03, 0xFF, 0x00, 0x01};
  EXPECT_EQ(diagnostics.tp_frames[0].data, expected);
}

TEST(DiagnosticsIso11783_12, DtcConversionMethodIsAlwaysZero)
{
  CapturingDiagnostics diagnostics;
  ros2_isobus::Diagnostics::Config config;
  config.source_address = 0x1C;
  config.active_dtcs = {{0x12345, 7, 3, 1}};
  diagnostics.setConfig(config);

  diagnostics.handleMsg(request(0xFECA));

  ASSERT_EQ(diagnostics.frames.size(), 1U);
  EXPECT_EQ(diagnostics.frames[0].data[5] & 0x80U, 0U);
  EXPECT_EQ(diagnostics.frames[0].data[5] & 0x7FU, 3U);
}

TEST(DiagnosticsIso11783_12, CertificationInformationIsDisabledByDefault)
{
  CapturingDiagnostics diagnostics;
  ros2_isobus::Diagnostics::Config config;
  config.source_address = 0x1C;
  diagnostics.setConfig(config);

  diagnostics.handleMsg(request(0xFD42));

  ASSERT_EQ(diagnostics.frames.size(), 1U);
  EXPECT_EQ(diagnostics.frames[0].pgn, 0xE800U);
  EXPECT_EQ(diagnostics.frames[0].data[0], 0x01U);
  EXPECT_TRUE(diagnostics.tp_frames.empty());
}

TEST(DiagnosticsIso11783_12, IncompleteEnabledCertificationIsRejected)
{
  CapturingDiagnostics diagnostics;
  ros2_isobus::Diagnostics::Config config;
  config.compliance_certification_enabled = true;

  EXPECT_THROW(diagnostics.setConfig(config), std::invalid_argument);
}

TEST(DiagnosticsIso11783_12, ValidEnabledCertificationIsEncoded)
{
  CapturingDiagnostics diagnostics;
  ros2_isobus::Diagnostics::Config config;
  config.source_address = 0x1C;
  config.compliance_certification_enabled = true;
  config.compliance_cert_year = 2026;
  config.compliance_cert_revision = 1;
  config.compliance_lab_type = 1;
  config.compliance_test_lab_id = 507;
  config.compliance_reference_number = 2;
  config.compliance_message_revision = 1;
  config.compliance_type_byte4 = 0;
  config.compliance_type_byte5 = 0;
  diagnostics.setConfig(config);

  diagnostics.handleMsg(request(0xFD42));

  ASSERT_EQ(diagnostics.frames.size(), 1U);
  EXPECT_EQ(diagnostics.frames[0].pgn, 0xFD42U);
  const std::array<std::uint8_t, 8> expected{0x5A, 0x62, 0x3F, 0x00, 0x00, 0x80, 0x02, 0x00};
  EXPECT_EQ(diagnostics.frames[0].data, expected);
}
}  // namespace
