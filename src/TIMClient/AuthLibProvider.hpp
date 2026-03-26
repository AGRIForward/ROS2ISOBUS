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
#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "authlib.h"
#include "TIMClient.hpp"

namespace ros2_isobus
{

/*
 *
 * AuthLibProvider implements TIM authentication flow using AuthLib (AEF 040 RIG 1):
 *  - Handles authentication short-frame and TP message exchange via IAuthProvider hooks
 *  - Encapsulates AuthLib usage for certificate-chain and challenge/secret operations
 *
 */
class AuthLibProvider final : public TimClient::IAuthProvider
{
public:
  AuthLibProvider(
    TimClient & owner,
    std::uint32_t period_ms,
    std::uint8_t implemented_version,
    std::uint8_t minimum_version,
    bool strict_mode,
    std::uint16_t client_cert_payload_max_len,
    std::uint16_t max_slice_iterations,
    const std::string & root_cert_path,
    const std::array<std::string, 4> & client_cert_paths,
    const std::string & client_private_key_hex,
    const std::string & server_public_key_hex);
  ~AuthLibProvider() override;

  // Reset per-session protocol state and cryptographic context.
  void reset(std::uint8_t client_sa, std::uint8_t server_sa) override;
  // Handle authentication single-frame traffic.
  void on_frame(const ros2_isobus::msg::IsobusFrame & fr) override;
  // Handle authentication TP payload traffic.
  void on_tp_payload(std::uint32_t pgn, const std::vector<std::uint8_t> & payload) override;
  // Periodic authentication state progression according to AEF 040 trigger/payload sequencing.
  void process(std::uint32_t now_ms) override;
  bool is_authenticated() const override;
  bool is_lead_server() const override;
  bool is_failed() const override;

private:
  bool init_authlib();
  void deinit_authlib();
  bool load_certificates();
  bool verify_server_certificate_chain();
  bool derive_common_secret();
  bool compute_client_signed_challenge();
  bool verify_server_signed_challenge();

  enum class Step : std::uint8_t
  {
    // AEF 040 message-pair sequence:
    // random trigger/payload -> certificate rounds trigger/payload -> finalize trigger/payload.
    WaitRandomTrigger = 0,
    WaitRandomTp,
    WaitCertTrigger,
    WaitCertTp,
    WaitFinalizeTrigger,
    WaitFinalizeTp
  };
  enum class CertExchangeState : std::uint8_t
  {
    NotRequested = 0,
    Requested,
    ClientSent,
    ServerReceived
  };

  ros2_isobus::msg::IsobusFrame make_auth_frame(std::uint8_t msg_code) const;
  ros2_isobus::msg::IsobusFrame make_client_version_response_frame() const;
  ros2_isobus::msg::IsobusFrame make_random_trigger_response_frame() const;
  ros2_isobus::msg::IsobusFrame make_cert_trigger_response_frame(std::uint8_t round) const;
  ros2_isobus::msg::IsobusFrame make_finalize_trigger_response_frame(std::uint8_t round) const;
  ros2_isobus::msg::IsobusTpFrame make_client_random_response_tp() const;
  ros2_isobus::msg::IsobusTpFrame make_client_certificate_response_tp(std::uint8_t round) const;
  ros2_isobus::msg::IsobusTpFrame make_client_finalize_response_tp(std::uint8_t round) const;
  void send_auth_status(std::uint32_t now_ms, bool restart_bit);
  void send_round_requests(std::uint32_t now_ms);
  bool protocol_violation(const std::string & msg);

  std::uint32_t period_ms_{1000};
  std::uint8_t implemented_version_{3};
  std::uint8_t minimum_version_{2};
  std::uint8_t auth_connection_version_{2};
  std::uint8_t client_sa_{0xFE};
  std::uint8_t server_sa_{0xFE};

  std::uint8_t retries_{0};
  Step step_{Step::WaitRandomTrigger};
  bool strict_mode_{true};
  std::uint16_t client_cert_payload_max_len_{0U};
  std::uint16_t max_slice_iterations_{64U};
  bool restart_sent_{false};
  bool authenticated_{false};
  bool failed_{false};
  bool server_status_seen_{false};
  bool got_server_tp_04_{false};
  bool got_server_tp_02_{false};
  bool got_server_tp_06_{false};
  bool server_finalize_payload_seen_{false};
  bool client_finalize_payload_sent_{false};
  bool pending_server_rnd_request_{false};
  bool pending_server_cert_request_{false};
  bool pending_server_finalize_request_{false};
  std::uint8_t requested_cert_round_{1};
  std::uint8_t requested_finalize_round_{1};
  std::uint8_t expected_server_cert_round_{0};
  std::uint8_t expected_server_finalize_round_{0};
  std::uint8_t auth_type_field_{0x0F};
  bool finalize_request_sent_{false};
  bool certs_valid_{false};
  bool challenge_signed_local_{false};
  bool challenge_signed_server_{false};
  bool server_authenticated_seen_{false};
  bool client_authenticated_status_sent_{false};
  bool pending_f8_request_{false};

  std::uint32_t last_status_tx_ms_{0};
  std::uint32_t last_round_tx_ms_{0};
  std::uint32_t step_deadline_ms_{0};

  std::string root_cert_path_;
  std::array<std::string, 4> client_cert_paths_{};
  std::vector<std::uint8_t> root_cert_;
  std::array<std::vector<std::uint8_t>, 4> client_certs_;
  std::array<std::vector<std::uint8_t>, 4> server_certs_;
  std::array<bool, 4> server_cert_received_{{false, false, false, false}};
  std::array<CertExchangeState, 4> cert_exchange_state_{{
      CertExchangeState::NotRequested,
      CertExchangeState::NotRequested,
      CertExchangeState::NotRequested,
      CertExchangeState::NotRequested}};
  CryptoLib_Crt_t parsed_root_cert_{};
  bool parsed_root_cert_valid_{false};
  std::array<CryptoLib_Crt_t, 4> parsed_server_certs_{};
  std::array<bool, 4> parsed_server_cert_valid_{{false, false, false, false}};
  std::string client_private_key_hex_;
  std::string server_public_key_hex_;
  std::array<std::uint8_t, 32> client_random_challenge_{};
  std::array<std::uint8_t, 32> server_random_challenge_{};
  std::array<std::uint8_t, 16> client_signed_challenge_{};
  std::array<std::uint8_t, 16> server_signed_challenge_{};
  bool common_secret_ready_{false};
  bool client_random_valid_{false};
  bool server_random_valid_{false};
  bool server_signed_valid_{false};
  std::array<std::uint8_t, 32> common_secret_{};
  bool authlib_initialized_{false};
};

}  // namespace ros2_isobus
