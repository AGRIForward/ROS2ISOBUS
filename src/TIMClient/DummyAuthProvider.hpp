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
#include <vector>

#include "TIMClient.hpp"

namespace ros2_isobus
{

// Demo-grade AEF040-like authentication provider without real cryptography/certificate validation.
class DummyAuthProvider final : public TimClient::IAuthProvider
{
public:
  DummyAuthProvider(
    TimClient & owner, std::uint32_t period_ms, std::uint8_t implemented_version, std::uint8_t minimum_version);

  void reset(std::uint8_t client_sa, std::uint8_t server_sa) override;
  void on_frame(const ros2_isobus::msg::IsobusFrame & fr) override;
  void on_tp_payload(std::uint32_t pgn, const std::vector<std::uint8_t> & payload) override;
  void process(std::uint32_t now_ms) override;
  bool is_authenticated() const override;
  bool is_lead_server() const override;
  bool is_failed() const override;

private:
  enum class Step : std::uint8_t
  {
    WaitRandomTrigger = 0,
    WaitRandomTp,
    WaitCertTrigger,
    WaitCertTp,
    WaitFinalizeTrigger,
    WaitFinalizeTp
  };

  ros2_isobus::msg::IsobusFrame make_auth_frame(std::uint8_t msg_code) const;
  ros2_isobus::msg::IsobusTpFrame make_auth_tp(std::uint8_t msg_code, std::size_t length, std::uint8_t round) const;
  void send_auth_status(std::uint32_t now_ms, bool restart_bit);
  void send_round_requests(std::uint32_t now_ms);
  void send_compat_response(std::uint8_t msg_code, std::uint8_t round);

  std::uint32_t period_ms_{1000};
  std::uint8_t implemented_version_{3};
  std::uint8_t minimum_version_{2};
  std::uint8_t auth_connection_version_{2};
  std::uint8_t client_sa_{0xFE};
  std::uint8_t server_sa_{0xFE};

  std::uint8_t cert_round_{1};
  std::uint8_t retries_{0};
  Step step_{Step::WaitRandomTrigger};
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
};

}  // namespace ros2_isobus
