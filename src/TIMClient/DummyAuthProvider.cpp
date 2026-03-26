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

#include "DummyAuthProvider.hpp"

namespace ros2_isobus
{

namespace
{
static constexpr std::uint8_t kMsgAuthServerCert = 0x02;
static constexpr std::uint8_t kMsgAuthClientCert = 0x03;
static constexpr std::uint8_t kMsgAuthServerRnd = 0x04;
static constexpr std::uint8_t kMsgAuthClientRnd = 0x05;
static constexpr std::uint8_t kMsgAuthServerFinalize = 0x06;
static constexpr std::uint8_t kMsgAuthClientFinalize = 0x07;
static constexpr std::uint8_t kMsgAuthClientVersion = 0xF7;
static constexpr std::uint8_t kMsgAuthServerVersion = 0xF8;
static constexpr std::uint32_t kStepTimeoutMs = 3000;
static constexpr std::uint8_t kMaxRetries = 3;
static constexpr std::uint8_t kCertRoundsRequired = 4;
}  // namespace

DummyAuthProvider::DummyAuthProvider(
  TimClient & owner,
  std::uint32_t period_ms,
  std::uint8_t implemented_version,
  std::uint8_t minimum_version)
: IAuthProvider(owner),
  period_ms_(period_ms),
  implemented_version_(implemented_version),
  minimum_version_(minimum_version)
{
  auth_connection_version_ = minimum_version_;
}

void DummyAuthProvider::reset(std::uint8_t client_sa, std::uint8_t server_sa)
{
  client_sa_ = client_sa;
  server_sa_ = server_sa;
  cert_round_ = 1;
  retries_ = 0;
  step_ = Step::WaitRandomTrigger;
  restart_sent_ = false;
  authenticated_ = false;
  failed_ = false;
  server_status_seen_ = false;
  got_server_tp_04_ = false;
  got_server_tp_02_ = false;
  got_server_tp_06_ = false;
  server_finalize_payload_seen_ = false;
  client_finalize_payload_sent_ = false;
  pending_server_rnd_request_ = false;
  pending_server_cert_request_ = false;
  pending_server_finalize_request_ = false;
  finalize_request_sent_ = false;
  certs_valid_ = false;
  challenge_signed_local_ = false;
  challenge_signed_server_ = false;
  server_authenticated_seen_ = false;
  client_authenticated_status_sent_ = false;
  pending_f8_request_ = false;
  last_status_tx_ms_ = 0;
  last_round_tx_ms_ = 0;
  step_deadline_ms_ = 0;
  auth_connection_version_ = minimum_version_;
  logInfo("TIM auth: session reset");
}

ros2_isobus::msg::IsobusFrame DummyAuthProvider::make_auth_frame(std::uint8_t msg_code) const
{
  ros2_isobus::msg::IsobusFrame fr;
  fr.pgn = PGN_AUTH_C2S;
  fr.page = false;
  fr.priority = 6;
  fr.sa = client_sa_;
  fr.pf = static_cast<std::uint8_t>((PGN_AUTH_C2S >> 8) & 0xFFU);
  fr.ps = server_sa_;
  fr.data.fill(0xFF);
  fr.data[0] = msg_code;
  return fr;
}

ros2_isobus::msg::IsobusTpFrame DummyAuthProvider::make_auth_tp(
  std::uint8_t msg_code, std::size_t length, std::uint8_t round) const
{
  ros2_isobus::msg::IsobusTpFrame tp;
  tp.priority = 6;
  tp.page = false;
  tp.pgn = PGN_AUTH_C2S;
  tp.sa = client_sa_;
  tp.pf = static_cast<std::uint8_t>((PGN_AUTH_C2S >> 8) & 0xFFU);
  tp.ps = server_sa_;
  tp.data.assign(length, 0xFF);
  tp.data[0] = msg_code;
  if (length > 1U) tp.data[1] = 0x00;
  if (length > 2U) tp.data[2] = 0x0F;

  if (msg_code == kMsgAuthClientRnd) {
    // Match working reference layout: 32-byte length in bytes 5..6.
    if (length > 3U) tp.data[3] = 0xFF;
    if (length > 4U) tp.data[4] = 0xFF;
    if (length > 5U) tp.data[5] = 0x20;
    if (length > 6U) tp.data[6] = 0x00;
    for (std::size_t i = 7U; i < length; ++i) {
      tp.data[i] = static_cast<std::uint8_t>((i - 7U) & 0xFFU);
    }
    return tp;
  }

  if (msg_code == kMsgAuthClientFinalize) {
    // Signed challenge response: 16-byte signed payload length in bytes 5..6,
    // aligned with TIMlog.asc working sequence.
    if (length > 3U) tp.data[3] = 0xFF;
    if (length > 4U) tp.data[4] = 0xFF;
    if (length > 5U) tp.data[5] = 0x10;
    if (length > 6U) tp.data[6] = 0x00;
    for (std::size_t i = 7U; i < length; ++i) {
      tp.data[i] = static_cast<std::uint8_t>((i - 7U) & 0xFFU);
    }
    return tp;
  }

  // Certificate demo payload uses round in byte 4 and ASCII body.
  if (length > 3U) tp.data[3] = 0x00;
  if (length > 4U) tp.data[4] = round;
  if (length > 5U) tp.data[5] = 0x1A;
  if (length > 6U) tp.data[6] = 0x00;
  for (std::size_t i = 7U; i < length; ++i) {
    tp.data[i] = static_cast<std::uint8_t>(0x41U + ((i - 7U) % 26U));
  }
  return tp;
}

void DummyAuthProvider::send_auth_status(std::uint32_t now_ms, bool restart_bit)
{
  if ((now_ms - last_status_tx_ms_) < period_ms_) return;
  last_status_tx_ms_ = now_ms;

  auto fr = make_auth_frame(MSG_AUTH_CLIENT_STATUS);
  fr.data[1] = 0x00;  // no error
  fr.data[2] = static_cast<std::uint8_t>((0x0U << 4) | (authenticated_ ? 0x1U : 0x0U));
  std::uint8_t sub_status = 0x00U;
  if (certs_valid_) {
    sub_status = 0x3CU;
  }
  if (challenge_signed_local_) {
    sub_status = 0x3EU;
  }
  if (restart_bit) {
    sub_status = 0x80U;
  }
  fr.data[3] = sub_status;  // auth sub-status / (Re)Start during initialization
  fr.data[6] = implemented_version_;
  fr.data[7] = minimum_version_;
  sendFrame(fr);
  if (restart_bit) {
    logInfo("TIM auth: sent client auth status with restart request");
  }
}

void DummyAuthProvider::send_compat_response(std::uint8_t msg_code, std::uint8_t round)
{
  auto fr = make_auth_frame(msg_code);
  fr.data[1] = 0xFF;
  fr.data[2] = 0x0F;
  if (
    msg_code == kMsgAuthServerRnd || msg_code == kMsgAuthClientRnd ||
    msg_code == kMsgAuthServerFinalize || msg_code == kMsgAuthClientFinalize)
  {
    fr.data[3] = 0xFF;
    fr.data[4] = 0xFF;
    fr.data[5] = 0xFF;
    fr.data[6] = 0xFF;
    fr.data[7] = 0xFF;
  } else {
    fr.data[3] = 0x00;
    fr.data[4] = round;
    fr.data[5] = 0xFF;
    fr.data[6] = 0xFF;
    fr.data[7] = 0xFF;
  }
  sendFrame(fr);
}

void DummyAuthProvider::send_round_requests(std::uint32_t now_ms)
{
  if ((now_ms - last_round_tx_ms_) < 100U) return;
  last_round_tx_ms_ = now_ms;

  if (step_ == Step::WaitRandomTrigger) {
    if (!pending_server_rnd_request_) return;
    logInfo("TIM auth: server requested random challenge, sending response");
    send_compat_response(kMsgAuthServerRnd, cert_round_);
    sendTpFrame(make_auth_tp(kMsgAuthClientRnd, 39U, cert_round_));
    pending_server_rnd_request_ = false;
    step_ = Step::WaitRandomTp;
    step_deadline_ms_ = now_ms + kStepTimeoutMs;
    return;
  }

  if (step_ == Step::WaitRandomTp) {
    if (!got_server_tp_04_) return;
    got_server_tp_04_ = false;
    retries_ = 0;
    step_ = Step::WaitCertTrigger;
    step_deadline_ms_ = 0;
    return;
  }

  if (step_ == Step::WaitCertTrigger) {
    if (!pending_server_cert_request_) return;
    logInfo("TIM auth: server requested certificate, sending response");
    send_compat_response(kMsgAuthServerCert, cert_round_);
    sendTpFrame(make_auth_tp(kMsgAuthClientCert, 33U, cert_round_));
    pending_server_cert_request_ = false;
    step_ = Step::WaitCertTp;
    step_deadline_ms_ = now_ms + kStepTimeoutMs;
    return;
  }

  if (step_ == Step::WaitCertTp) {
    if (!got_server_tp_02_) return;
    got_server_tp_02_ = false;
    retries_ = 0;
    cert_round_ = static_cast<std::uint8_t>(cert_round_ + 1U);
    if (cert_round_ > kCertRoundsRequired) {
      certs_valid_ = true;
      last_status_tx_ms_ = 0;
      step_ = Step::WaitFinalizeTrigger;
    } else {
      step_ = Step::WaitCertTrigger;
    }
    step_deadline_ms_ = 0;
    return;
  }

  if (step_ == Step::WaitFinalizeTrigger) {
    // Request signed challenge first, then send TP response when server triggers it.
    if (!challenge_signed_local_) {
      challenge_signed_local_ = true;
      last_status_tx_ms_ = 0;
      return;
    }
    if (!challenge_signed_server_) return;
    if (!finalize_request_sent_) {
      // Request signed challenge with short 0x06 frame, send actual 0x07 payload via TP.
      send_compat_response(kMsgAuthServerFinalize, cert_round_);
      finalize_request_sent_ = true;
      step_deadline_ms_ = now_ms + kStepTimeoutMs;
      return;
    }
    if (!pending_server_finalize_request_) return;
    logInfo("TIM auth: server requested signed challenge, sending response payload");
    sendTpFrame(make_auth_tp(kMsgAuthClientFinalize, 23U, cert_round_));
    client_finalize_payload_sent_ = true;
    pending_server_finalize_request_ = false;
    step_ = Step::WaitFinalizeTp;
    step_deadline_ms_ = now_ms + kStepTimeoutMs;
    return;
  }

  if (step_ == Step::WaitFinalizeTp) {
    if (got_server_tp_06_) {
      got_server_tp_06_ = false;
      server_finalize_payload_seen_ = true;
      pending_server_finalize_request_ = false;
      retries_ = 0;
    }
    if (
      server_finalize_payload_seen_ &&
      client_finalize_payload_sent_ &&
      !client_authenticated_status_sent_)
    {
      auto fr = make_auth_frame(MSG_AUTH_CLIENT_STATUS);
      fr.data[1] = 0x00;
      fr.data[2] = 0x01;  // authenticated
      fr.data[3] = 0x3E;  // certificates valid + challenge signed
      fr.data[6] = implemented_version_;
      fr.data[7] = minimum_version_;
      sendFrame(fr);
      client_authenticated_status_sent_ = true;
    }
    if (
      server_finalize_payload_seen_ &&
      client_finalize_payload_sent_ &&
      client_authenticated_status_sent_ &&
      server_authenticated_seen_)
    {
      authenticated_ = true;
      logInfo("TIM auth: authentication completed (mutual final status exchange)");
      step_deadline_ms_ = 0;
    }
  }
}

void DummyAuthProvider::on_frame(const ros2_isobus::msg::IsobusFrame & fr)
{
  const std::uint8_t pf = fr.pf ? fr.pf : static_cast<std::uint8_t>((fr.pgn >> 8) & 0xFFU);
  const std::uint32_t pgn_from_pf = (static_cast<std::uint32_t>(pf) << 8U);
  if (pgn_from_pf != PGN_AUTH_S2C && pgn_from_pf != PGN_AUTH_C2S) return;
  if (fr.sa != server_sa_) return;
  if (fr.ps != client_sa_) return;

  const std::uint8_t msg = fr.data[0];
  if (msg == MSG_AUTH_SERVER_STATUS) {
    server_status_seen_ = true;
    const std::uint8_t err = fr.data[1];
    const std::uint8_t auth_type = static_cast<std::uint8_t>((fr.data[2] >> 4) & 0x0FU);
    const std::uint8_t auth_status = static_cast<std::uint8_t>(fr.data[2] & 0x0FU);
    challenge_signed_server_ = ((fr.data[3] & 0x02U) != 0U) || (fr.data[3] == 0x3EU);
    if (auth_type == 0x00U && err == 0x00U && auth_status == 0x01U) {
      server_authenticated_seen_ = true;
      logInfo("TIM auth: server status confirms authenticated");
    }
    return;
  }

  if (msg == kMsgAuthServerVersion) {
    pending_f8_request_ = true;
    logInfo("TIM auth: received server version request (F8)");
  } else if (msg == kMsgAuthServerRnd || msg == kMsgAuthClientRnd) {
    pending_server_rnd_request_ = true;
    logInfo("TIM auth: received server random-challenge request trigger");
  } else if (msg == kMsgAuthServerCert || msg == kMsgAuthClientCert) {
    pending_server_cert_request_ = true;
    logInfo("TIM auth: received server certificate request trigger");
  } else if (msg == kMsgAuthServerFinalize || msg == kMsgAuthClientFinalize) {
    pending_server_finalize_request_ = true;
    logInfo("TIM auth: received server finalize request trigger");
  }
}

void DummyAuthProvider::on_tp_payload(std::uint32_t pgn, const std::vector<std::uint8_t> & payload)
{
  if ((pgn != PGN_AUTH_S2C && pgn != PGN_AUTH_C2S) || payload.empty()) return;
  if (payload[0] == kMsgAuthServerRnd) {
    got_server_tp_04_ = true;
    logInfo("TIM auth: received TP server random payload");
  } else if (payload[0] == kMsgAuthServerCert) {
    got_server_tp_02_ = true;
    logInfo("TIM auth: received TP server certificate payload");
  } else if (payload[0] == kMsgAuthServerFinalize) {
    got_server_tp_06_ = true;
    server_finalize_payload_seen_ = true;
    logInfo("TIM auth: received TP server finalize payload");
  }
}

void DummyAuthProvider::process(std::uint32_t now_ms)
{
  if (client_sa_ > 0xFDU || server_sa_ > 0xFDU) return;

  // Always keep status alive during authentication process.
  send_auth_status(now_ms, !restart_sent_);
  restart_sent_ = true;

  if (authenticated_) return;
  if (failed_) return;
  if (!server_status_seen_) return;

  // Reply to server version request only when requested (F8).
  if (pending_f8_request_) {
    auto fr = make_auth_frame(kMsgAuthClientVersion);
    fr.data[1] = 0x00;
    fr.data[2] = implemented_version_;
    fr.data[3] = minimum_version_;
    sendFrame(fr);
    pending_f8_request_ = false;
    logInfo("TIM auth: sent client version response (F7)");
  }

  if (step_deadline_ms_ != 0 && now_ms > step_deadline_ms_) {
    if (retries_ < kMaxRetries) {
      retries_ = static_cast<std::uint8_t>(retries_ + 1U);
      if (step_ == Step::WaitRandomTp) {
        step_ = Step::WaitRandomTrigger;
      } else if (step_ == Step::WaitCertTp) {
        step_ = Step::WaitCertTrigger;
      } else if (step_ == Step::WaitFinalizeTrigger) {
        finalize_request_sent_ = false;
      } else if (step_ == Step::WaitFinalizeTp) {
        step_ = Step::WaitFinalizeTrigger;
        finalize_request_sent_ = false;
        client_finalize_payload_sent_ = false;
        server_finalize_payload_seen_ = false;
        client_authenticated_status_sent_ = false;
      }
      step_deadline_ms_ = 0;
      logWarn("TIM auth: step timeout, retrying");
    } else {
      failed_ = true;
      logError("TIM auth: failed after maximum retries");
      return;
    }
  }

  send_round_requests(now_ms);
}

bool DummyAuthProvider::is_authenticated() const
{
  return authenticated_;
}

bool DummyAuthProvider::is_lead_server() const
{
  return authenticated_;
}

bool DummyAuthProvider::is_failed() const
{
  return failed_;
}

}  // namespace ros2_isobus
