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

#include "TIMClient.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ros2_isobus
{

static constexpr std::uint8_t MAX_REQ_ATTEMPTS = 4; // initial + 3 retries
static constexpr std::uint32_t PGN_ACKM = 0xE800;   // J1939/ISOBUS Acknowledgement Message
static constexpr std::uint32_t ASSIGNMENT_RESPONSE_TIMEOUT_MS = 1500;
static constexpr std::uint32_t PARSE_WARN_SUPPORT_SHORT = (1U << 0);
static constexpr std::uint32_t PARSE_WARN_SUPPORT_TUPLES = (1U << 1);
static constexpr std::uint32_t PARSE_WARN_ASSIGN_SHORT = (1U << 2);
static constexpr std::uint32_t PARSE_WARN_ASSIGN_STATUS_SHORT = (1U << 3);

// Decode little-endian u16 at offset idx from TIM frame payload.
std::uint16_t TimClient::u16_le(const std::array<std::uint8_t,8> & d, std::size_t idx)
{
  return static_cast<std::uint16_t>(d[idx]) |
         (static_cast<std::uint16_t>(d[idx + 1]) << 8);
}

// Encode little-endian u16 at offset idx into TIM frame payload.
void TimClient::put_u16_le(std::array<std::uint8_t,8> & d, std::size_t idx, std::uint16_t v)
{
  d[idx]     = static_cast<std::uint8_t>(v & 0xFFu);
  d[idx + 1] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
}

// Convert engineering value to raw TIM SLOT encoding with clamp to 16-bit range.
std::uint16_t TimClient::enc_u16(float value, float resolution, float offset)
{
  // raw = (value - offset) / resolution
  const float rawf = (value - offset) / resolution;
  long r = static_cast<long>(std::lround(rawf));
  if (r < 0) r = 0;
  if (r > 65535) r = 65535;
  return static_cast<std::uint16_t>(r);
}

// Convert raw TIM SLOT value to engineering unit.
float TimClient::dec_u16(std::uint16_t raw, float resolution, float offset)
{
  return (static_cast<float>(raw) * resolution) + offset;
}

std::uint8_t TimClient::next_heartbeat_counter()
{
  // First transmitted value after initialization is 0xFB, then sequence 0..250.
  if (!heartbeat_initialized_) {
    heartbeat_ = 0xFB;
    heartbeat_initialized_ = true;
    return heartbeat_;
  }
  if (heartbeat_ == 0xFB || heartbeat_ >= 0xFA) {
    heartbeat_ = 0x00;
  } else {
    heartbeat_ = static_cast<std::uint8_t>(heartbeat_ + 1U);
  }
  return heartbeat_;
}

std::uint8_t TimClient::next_v1_request_counter()
{
  if (v1_req_counter_ >= 0x0D) {
    v1_req_counter_ = 0x00;
  } else {
    v1_req_counter_ = static_cast<std::uint8_t>(v1_req_counter_ + 1U);
  }
  return v1_req_counter_;
}

bool TimClient::is_automation_active(std::uint8_t s) const
{
  return s == TIM_AUTO_ACTIVE || s == TIM_AUTO_ACTIVE_LIMIT_HIGH || s == TIM_AUTO_ACTIVE_LIMIT_LOW;
}

bool TimClient::is_automation_pending(std::uint8_t s) const
{
  return s == TIM_AUTO_PENDING;
}

bool TimClient::is_function_enabled(std::uint8_t fn) const
{
  for (std::size_t i = 0; i < cfg_.num_aux && i < MAX_AUX; ++i) {
    if (cfg_.aux_fn_id[i] == fn) return targets_.aux_enable[i];
  }
  if (cfg_.enable_speed && fn == cfg_.speed_fn_id) return targets_.speed_enable;
  if (cfg_.enable_curvature && fn == cfg_.curvature_fn_id) return targets_.curvature_enable;
  if (cfg_.enable_rear_pto && fn == cfg_.rear_pto_fn_id) return targets_.rear_pto_enable;
  if (cfg_.enable_rear_hitch && fn == cfg_.rear_hitch_fn_id) return targets_.rear_hitch_enable;
  return false;
}

std::uint8_t TimClient::compute_effective_automation_status() const
{
  std::size_t enabled_count = 0;
  std::size_t active_count = 0;
  bool any_fault = false;
  bool any_pending = false;
  bool any_enabled = false;
  bool any_ready = false;
  bool any_not_ready = false;
  bool any_limited_high = false;
  bool any_limited_low = false;

  const auto fold = [&](bool enabled, bool valid, std::uint8_t status) {
    if (!enabled) return;
    ++enabled_count;
    const std::uint8_t s = valid ? status : TIM_AUTO_UNAVAILABLE;
    if (s == TIM_AUTO_FAULT) {
      any_fault = true;
      return;
    }
    if (s == TIM_AUTO_PENDING) any_pending = true;
    if (s == TIM_AUTO_ACTIVE || s == TIM_AUTO_ACTIVE_LIMIT_HIGH || s == TIM_AUTO_ACTIVE_LIMIT_LOW) {
      ++active_count;
      if (s == TIM_AUTO_ACTIVE_LIMIT_HIGH) any_limited_high = true;
      if (s == TIM_AUTO_ACTIVE_LIMIT_LOW) any_limited_low = true;
      return;
    }
    if (s == TIM_AUTO_ENABLED) any_enabled = true;
    else if (s == TIM_AUTO_READY_TO_ENABLE) any_ready = true;
    else any_not_ready = true;
  };

  for (std::size_t i = 0; i < cfg_.num_aux && i < MAX_AUX; ++i) {
    fold(targets_.aux_enable[i], actuals_.aux_valid[i], actuals_.aux_automation_status[i]);
  }
  if (cfg_.enable_speed) {
    fold(targets_.speed_enable, actuals_.speed_valid, actuals_.speed_automation_status);
  }
  if (cfg_.enable_curvature) {
    fold(targets_.curvature_enable, actuals_.curvature_valid, actuals_.curvature_automation_status);
  }
  if (cfg_.enable_rear_pto) {
    fold(targets_.rear_pto_enable, actuals_.rear_pto_valid, actuals_.rear_pto_automation_status);
  }
  if (cfg_.enable_rear_hitch) {
    fold(targets_.rear_hitch_enable, actuals_.rear_hitch_valid, actuals_.rear_hitch_automation_status);
  }

  if (enabled_count == 0) return TIM_AUTO_ENABLED;
  if (any_fault) return TIM_AUTO_FAULT;
  if (any_pending) return TIM_AUTO_PENDING;
  if (active_count == enabled_count) {
    if (any_limited_high) return TIM_AUTO_ACTIVE_LIMIT_HIGH;
    if (any_limited_low) return TIM_AUTO_ACTIVE_LIMIT_LOW;
    return TIM_AUTO_ACTIVE;
  }
  if (active_count > 0) return TIM_AUTO_PENDING;
  if (any_enabled) return TIM_AUTO_ENABLED;
  if (any_ready) return TIM_AUTO_READY_TO_ENABLE;
  if (any_not_ready) return TIM_AUTO_NOT_READY;
  return TIM_AUTO_UNAVAILABLE;
}

void TimClient::update_function_control_states()
{
  const auto map_state = [](bool enabled, bool valid, std::uint8_t status) -> FunctionControlState {
    if (!enabled) {
      if (status == TIM_AUTO_ACTIVE || status == TIM_AUTO_ACTIVE_LIMIT_HIGH ||
          status == TIM_AUTO_ACTIVE_LIMIT_LOW || status == TIM_AUTO_PENDING)
      {
        return FunctionControlState::Releasing;
      }
      return FunctionControlState::Disabled;
    }
    if (!valid) return FunctionControlState::ReadyToEnable;
    switch (status) {
      case TIM_AUTO_FAULT: return FunctionControlState::Fault;
      case TIM_AUTO_ACTIVE:
      case TIM_AUTO_ACTIVE_LIMIT_HIGH:
      case TIM_AUTO_ACTIVE_LIMIT_LOW: return FunctionControlState::Active;
      case TIM_AUTO_PENDING: return FunctionControlState::Pending;
      // When function is enabled but not yet pending/active, keep sending
      // READY_TO_CONTROL requests to (re-)arm control transition.
      case TIM_AUTO_ENABLED: return FunctionControlState::ReadyToEnable;
      case TIM_AUTO_READY_TO_ENABLE: return FunctionControlState::ReadyToEnable;
      default: return FunctionControlState::ReadyToEnable;
    }
  };

  for (std::size_t i = 0; i < cfg_.num_aux && i < MAX_AUX; ++i) {
    aux_control_state_[i] = map_state(targets_.aux_enable[i], actuals_.aux_valid[i], actuals_.aux_automation_status[i]);
  }
  speed_control_state_ = map_state(targets_.speed_enable, actuals_.speed_valid, actuals_.speed_automation_status);
  curvature_control_state_ = map_state(targets_.curvature_enable, actuals_.curvature_valid, actuals_.curvature_automation_status);
  rear_pto_control_state_ = map_state(targets_.rear_pto_enable, actuals_.rear_pto_valid, actuals_.rear_pto_automation_status);
  rear_hitch_control_state_ = map_state(targets_.rear_hitch_enable, actuals_.rear_hitch_valid, actuals_.rear_hitch_automation_status);
}

bool TimClient::has_controlled_functions() const
{
  auto active_state = [](FunctionControlState s) {
    return s != FunctionControlState::Disabled;
  };
  for (std::size_t i = 0; i < cfg_.num_aux && i < MAX_AUX; ++i) {
    if (active_state(aux_control_state_[i])) return true;
  }
  return active_state(speed_control_state_) ||
         active_state(curvature_control_state_) ||
         active_state(rear_pto_control_state_) ||
         active_state(rear_hitch_control_state_);
}

DiagState TimClient::diag_from_automation_status(std::uint8_t s) const
{
  switch (s) {
    case TIM_AUTO_UNAVAILABLE: return DiagState::Unavailable;
    case TIM_AUTO_NOT_READY: return DiagState::NotReady;
    case TIM_AUTO_READY_TO_ENABLE: return DiagState::Ready;
    case TIM_AUTO_ENABLED: return DiagState::Enabled;
    case TIM_AUTO_PENDING: return DiagState::Pending;
    case TIM_AUTO_ACTIVE: return DiagState::Active;
    case TIM_AUTO_ACTIVE_LIMIT_HIGH: return DiagState::ActiveLimitedHigh;
    case TIM_AUTO_ACTIVE_LIMIT_LOW: return DiagState::ActiveLimitedLow;
    case TIM_AUTO_FAULT: return DiagState::Error;
    default: return DiagState::NotReady;
  }
}

bool TimClient::should_send_function(std::uint8_t fn) const
{
  if (requested_fns_.empty()) {
    return true;
  }
  return std::find(requested_fns_.begin(), requested_fns_.end(), fn) != requested_fns_.end();
}

bool TimClient::is_special_mode() const
{
  return request_mode_ != RequestMode::Setpoint;
}

bool TimClient::use_handshake_setpoint_reference() const
{
  return false;
}

void TimClient::validate_server_heartbeat(std::uint8_t hb)
{
  if (hb == 0xFF) {
    server_shutdown_seen_ = true;
    logError("TIM communication error: server heartbeat indicates shutdown (0xFF)");
    comm_error_ = true;
    return;
  }
  if (hb == 0xFE || hb == 0xFC || hb == 0xFD) {
    logError("TIM communication error: server heartbeat contains reserved/error value");
    comm_error_ = true;
    return;
  }
  if (hb == 0xFB) {
    have_server_heartbeat_ = false;
    return;
  }
  if (hb > 0xFA) {
    logError("TIM communication error: server heartbeat out of valid range");
    comm_error_ = true;
    return;
  }

  if (!have_server_heartbeat_) {
    server_heartbeat_prev_ = hb;
    have_server_heartbeat_ = true;
    return;
  }

  const std::uint16_t prev = server_heartbeat_prev_;
  const std::uint16_t curr = hb;
  const std::uint16_t delta = static_cast<std::uint16_t>((curr + 251U - prev) % 251U);
  if (delta == 0U || delta > 3U) {
    logError("TIM communication error: server heartbeat sequence jump/out-of-order");
    comm_error_ = true;
  }
  server_heartbeat_prev_ = hb;
}

void TimClient::observe_v1_status(std::uint8_t reflected_counter, std::uint8_t automation_status)
{
  const std::uint32_t now = nowMs();
  last_function_status_rx_ms_ = now;
  if (awaiting_release_status_ && release_request_since_ms_ != 0U && now >= release_request_since_ms_) {
    awaiting_release_status_ = false;
  }
  if (automation_status == TIM_AUTO_FAULT) {
    logError("TIM communication error: v1 function status reported automation fault");
    comm_error_ = true;
    return;
  }

  if (reflected_counter == 0x0E) {
    logError("TIM communication error: v1 reflected request counter invalid (0x0E)");
    comm_error_ = true;
    return;
  }
  if (awaiting_v1_reflection_ && reflected_counter <= 0x0D) {
    awaiting_v1_reflection_ = false;
  }
  if (is_automation_pending(automation_status) && pending_since_ms_ == 0U) {
    pending_since_ms_ = now;
  }
}

void TimClient::observe_v2_status(std::uint8_t automation_status)
{
  const std::uint32_t now = nowMs();
  last_function_status_rx_ms_ = now;
  if (awaiting_release_status_ && release_request_since_ms_ != 0U && now >= release_request_since_ms_) {
    awaiting_release_status_ = false;
  }
  if (automation_status == TIM_AUTO_FAULT) {
    logError("TIM communication error: v2 function status reported automation fault");
    comm_error_ = true;
    return;
  }
  if (is_automation_pending(automation_status) && pending_since_ms_ == 0U) {
    pending_since_ms_ = last_function_status_rx_ms_;
  }
}

// Initialize TIM client with static peer addresses, version bounds and function configuration.
TimClient::TimClient(std::uint8_t sa_client,
                     std::uint8_t sa_server,
                     std::uint8_t implemented_version,
                     std::uint8_t minimum_version,
                     const FunctionConfig & cfg,
                     const TimingConfig & timing)
: sa_client_(sa_client),
  sa_server_(sa_server),
  implemented_version_(implemented_version),
  minimum_version_(minimum_version),
  cfg_(cfg),
  timing_(timing)
{
  // Fill missing v2 control mode entries with default percent-flow mode for AUX functions.
  for (std::size_t i = 0; i < cfg_.aux_ctrl_mode.size(); ++i) {
    if (cfg_.aux_ctrl_mode[i] == 0) cfg_.aux_ctrl_mode[i] = CM_AUX_PERCENT_FLOW;
  }
}

// Set AUX target flow [%] and enable flag for function request generation.
void TimClient::set_aux_flow(std::size_t idx, float pct_flow, bool enable)
{
  if (idx >= MAX_AUX) return;
  pct_flow = std::clamp(pct_flow, -100.0f, 100.0f);
  targets_.aux_flow_pct[idx] = pct_flow;
  targets_.aux_enable[idx] = enable;
  send_aux_request_immediate(idx);
}

// Set vehicle speed target [m/s], clamped to TIM SLOT domain.
void TimClient::set_speed_mps(float mps, bool enable)
{
  // Vehicle speed SLOT encoding (AEF 023 RIG 4, Annex D / 10.6.2.x).
  mps = std::clamp(mps, -32.127f, 32.127f);
  targets_.speed_mps = mps;
  targets_.speed_enable = enable;
  send_speed_request_immediate();
}

// Set guidance curvature target [km^-1], clamped to TIM SLOT domain.
void TimClient::set_curvature_km_inv(float km_inv, bool enable)
{
  // Curvature SLOT encoding (AEF 023 RIG 4, Annex D / 10.x).
  km_inv = std::clamp(km_inv, -8032.0f, 8031.75f);
  targets_.curvature_km_inv = km_inv;
  targets_.curvature_enable = enable;
  send_curvature_request_immediate();
}

// Set rear PTO target [rpm], clamped to TIM SLOT domain.
void TimClient::set_rear_pto_rpm(float rpm, bool enable)
{
  // PTO SLOT encoding (AEF 023 RIG 4, Annex D / 10.x).
  rpm = std::max(0.0f, rpm);
  targets_.rear_pto_rpm = rpm;
  targets_.rear_pto_enable = enable;
  send_rear_pto_request_immediate();
}

// Set rear hitch target [%], clamped to TIM SLOT domain.
void TimClient::set_rear_hitch_pct(float pct, bool enable)
{
  // Hitch SLOT encoding (AEF 023 RIG 4, Annex D / 10.x).
  pct = std::clamp(pct, 0.0f, 100.0f);
  targets_.rear_hitch_pct = pct;
  targets_.rear_hitch_enable = enable;
  send_rear_hitch_request_immediate();
}

TimClient::RequestMode TimClient::mode_from_control_state(FunctionControlState s) const
{
  switch (s) {
    case FunctionControlState::Releasing: return RequestMode::ReleaseOperatorAction;
    case FunctionControlState::ReadyToEnable: return RequestMode::ReadyToControl;
    case FunctionControlState::Enabled:
    case FunctionControlState::Pending:
    case FunctionControlState::Active: return RequestMode::Setpoint;
    case FunctionControlState::Fault: return RequestMode::ReadyToControl;
    case FunctionControlState::Disabled:
    default: return RequestMode::ReleaseAcceptIncrease;
  }
}

void TimClient::update_release_tracking_after_function_tx(std::uint32_t now_ms)
{
  // Release/request-mode bookkeeping corresponds to AEF 023 operational request
  // semantics where control requests may switch from setpoint to release variants.
  last_func_tx_ms_ = now_ms;
  last_function_req_tx_ms_ = now_ms;
  const bool release_mode =
      (request_mode_ == RequestMode::ReleaseOperatorAction) ||
      (request_mode_ == RequestMode::ReleaseAcceptIncrease);
  if (!release_mode) {
    release_mode_active_ = false;
    awaiting_release_status_ = false;
    release_request_since_ms_ = 0;
    return;
  }
  if (!release_mode_active_) {
    release_mode_active_ = true;
    awaiting_release_status_ = true;
    release_request_since_ms_ = now_ms;
  }
}

void TimClient::send_aux_request_immediate(std::size_t idx)
{
  // Direct mode: emit only the updated function request immediately after setter update.
  // This keeps bus load focused to changed function while preserving Annex D encoding.
  if (!use_direct_commands() || state_ != State::Operational) return;
  if (idx >= cfg_.num_aux || idx >= MAX_AUX) return;

  update_function_control_states();
  const auto control_state = aux_control_state_[idx];
  if (control_state == FunctionControlState::Disabled) return;
  if (!should_send_function(cfg_.aux_fn_id[idx])) return;

  const auto mode = mode_from_control_state(control_state);
  if (negotiated_version_ <= 1) send_aux_req_v1(idx, mode);
  else send_aux_req_v2(idx, mode);
  update_release_tracking_after_function_tx(nowMs());
}

void TimClient::send_speed_request_immediate()
{
  // Direct mode immediate send for vehicle speed function.
  if (!use_direct_commands() || state_ != State::Operational || !cfg_.enable_speed) return;

  update_function_control_states();
  if (speed_control_state_ == FunctionControlState::Disabled) return;
  if (!should_send_function(cfg_.speed_fn_id)) return;

  const auto mode = mode_from_control_state(speed_control_state_);
  if (negotiated_version_ <= 1) send_speed_req_v1(mode);
  else send_speed_req_v2(mode);
  update_release_tracking_after_function_tx(nowMs());
}

void TimClient::send_curvature_request_immediate()
{
  // Direct mode immediate send for guidance curvature function.
  if (!use_direct_commands() || state_ != State::Operational || !cfg_.enable_curvature) return;

  update_function_control_states();
  if (curvature_control_state_ == FunctionControlState::Disabled) return;
  if (!should_send_function(cfg_.curvature_fn_id)) return;

  const auto mode = mode_from_control_state(curvature_control_state_);
  if (negotiated_version_ <= 1) send_curv_req_v1(mode);
  else send_curv_req_v2(mode);
  update_release_tracking_after_function_tx(nowMs());
}

void TimClient::send_rear_pto_request_immediate()
{
  // Direct mode immediate send for rear PTO function.
  if (!use_direct_commands() || state_ != State::Operational || !cfg_.enable_rear_pto) return;

  update_function_control_states();
  if (rear_pto_control_state_ == FunctionControlState::Disabled) return;
  if (!should_send_function(cfg_.rear_pto_fn_id)) return;

  const auto mode = mode_from_control_state(rear_pto_control_state_);
  if (negotiated_version_ <= 1) send_pto_req_v1(mode);
  else send_pto_req_v2(mode);
  update_release_tracking_after_function_tx(nowMs());
}

void TimClient::send_rear_hitch_request_immediate()
{
  // Direct mode immediate send for rear hitch function.
  if (!use_direct_commands() || state_ != State::Operational || !cfg_.enable_rear_hitch) return;

  update_function_control_states();
  if (rear_hitch_control_state_ == FunctionControlState::Disabled) return;
  if (!should_send_function(cfg_.rear_hitch_fn_id)) return;

  const auto mode = mode_from_control_state(rear_hitch_control_state_);
  if (negotiated_version_ <= 1) send_hitch_req_v1(mode);
  else send_hitch_req_v2(mode);
  update_release_tracking_after_function_tx(nowMs());
}

// Set operator gate used by TIM client state transition to Active.
void TimClient::set_operator_enable(bool enable)
{
  operator_enable_ = enable;
}

void TimClient::set_timing_config(const TimingConfig & timing)
{
  timing_ = timing;
}

void TimClient::set_client_name(std::uint64_t client_name)
{
  client_name_known_ = true;
  client_name_ = client_name;
}

void TimClient::set_auth_provider(std::shared_ptr<IAuthProvider> auth_provider)
{
  auth_provider_ = std::move(auth_provider);
  if (auth_provider_) {
    auth_provider_->reset(sa_client_, sa_server_);
  }
}

void TimClient::set_client_sa(std::uint8_t sa_client)
{
  if (sa_client_ == sa_client) return;
  sa_client_ = sa_client;
  comm_error_ = false;
  server_shutdown_seen_ = false;
  have_server_heartbeat_ = false;
  heartbeat_ = 0;
  heartbeat_initialized_ = false;
  v1_req_counter_ = 0;
  graceful_shutdown_requested_ = false;
  graceful_shutdown_sent_ = false;
  last_client_status_ms_ = 0;
  last_server_status_rx_ms_ = 0;
  last_conn_req_ms_ = 0;
  last_support_req_ms_ = 0;
  last_assign_req_ms_ = 0;
  last_assign_status_req_ms_ = 0;
  last_func_tx_ms_ = 0;
  conn_req_sent_once_ = false;
  if (auth_provider_) {
    auth_provider_->reset(sa_client_, sa_server_);
  }
  reset_session_state();
}

void TimClient::set_server_sa(std::uint8_t sa_server)
{
  if (sa_server_ == sa_server) return;
  sa_server_ = sa_server;
  comm_error_ = false;
  server_shutdown_seen_ = false;
  have_server_heartbeat_ = false;
  heartbeat_ = 0;
  heartbeat_initialized_ = false;
  v1_req_counter_ = 0;
  graceful_shutdown_requested_ = false;
  graceful_shutdown_sent_ = false;
  last_client_status_ms_ = 0;
  last_server_status_rx_ms_ = 0;
  last_conn_req_ms_ = 0;
  last_support_req_ms_ = 0;
  last_assign_req_ms_ = 0;
  last_assign_status_req_ms_ = 0;
  last_func_tx_ms_ = 0;
  conn_req_sent_once_ = false;
  if (auth_provider_) {
    auth_provider_->reset(sa_client_, sa_server_);
  }
  reset_session_state();
}

void TimClient::request_graceful_shutdown()
{
  graceful_shutdown_requested_ = true;
}

bool TimClient::is_server_authorized() const
{
  // No provider means auth is bypassed by configuration (auth_mode=None).
  if (!auth_provider_) {
    return true;
  }
  // With a provider, always use provider state.
  if (auth_provider_) {
    return auth_provider_->is_authenticated() && auth_provider_->is_lead_server();
  }
  return false;
}

void TimClient::log_parse_warn_once(std::uint32_t bit, const std::string & msg)
{
  if ((parse_warn_mask_ & bit) != 0U) return;
  parse_warn_mask_ |= bit;
  logWarn(msg);
}

void TimClient::reset_session_state()
{
  state_ = State::Discover;
  // First TIM_ClientStatus after reset should start from "Unavailable" (0),
  // then process() advances to "NotReady" (1) in Discover.
  diag_ = DiagState::Unavailable;
  negotiated_version_ = 0;
  pending_response_ = PendingResponse::None;
  version_req_attempts_ = 0;
  support_req_attempts_ = 0;
  assign_req_attempts_ = 0;
  assign_status_req_attempts_ = 0;
  support_done_ = false;
  support_ok_ = false;
  assign_done_ = false;
  assign_ok_ = false;
  assign_status_done_ = false;
  assign_status_ok_ = false;
  conn_req_sent_once_ = false;
  requested_fns_.clear();
  supported_fns_.clear();
  last_function_status_rx_ms_ = 0;
  last_function_req_tx_ms_ = 0;
  pending_since_ms_ = 0;
  awaiting_v1_reflection_ = false;
  release_mode_active_ = false;
  awaiting_release_status_ = false;
  release_request_since_ms_ = 0;
  ready_to_control_sent_ = false;
  aux_control_state_.fill(FunctionControlState::Disabled);
  speed_control_state_ = FunctionControlState::Disabled;
  curvature_control_state_ = FunctionControlState::Disabled;
  rear_pto_control_state_ = FunctionControlState::Disabled;
  rear_hitch_control_state_ = FunctionControlState::Disabled;
  parse_warn_mask_ = 0;
}

void TimClient::start_nack_cooldown()
{
  nack_cooldown_active_ = true;
  nack_cooldown_until_ms_ = nowMs() + 300U;
}

void TimClient::send_client_status_with_heartbeat(std::uint8_t hb)
{
  std::array<std::uint8_t,8> d{};
  d.fill(0xFF);
  d[0] = MSG_CLIENT_STATUS;
  d[1] = hb;
  // TIM_ClientStatus byte 3:
  // bits 8..5 = TIM client state, bits 4..1 = reserved.
  std::uint8_t tim_client_state = static_cast<std::uint8_t>(diag_) & 0x0F;
  // CANoe TIM server expects no "pending" value in TIM_ClientStatus state nibble.
  // Keep sending enabled while internal control state is pending.
  if (diag_ == DiagState::Pending) {
    tim_client_state = static_cast<std::uint8_t>(DiagState::Enabled) & 0x0F;
  }
  d[2] = static_cast<std::uint8_t>((tim_client_state << 4) | 0x0F);

  // TIM_ClientStatus uses TIM C2S operation PGN 0x2400.
  sendFrame(make_frame(PGN_TIM_V1_C2S, 4, sa_client_, sa_server_, d));
}

// Build one ISOBUS frame and apply PDU1/PDU2 PS semantics.
ros2_isobus::msg::IsobusFrame TimClient::make_frame(std::uint32_t pgn, std::uint8_t priority,
                                                    std::uint8_t sa, std::uint8_t da,
                                                    const std::array<std::uint8_t,8> & data) const
{
  ros2_isobus::msg::IsobusFrame fr;
  fr.pgn = pgn;
  fr.page = static_cast<bool>((pgn >> 16) & 0x01u);
  fr.priority = priority;
  fr.sa = sa;

  fr.pf = static_cast<std::uint8_t>((pgn >> 8) & 0xFFu);
  // For PF<240 (PDU1), PS MUST be destination address.
  // For PF>=240 (PDU2), PS is group extension (part of PGN).
  if (fr.pf < 240) {
    fr.ps = da;
  } else {
    fr.ps = static_cast<std::uint8_t>(pgn & 0xFFu);
  }

  fr.data = data;
  return fr;
}

// Send variable-length payload as single-frame or TP frame.
void TimClient::send_var(std::uint32_t pgn, std::uint8_t priority, std::uint8_t da,
                const std::vector<std::uint8_t> & payload)
{
  if (payload.size() <= 8) {
    std::array<std::uint8_t,8> d{};
    d.fill(0xFF);
    for (std::size_t i = 0; i < payload.size(); ++i) d[i] = payload[i];
    sendFrame(make_frame(pgn, priority, sa_client_, da, d));
    return;
  }
  ros2_isobus::msg::IsobusTpFrame tp;
  tp.pgn = pgn;
  tp.page = static_cast<bool>((pgn >> 16) & 0x01u);
  tp.priority = priority;
  tp.sa = sa_client_;
  tp.pf = static_cast<std::uint8_t>((pgn >> 8) & 0xFF);
  tp.ps = (tp.pf < 240) ? da : static_cast<std::uint8_t>(pgn & 0xFF);
  tp.data = payload;
  sendTpFrame(tp);
}

// Periodic TIM_ClientStatus transmission with heartbeat (AEF 023 RIG 4, 6.3.3.1 and 9.3).
void TimClient::send_client_status()
{
  const std::uint32_t now = nowMs();
  if (now - last_client_status_ms_ < timing_.client_status_period_ms) return;
  last_client_status_ms_ = now;

  const std::uint8_t hb = next_heartbeat_counter();
  send_client_status_with_heartbeat(hb);
}

// Periodic TIM_ConnectionVersionRequest transmission (AEF 023 RIG 4, 6.3.4 and 8.2).
void TimClient::send_connection_version_req()
{
  const std::uint32_t now = nowMs();
  if (now - last_conn_req_ms_ < timing_.conn_req_period_ms) return;
  if (version_req_attempts_ >= MAX_REQ_ATTEMPTS) {
    logError("TIM handshake failed: connection version response timeout/retries exceeded");
    comm_error_ = true;
    state_ = State::Fault;
    diag_ = DiagState::Error;
    return;
  }
  last_conn_req_ms_ = now;
  version_req_attempts_ = static_cast<std::uint8_t>(version_req_attempts_ + 1U);
  pending_response_ = PendingResponse::Version;

  std::array<std::uint8_t,8> d{};
  d.fill(0xFF);
  d[0] = MSG_CONNECTION_VERSION;
  // TIM_ConnectionVersionRequest fields start after reserved byte:
  // data[1] reserved, data[2] implemented version, data[3] minimum version.
  d[2] = implemented_version_;
  d[3] = minimum_version_;

  // TIM_ConnectionVersionRequest uses TIM C2S operation PGN 0x2400.
  sendFrame(make_frame(PGN_TIM_V1_C2S, 6, sa_client_, sa_server_, d));
  conn_req_sent_once_ = true;
}

// Handle TIM_ConnectionVersionResponse and clamp to supported local version range.
void TimClient::handle_connection_version_resp(const ros2_isobus::msg::IsobusFrame & fr)
{
  if (fr.data[0] != MSG_CONNECTION_VERSION) return;
  if (pending_response_ != PendingResponse::Version) return;
  const std::uint8_t err = fr.data[1];
  if (err != 0) {
    logError("TIM handshake failed: connection version response returned error");
    state_ = State::Fault;
    diag_ = DiagState::Error;
    comm_error_ = true;
    return;
  }
  negotiated_version_ = fr.data[2];
  if (negotiated_version_ < 1) negotiated_version_ = 1;
  if (negotiated_version_ > 2) negotiated_version_ = 2;
  pending_response_ = PendingResponse::None;
  version_req_attempts_ = 0;

  char msg[192];
  std::snprintf(
      msg, sizeof(msg),
      "TIM handshake: ConnectionVersion OK (implemented=%u minimum=%u negotiated=%u)",
      static_cast<unsigned>(implemented_version_),
      static_cast<unsigned>(minimum_version_),
      static_cast<unsigned>(negotiated_version_));
  logInfo(msg);
}

// Update server status watchdog timestamp from TIM_ServerStatus.
void TimClient::handle_server_status(const ros2_isobus::msg::IsobusFrame & fr)
{
  if (fr.data[0] != MSG_SERVER_STATUS) return;
  if (fr.data[4] != 0xFF || fr.data[5] != 0xFF || fr.data[6] != 0xFF || fr.data[7] != 0xFF) {
    logWarn("TIM server requested shutdown via server status payload");
    request_graceful_shutdown();
    return;
  }

  const std::uint8_t lead_ind = static_cast<std::uint8_t>((fr.data[2] >> 4) & 0x0F);
  const std::uint8_t tim_system_state = static_cast<std::uint8_t>(fr.data[2] & 0x0F);
  const std::uint8_t tim_server_state = static_cast<std::uint8_t>((fr.data[3] >> 4) & 0x0F);
  const std::uint8_t tim_op_state = static_cast<std::uint8_t>(fr.data[3] & 0x0F);

  const bool lead_ok = (lead_ind == 0x0U || lead_ind == 0x1U);
  const bool system_state_ok = (tim_system_state == 0x1U || tim_system_state == 0x5U);
  const bool server_state_ok = (tim_server_state == 0x0U || tim_server_state == 0x1U ||
                                tim_server_state == 0x3U || tim_server_state == 0x5U);
  const bool op_state_ok = (tim_op_state <= 0x3U);
  if (!lead_ok || !system_state_ok || !server_state_ok || !op_state_ok) {
    logWarn("TIM server status invalid for control state, requesting graceful shutdown");
    request_graceful_shutdown();
    return;
  }

  validate_server_heartbeat(fr.data[1]);
  last_server_status_rx_ms_ = nowMs();
}

void TimClient::handle_acknowledgement(const ros2_isobus::msg::IsobusFrame & fr)
{
  if (fr.sa != sa_server_) return;
  if (fr.ps != sa_client_) return;

  const std::uint8_t control = fr.data[0];
  const std::uint8_t group_function = fr.data[1];
  const std::uint8_t nack_address = fr.data[4];
  const std::uint32_t acked_pgn = static_cast<std::uint32_t>(fr.data[5]) |
                                  (static_cast<std::uint32_t>(fr.data[6]) << 8U) |
                                  (static_cast<std::uint32_t>(fr.data[7]) << 16U);
  const char * control_text = "UNKNOWN";
  switch (control) {
    case 0x00: control_text = "ACK"; break;
    case 0x01: control_text = "NACK"; break;
    case 0x02: control_text = "AccessDenied"; break;
    case 0x03: control_text = "CannotRespond"; break;
    default: break;
  }

  if (control == 0x00) return;

  char msg[256];
  std::snprintf(
      msg, sizeof(msg),
      "TIM ACKM %s: GFV=0x%02X NACK_SA=0x%02X PGN=0x%06X",
      control_text, group_function, nack_address, acked_pgn);
  logWarn(msg);

  const bool is_tim_c2s_pgn = (acked_pgn == PGN_TIM_V1_C2S || acked_pgn == PGN_TIM_V2_C2S);
  if (!is_tim_c2s_pgn) return;

  if (group_function == MSG_CLIENT_STATUS) {
    // Keep sending TIM_ClientStatus at 100 ms; do not force full handshake restart on F9 NACK.
    return;
  }

  if (group_function == MSG_CONNECTION_VERSION && pending_response_ == PendingResponse::Version) {
    comm_error_ = true;
    return;
  }
  if (group_function == MSG_FUNCTIONS_SUPPORT_REQ && pending_response_ == PendingResponse::Support) {
    comm_error_ = true;
    return;
  }
  if (group_function == MSG_FUNCTIONS_ASSIGN_REQ && pending_response_ == PendingResponse::Assignment) {
    comm_error_ = true;
    return;
  }
  if (group_function == MSG_FUNCTIONS_ASSIGN_STATUS_REQ &&
      pending_response_ == PendingResponse::AssignmentStatus)
  {
    comm_error_ = true;
    return;
  }
}

// Send TIM_FunctionsSupportRequest in fixed 8-byte form (8.3.1).
void TimClient::send_functions_support_req()
{
  const std::uint32_t now = nowMs();
  if (now - last_support_req_ms_ < timing_.support_req_period_ms) return;
  if (support_req_attempts_ >= MAX_REQ_ATTEMPTS) {
    logError("TIM handshake failed: functions support response timeout/retries exceeded");
    comm_error_ = true;
    state_ = State::Fault;
    diag_ = DiagState::Error;
    return;
  }
  last_support_req_ms_ = now;
  support_req_attempts_ = static_cast<std::uint8_t>(support_req_attempts_ + 1U);
  pending_response_ = PendingResponse::Support;

  std::vector<std::uint8_t> req(8U, 0xFFU);
  req[0] = MSG_FUNCTIONS_SUPPORT_REQ;

  // TIM_FunctionsSupportRequest uses TIM C2S operation PGN 0x2400.
  send_var(PGN_TIM_V1_C2S, 6, sa_server_, req);
}

// Parse TIM_FunctionsSupportResponse with function/facility tuples (8.3.2).
void TimClient::handle_functions_support_resp_payload(const std::vector<std::uint8_t> & payload)
{
  if (payload.size() < 2) {
    log_parse_warn_once(PARSE_WARN_SUPPORT_SHORT, "TIM parse warning: FunctionsSupportResponse too short");
    return;
  }
  if (payload[0] != MSG_FUNCTIONS_SUPPORT_RESP) return;
  if (pending_response_ != PendingResponse::Support) return;
  aux_facility_len_.fill(0U);
  speed_facility_len_ = 0;
  curvature_facility_len_ = 0;
  rear_pto_facility_len_ = 0;
  rear_hitch_facility_len_ = 0;
  const std::size_t n = payload[1];
  auto parse_functions = [&](std::size_t start_idx, std::vector<std::uint8_t> & out) -> bool {
    std::size_t idx = start_idx;
    out.clear();
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      if ((idx + 1U) >= payload.size()) return false;
      const std::uint8_t fn = payload[idx++];
      const std::size_t facility_len = payload[idx++];
      if ((idx + facility_len) > payload.size()) return false;
      out.push_back(fn);
      for (std::size_t j = 0; j < cfg_.num_aux && j < MAX_AUX; ++j) {
        if (fn == cfg_.aux_fn_id[j]) {
          aux_facility_len_[j] = static_cast<std::uint8_t>(facility_len);
        }
      }
      if (fn == cfg_.speed_fn_id) speed_facility_len_ = static_cast<std::uint8_t>(facility_len);
      if (fn == cfg_.curvature_fn_id) curvature_facility_len_ = static_cast<std::uint8_t>(facility_len);
      if (fn == cfg_.rear_pto_fn_id) rear_pto_facility_len_ = static_cast<std::uint8_t>(facility_len);
      if (fn == cfg_.rear_hitch_fn_id) rear_hitch_facility_len_ = static_cast<std::uint8_t>(facility_len);
      idx += facility_len; // Facilities are currently not consumed by control logic.
    }
    return true;
  };

  std::vector<std::uint8_t> parsed_fns;
  if (!parse_functions(2U, parsed_fns)) {
    // Some implementations include one reserved byte before the first function tuple.
    if (payload.size() < 3U || !parse_functions(3U, parsed_fns)) {
      log_parse_warn_once(
          PARSE_WARN_SUPPORT_TUPLES,
          "TIM parse warning: FunctionsSupportResponse tuples malformed/truncated");
      return;
    }
  }

  std::sort(parsed_fns.begin(), parsed_fns.end());
  parsed_fns.erase(std::unique(parsed_fns.begin(), parsed_fns.end()), parsed_fns.end());
  supported_fns_ = parsed_fns;

  auto has = [&](std::uint8_t fn) {
    return std::find(supported_fns_.begin(), supported_fns_.end(), fn) != supported_fns_.end();
  };

  bool ok = true;
  auto log_missing_fn = [this](const char * name, std::uint8_t fn) {
    char msg[160];
    std::snprintf(msg, sizeof(msg), "TIM support mismatch: %s function 0x%02X not reported by server", name, fn);
    logWarn(msg);
  };
  auto log_bad_facility = [this](const char * name, std::uint8_t fn) {
    char msg[160];
    std::snprintf(msg, sizeof(msg), "TIM support mismatch: %s function 0x%02X has zero facility length", name, fn);
    logWarn(msg);
  };
  for (std::size_t i = 0; i < cfg_.num_aux && i < MAX_AUX; ++i) {
    if (!has(cfg_.aux_fn_id[i])) {
      ok = false;
      log_missing_fn("AUX", cfg_.aux_fn_id[i]);
    }
    if (cfg_.aux_ctrl_mode[i] != CM_AUX_PERCENT_FLOW) {
      ok = false;
      char msg[160];
      std::snprintf(
          msg, sizeof(msg),
          "TIM support mismatch: AUX function 0x%02X control mode %u unsupported",
          cfg_.aux_fn_id[i], static_cast<unsigned>(cfg_.aux_ctrl_mode[i]));
      logWarn(msg);
    }
    if (aux_facility_len_[i] == 0U) {
      ok = false;
      log_bad_facility("AUX", cfg_.aux_fn_id[i]);
    }
  }
  if (cfg_.enable_speed) {
    if (!has(cfg_.speed_fn_id)) {
      ok = false;
      log_missing_fn("speed", cfg_.speed_fn_id);
    }
    if (cfg_.speed_ctrl_mode != CM_SPEED_LINEAR) {
      ok = false;
      logWarn("TIM support mismatch: speed control mode unsupported");
    }
    if (speed_facility_len_ == 0U) {
      ok = false;
      log_bad_facility("speed", cfg_.speed_fn_id);
    }
  }
  if (cfg_.enable_curvature) {
    if (!has(cfg_.curvature_fn_id)) {
      ok = false;
      log_missing_fn("curvature", cfg_.curvature_fn_id);
    }
    if (cfg_.curvature_ctrl_mode != CM_GUIDE_CURVATURE) {
      ok = false;
      logWarn("TIM support mismatch: curvature control mode unsupported");
    }
    if (curvature_facility_len_ == 0U) {
      ok = false;
      log_bad_facility("curvature", cfg_.curvature_fn_id);
    }
  }
  if (cfg_.enable_rear_pto) {
    if (!has(cfg_.rear_pto_fn_id)) {
      ok = false;
      log_missing_fn("rear_pto", cfg_.rear_pto_fn_id);
    }
    if (cfg_.rear_pto_ctrl_mode != CM_PTO_SPEED) {
      ok = false;
      logWarn("TIM support mismatch: rear_pto control mode unsupported");
    }
    if (rear_pto_facility_len_ == 0U) {
      ok = false;
      log_bad_facility("rear_pto", cfg_.rear_pto_fn_id);
    }
  }
  if (cfg_.enable_rear_hitch) {
    if (!has(cfg_.rear_hitch_fn_id)) {
      ok = false;
      log_missing_fn("rear_hitch", cfg_.rear_hitch_fn_id);
    }
    if (cfg_.rear_hitch_ctrl_mode != CM_HITCH_PERCENT) {
      ok = false;
      logWarn("TIM support mismatch: rear_hitch control mode unsupported");
    }
    if (rear_hitch_facility_len_ == 0U) {
      ok = false;
      log_bad_facility("rear_hitch", cfg_.rear_hitch_fn_id);
    }
  }

  support_ok_ = ok;
  support_done_ = true;
  pending_response_ = PendingResponse::None;
  support_req_attempts_ = 0;

  char msg[192];
  std::snprintf(
      msg, sizeof(msg),
      "TIM handshake: FunctionsSupport received (supported=%u requested_cfg=%u result=%s)",
      static_cast<unsigned>(supported_fns_.size()),
      static_cast<unsigned>(cfg_.num_aux + (cfg_.enable_speed ? 1U : 0U) +
                            (cfg_.enable_curvature ? 1U : 0U) +
                            (cfg_.enable_rear_pto ? 1U : 0U) +
                            (cfg_.enable_rear_hitch ? 1U : 0U)),
      support_ok_ ? "OK" : "NOT_OK");
  logInfo(msg);
}

// Send TIM_FunctionsAssignmentRequest using function ID + request information tuples (8.4.1).
void TimClient::send_functions_assignment_req()
{
  const std::uint32_t now = nowMs();
  // AEF023 8.4.2: wait up to 1500 ms for FunctionsAssignmentResponse before retrying.
  if (pending_response_ == PendingResponse::Assignment &&
      (now - last_assign_req_ms_) < ASSIGNMENT_RESPONSE_TIMEOUT_MS)
  {
    return;
  }
  if (pending_response_ == PendingResponse::None &&
      (now - last_assign_req_ms_) < timing_.assign_req_period_ms)
  {
    return;
  }
  if (assign_req_attempts_ >= MAX_REQ_ATTEMPTS) {
    logError("TIM handshake failed: functions assignment response timeout/retries exceeded");
    comm_error_ = true;
    state_ = State::Fault;
    diag_ = DiagState::Error;
    return;
  }
  last_assign_req_ms_ = now;
  assign_req_attempts_ = static_cast<std::uint8_t>(assign_req_attempts_ + 1U);
  pending_response_ = PendingResponse::Assignment;

  requested_fns_.clear();
  auto has = [&](std::uint8_t fn) {
    return std::find(supported_fns_.begin(), supported_fns_.end(), fn) != supported_fns_.end();
  };

  for (std::size_t i = 0; i < cfg_.num_aux && i < MAX_AUX; ++i) if (has(cfg_.aux_fn_id[i])) requested_fns_.push_back(cfg_.aux_fn_id[i]);
  if (cfg_.enable_speed && has(cfg_.speed_fn_id)) requested_fns_.push_back(cfg_.speed_fn_id);
  if (cfg_.enable_curvature && has(cfg_.curvature_fn_id)) requested_fns_.push_back(cfg_.curvature_fn_id);
  if (cfg_.enable_rear_pto && has(cfg_.rear_pto_fn_id)) requested_fns_.push_back(cfg_.rear_pto_fn_id);
  if (cfg_.enable_rear_hitch && has(cfg_.rear_hitch_fn_id)) requested_fns_.push_back(cfg_.rear_hitch_fn_id);

  std::sort(requested_fns_.begin(), requested_fns_.end());
  requested_fns_.erase(std::unique(requested_fns_.begin(), requested_fns_.end()), requested_fns_.end());

  // 8.4.1 Byte 4 format:
  //   bits 8..6 request type, bits 5..1 reserved.
  // TIM server compatibility: "assign to requesting TIM client" = 0b001 (0x3F with reserved bits=1).
  static constexpr std::uint8_t ASSIGN_REQ_TYPE_ASSIGN = 0x1;
  static constexpr std::uint8_t ASSIGN_REQ_INFO_RESERVED = 0x1F;
  const std::uint8_t assign_req_info =
      static_cast<std::uint8_t>((ASSIGN_REQ_TYPE_ASSIGN << 5) | ASSIGN_REQ_INFO_RESERVED);

  std::vector<std::uint8_t> pl;
  pl.reserve(2 + (2 * requested_fns_.size()));
  pl.push_back(MSG_FUNCTIONS_ASSIGN_REQ);
  pl.push_back(static_cast<std::uint8_t>(requested_fns_.size()));
  for (const auto fn : requested_fns_) {
    pl.push_back(fn);
    pl.push_back(assign_req_info);
  }

  // TIM_FunctionsAssignmentRequest uses TIM C2S operation PGN 0x2400.
  send_var(PGN_TIM_V1_C2S, 6, sa_server_, pl);
}

// Parse TIM_FunctionsAssignmentResponse tuples and require assigned status (8.4.2).
void TimClient::handle_functions_assignment_resp_payload(const std::vector<std::uint8_t> & payload)
{
  if (payload.size() < 2) {
    log_parse_warn_once(PARSE_WARN_ASSIGN_SHORT, "TIM parse warning: FunctionsAssignmentResponse too short");
    return;
  }
  if (payload[0] != MSG_FUNCTIONS_ASSIGN_RESP) return;
  if (pending_response_ != PendingResponse::Assignment) return;
  const std::size_t n = payload[1];
  if (payload.size() < 2 + (2 * n)) {
    log_parse_warn_once(
        PARSE_WARN_ASSIGN_SHORT,
        "TIM parse warning: FunctionsAssignmentResponse length does not match tuple count");
    return;
  }

  // Assignment Result byte format in 8.4.2: status in bits 8..6, reason in bits 5..1.
  static constexpr std::uint8_t ASSIGN_STATUS_NOT_ASSIGNED = 0x0;
  static constexpr std::uint8_t ASSIGN_STATUS_ASSIGNED = 0x1;
  static constexpr std::uint8_t ASSIGN_STATUS_ASSIGNMENT_NOT_SUCCESSFUL = 0x5;
  static constexpr std::uint8_t ASSIGN_STATUS_ERROR = 0x6;
  static constexpr std::uint8_t ASSIGN_STATUS_NOT_AVAILABLE = 0x7;
  static constexpr std::uint8_t ASSIGN_REASON_ALL_CLEAR = 0x00;

  bool ok = true;
  std::size_t rejected_count = 0U;
  std::uint8_t first_reject_fn = 0xFFU;
  std::uint8_t first_reject_status = ASSIGN_STATUS_NOT_AVAILABLE;
  std::uint8_t first_reject_reason = 0x1FU;
  for (auto fn : requested_fns_) {
    bool found = false;
    std::uint8_t status = ASSIGN_STATUS_NOT_AVAILABLE;
    std::uint8_t reason = 0x1F;
    for (std::size_t i = 0; i < n; ++i) {
      const std::uint8_t r_fn = payload[2 + (2 * i)];
      const std::uint8_t assignment_result = payload[2 + (2 * i) + 1];
      const std::uint8_t r_status = static_cast<std::uint8_t>((assignment_result >> 5) & 0x07);
      const std::uint8_t r_reason = static_cast<std::uint8_t>(assignment_result & 0x1F);
      if (r_fn == fn) {
        found = true;
        status = r_status;
        reason = r_reason;
        break;
      }
    }
    if (!found) {
      ok = false;
      ++rejected_count;
      if (first_reject_fn == 0xFFU) {
        first_reject_fn = fn;
      }
      continue;
    }
    if (status == ASSIGN_STATUS_ASSIGNMENT_NOT_SUCCESSFUL ||
        status == ASSIGN_STATUS_ERROR ||
        status == ASSIGN_STATUS_NOT_AVAILABLE) {
      ok = false;
      ++rejected_count;
      if (first_reject_fn == 0xFFU) {
        first_reject_fn = fn;
        first_reject_status = status;
        first_reject_reason = reason;
      }
    }
    if (status != ASSIGN_STATUS_ASSIGNED || reason != ASSIGN_REASON_ALL_CLEAR) {
      ok = false;
      ++rejected_count;
      if (first_reject_fn == 0xFFU) {
        first_reject_fn = fn;
        first_reject_status = status;
        first_reject_reason = reason;
      }
    }
  }

  assign_ok_ = ok;
  assign_done_ = true;
  pending_response_ = PendingResponse::None;
  assign_req_attempts_ = 0;

  char msg[192];
  std::snprintf(
      msg, sizeof(msg),
      "TIM handshake: FunctionsAssignment response (requested=%u result=%s)",
      static_cast<unsigned>(requested_fns_.size()),
      assign_ok_ ? "OK" : "NOT_OK");
  logInfo(msg);
  if (!assign_ok_) {
    char rej[224];
    std::snprintf(
      rej, sizeof(rej),
      "TIM assignment detail: rejected=%u first_fn=0x%02X status=%u reason=%u",
      static_cast<unsigned>(rejected_count),
      static_cast<unsigned>(first_reject_fn),
      static_cast<unsigned>(first_reject_status),
      static_cast<unsigned>(first_reject_reason));
    logWarn(rej);
  }
}

void TimClient::send_functions_assignment_status_req()
{
  const std::uint32_t now = nowMs();
  if (now - last_assign_status_req_ms_ < timing_.assign_req_period_ms) return;
  if (assign_status_req_attempts_ >= MAX_REQ_ATTEMPTS) {
    logError("TIM handshake failed: assignment status response timeout/retries exceeded");
    comm_error_ = true;
    state_ = State::Fault;
    diag_ = DiagState::Error;
    return;
  }
  last_assign_status_req_ms_ = now;
  assign_status_req_attempts_ = static_cast<std::uint8_t>(assign_status_req_attempts_ + 1U);
  pending_response_ = PendingResponse::AssignmentStatus;

  std::vector<std::uint8_t> req(8U, 0xFFU);
  req[0] = MSG_FUNCTIONS_ASSIGN_STATUS_REQ;
  // TIM_FunctionsAssignmentStatusRequest uses TIM C2S operation PGN 0x2400.
  send_var(PGN_TIM_V1_C2S, 6, sa_server_, req);
}

void TimClient::handle_functions_assignment_status_resp_payload(const std::vector<std::uint8_t> & payload)
{
  if (payload.size() < 3U) {
    log_parse_warn_once(
        PARSE_WARN_ASSIGN_STATUS_SHORT, "TIM parse warning: FunctionsAssignmentStatusResponse too short");
    return;
  }
  if (payload[0] != MSG_FUNCTIONS_ASSIGN_STATUS_RESP) return;
  if (pending_response_ != PendingResponse::AssignmentStatus) return;
  std::size_t idx = 1;

  const std::size_t n_unassigned = payload[idx++];
  if ((idx + n_unassigned) > payload.size()) {
    log_parse_warn_once(
        PARSE_WARN_ASSIGN_STATUS_SHORT,
        "TIM parse warning: FunctionsAssignmentStatusResponse unassigned list truncated");
    return;
  }
  std::vector<std::uint8_t> unassigned(payload.begin() + idx, payload.begin() + idx + n_unassigned);
  idx += n_unassigned;
  if (idx >= payload.size()) {
    log_parse_warn_once(
        PARSE_WARN_ASSIGN_STATUS_SHORT,
        "TIM parse warning: FunctionsAssignmentStatusResponse missing controlled-function list count");
    return;
  }

  const std::size_t n_cfs = payload[idx++];
  std::vector<std::uint8_t> assigned_any;
  std::vector<std::uint8_t> assigned_to_self;
  for (std::size_t cf = 0; cf < n_cfs; ++cf) {
    if ((idx + 8U) > payload.size()) {
      log_parse_warn_once(
          PARSE_WARN_ASSIGN_STATUS_SHORT,
          "TIM parse warning: FunctionsAssignmentStatusResponse truncated before CF NAME");
      return; // NAME
    }
    std::uint64_t cf_name_le = 0;
    for (std::size_t b = 0; b < 8U; ++b) {
      cf_name_le |= (static_cast<std::uint64_t>(payload[idx + b]) << (8U * b));
    }
    idx += 8U;
    if (idx >= payload.size()) {
      log_parse_warn_once(
          PARSE_WARN_ASSIGN_STATUS_SHORT,
          "TIM parse warning: FunctionsAssignmentStatusResponse missing CF function-count byte");
      return;
    }
    const std::size_t n_fn = payload[idx++];
    if ((idx + n_fn) > payload.size()) {
      log_parse_warn_once(
          PARSE_WARN_ASSIGN_STATUS_SHORT,
          "TIM parse warning: FunctionsAssignmentStatusResponse CF function list truncated");
      return;
    }
    assigned_any.insert(assigned_any.end(), payload.begin() + idx, payload.begin() + idx + n_fn);
    if (client_name_known_ && cf_name_le == client_name_) {
      assigned_to_self.insert(assigned_to_self.end(), payload.begin() + idx, payload.begin() + idx + n_fn);
    }
    idx += n_fn;
  }

  bool ok = true;
  for (const auto fn : requested_fns_) {
    if (std::find(unassigned.begin(), unassigned.end(), fn) != unassigned.end()) {
      ok = false;
      continue;
    }
    const auto & owned = client_name_known_ ? assigned_to_self : assigned_any;
    if (std::find(owned.begin(), owned.end(), fn) == owned.end()) {
      ok = false;
    }
  }

  assign_status_ok_ = ok;
  assign_status_done_ = true;
  pending_response_ = PendingResponse::None;
  assign_status_req_attempts_ = 0;

  char msg[224];
  std::snprintf(
      msg, sizeof(msg),
      "TIM handshake: FunctionsAssignmentStatus received (unassigned=%u cfs=%u result=%s%s)",
      static_cast<unsigned>(n_unassigned),
      static_cast<unsigned>(n_cfs),
      assign_status_ok_ ? "OK" : "NOT_OK",
      client_name_known_ ? " own-name-check=enabled" : " own-name-check=disabled");
  logInfo(msg);
}

// Send periodic TIM function requests for enabled targets (Annex D / 10.x).
void TimClient::send_function_requests()
{
  const std::uint32_t now = nowMs();
  // Periodic path is active only in Periodic/Both command modes.
  // In Direct mode, setters trigger immediate single-function transmission instead.
  if (!use_periodic_commands()) return;
  if (now - last_func_tx_ms_ < timing_.func_tx_period_ms) return;

  last_func_tx_ms_ = now;
  bool sent_any = false;

  const auto should_send_for = [](FunctionControlState s) {
    return s != FunctionControlState::Disabled;
  };

  if (negotiated_version_ <= 1) {
    for (std::size_t i = 0; i < cfg_.num_aux && i < MAX_AUX; ++i) {
      if (should_send_for(aux_control_state_[i]) && should_send_function(cfg_.aux_fn_id[i])) {
        send_aux_req_v1(i, mode_from_control_state(aux_control_state_[i]));
        sent_any = true;
      }
    }
    if (cfg_.enable_speed && should_send_for(speed_control_state_) && should_send_function(cfg_.speed_fn_id)) {
      send_speed_req_v1(mode_from_control_state(speed_control_state_));
      sent_any = true;
    }
    if (cfg_.enable_curvature && should_send_for(curvature_control_state_) &&
        should_send_function(cfg_.curvature_fn_id)) {
      send_curv_req_v1(mode_from_control_state(curvature_control_state_));
      sent_any = true;
    }
    if (cfg_.enable_rear_pto && should_send_for(rear_pto_control_state_) &&
        should_send_function(cfg_.rear_pto_fn_id)) {
      send_pto_req_v1(mode_from_control_state(rear_pto_control_state_));
      sent_any = true;
    }
    if (cfg_.enable_rear_hitch && should_send_for(rear_hitch_control_state_) &&
        should_send_function(cfg_.rear_hitch_fn_id)) {
      send_hitch_req_v1(mode_from_control_state(rear_hitch_control_state_));
      sent_any = true;
    }
  } else {
    for (std::size_t i = 0; i < cfg_.num_aux && i < MAX_AUX; ++i) {
      if (should_send_for(aux_control_state_[i]) && should_send_function(cfg_.aux_fn_id[i])) {
        send_aux_req_v2(i, mode_from_control_state(aux_control_state_[i]));
        sent_any = true;
      }
    }
    if (cfg_.enable_speed && should_send_for(speed_control_state_) && should_send_function(cfg_.speed_fn_id)) {
      send_speed_req_v2(mode_from_control_state(speed_control_state_));
      sent_any = true;
    }
    if (cfg_.enable_curvature && should_send_for(curvature_control_state_) &&
        should_send_function(cfg_.curvature_fn_id)) {
      send_curv_req_v2(mode_from_control_state(curvature_control_state_));
      sent_any = true;
    }
    if (cfg_.enable_rear_pto && should_send_for(rear_pto_control_state_) &&
        should_send_function(cfg_.rear_pto_fn_id)) {
      send_pto_req_v2(mode_from_control_state(rear_pto_control_state_));
      sent_any = true;
    }
    if (cfg_.enable_rear_hitch && should_send_for(rear_hitch_control_state_) &&
        should_send_function(cfg_.rear_hitch_fn_id)) {
      send_hitch_req_v2(mode_from_control_state(rear_hitch_control_state_));
      sent_any = true;
    }
  }

  if (!sent_any) return;
  update_release_tracking_after_function_tx(now);
}

// Send TIM v1 AUX request; low nibble carries 4-bit request counter (6.3.3.2, 7.2.5).
void TimClient::send_aux_req_v1(std::size_t idx, RequestMode mode)
{
  const auto req_counter = next_v1_request_counter();
  const auto fn = cfg_.aux_fn_id[idx];
  float request_flow = targets_.aux_flow_pct[idx];
  if (use_handshake_setpoint_reference() && actuals_.aux_valid[idx]) {
    // During handshake, hold current server value instead of applying external setpoints.
    request_flow = actuals_.aux_flow_pct[idx];
  }
  std::uint16_t raw = enc_u16(request_flow, 0.004f, -128.512f);
  if (mode == RequestMode::ReadyToControl) raw = TIM_REQ_READY_TO_CONTROL;
  else if (mode == RequestMode::ReleaseOperatorAction) raw = TIM_REQ_RELEASE_OPERATOR_ACTION;
  else if (mode == RequestMode::ReleaseAcceptIncrease) raw = TIM_REQ_RELEASE_ACCEPT_INCREASE;

  std::array<std::uint8_t,8> d{};
  d.fill(0xFF);
  d[0] = fn;
  d[1] = static_cast<std::uint8_t>((V1_FUNCTION_REQUEST << 4) | req_counter);
  put_u16_le(d, 4, raw);
  expected_v1_reflected_counter_ = req_counter;
  expected_v1_reflected_since_ms_ = nowMs();
  awaiting_v1_reflection_ = true;

  sendFrame(make_frame(PGN_TIM_V1_C2S, 3, sa_client_, sa_server_, d));
}

// Send TIM v2 AUX request; low nibble carries control mode instead of request counter (7.2.6).
void TimClient::send_aux_req_v2(std::size_t idx, RequestMode mode)
{
  const auto fn = cfg_.aux_fn_id[idx];
  const auto cm = cfg_.aux_ctrl_mode[idx] ? cfg_.aux_ctrl_mode[idx] : CM_AUX_PERCENT_FLOW;
  float request_flow = targets_.aux_flow_pct[idx];
  if (use_handshake_setpoint_reference() && actuals_.aux_valid[idx]) {
    // During handshake, hold current server value instead of applying external setpoints.
    request_flow = actuals_.aux_flow_pct[idx];
  }
  std::uint16_t raw = enc_u16(request_flow, 0.004f, -128.512f);
  if (mode == RequestMode::ReadyToControl) raw = TIM_REQ_READY_TO_CONTROL;
  else if (mode == RequestMode::ReleaseOperatorAction) raw = TIM_REQ_RELEASE_OPERATOR_ACTION;
  else if (mode == RequestMode::ReleaseAcceptIncrease) raw = TIM_REQ_RELEASE_ACCEPT_INCREASE;

  std::array<std::uint8_t,8> d{};
  d.fill(0xFF);
  d[0] = fn;
  d[1] = static_cast<std::uint8_t>((V2_FUNCTION_REQUEST << 4) | (cm & 0x0F));
  put_u16_le(d, 4, raw);

  sendFrame(make_frame(PGN_TIM_V2_C2S, 3, sa_client_, sa_server_, d));
}

// Send TIM v1 vehicle speed request.
void TimClient::send_speed_req_v1(RequestMode mode)
{
  const auto req_counter = next_v1_request_counter();
  std::uint16_t raw = enc_u16(targets_.speed_mps, 0.001f, -32.128f);
  if (mode == RequestMode::ReadyToControl) raw = TIM_REQ_READY_TO_CONTROL;
  else if (mode == RequestMode::ReleaseOperatorAction) raw = TIM_REQ_RELEASE_OPERATOR_ACTION;
  else if (mode == RequestMode::ReleaseAcceptIncrease) raw = TIM_REQ_RELEASE_ACCEPT_INCREASE;

  std::array<std::uint8_t,8> d{};
  d.fill(0xFF);
  d[0] = cfg_.speed_fn_id;
  d[1] = static_cast<std::uint8_t>((V1_FUNCTION_REQUEST << 4) | req_counter);
  put_u16_le(d, 4, raw);
  expected_v1_reflected_counter_ = req_counter;
  expected_v1_reflected_since_ms_ = nowMs();
  awaiting_v1_reflection_ = true;

  sendFrame(make_frame(PGN_TIM_V1_C2S, 3, sa_client_, sa_server_, d));
}

// Send TIM v2 vehicle speed request.
void TimClient::send_speed_req_v2(RequestMode mode)
{
  std::uint16_t raw = enc_u16(targets_.speed_mps, 0.001f, -32.128f);
  if (mode == RequestMode::ReadyToControl) raw = TIM_REQ_READY_TO_CONTROL;
  else if (mode == RequestMode::ReleaseOperatorAction) raw = TIM_REQ_RELEASE_OPERATOR_ACTION;
  else if (mode == RequestMode::ReleaseAcceptIncrease) raw = TIM_REQ_RELEASE_ACCEPT_INCREASE;

  std::array<std::uint8_t,8> d{};
  d.fill(0xFF);
  d[0] = cfg_.speed_fn_id;
  d[1] = static_cast<std::uint8_t>((V2_FUNCTION_REQUEST << 4) | (cfg_.speed_ctrl_mode & 0x0F));
  put_u16_le(d, 4, raw);

  sendFrame(make_frame(PGN_TIM_V2_C2S, 3, sa_client_, sa_server_, d));
}

// Send TIM v1 guidance curvature request.
void TimClient::send_curv_req_v1(RequestMode mode)
{
  const auto req_counter = next_v1_request_counter();
  std::uint16_t raw = enc_u16(targets_.curvature_km_inv, 0.25f, -8032.0f);
  if (mode == RequestMode::ReadyToControl) raw = TIM_REQ_READY_TO_CONTROL;
  else if (mode == RequestMode::ReleaseOperatorAction) raw = TIM_REQ_RELEASE_OPERATOR_ACTION;
  else if (mode == RequestMode::ReleaseAcceptIncrease) raw = TIM_REQ_RELEASE_ACCEPT_INCREASE;

  std::array<std::uint8_t,8> d{};
  d.fill(0xFF);
  d[0] = cfg_.curvature_fn_id;
  d[1] = static_cast<std::uint8_t>((V1_FUNCTION_REQUEST << 4) | req_counter);
  put_u16_le(d, 4, raw);
  expected_v1_reflected_counter_ = req_counter;
  expected_v1_reflected_since_ms_ = nowMs();
  awaiting_v1_reflection_ = true;

  sendFrame(make_frame(PGN_TIM_V1_C2S, 3, sa_client_, sa_server_, d));
}

// Send TIM v2 guidance curvature request.
void TimClient::send_curv_req_v2(RequestMode mode)
{
  std::uint16_t raw = enc_u16(targets_.curvature_km_inv, 0.25f, -8032.0f);
  if (mode == RequestMode::ReadyToControl) raw = TIM_REQ_READY_TO_CONTROL;
  else if (mode == RequestMode::ReleaseOperatorAction) raw = TIM_REQ_RELEASE_OPERATOR_ACTION;
  else if (mode == RequestMode::ReleaseAcceptIncrease) raw = TIM_REQ_RELEASE_ACCEPT_INCREASE;

  std::array<std::uint8_t,8> d{};
  d.fill(0xFF);
  d[0] = cfg_.curvature_fn_id;
  d[1] = static_cast<std::uint8_t>((V2_FUNCTION_REQUEST << 4) | (cfg_.curvature_ctrl_mode & 0x0F));
  put_u16_le(d, 4, raw);

  sendFrame(make_frame(PGN_TIM_V2_C2S, 3, sa_client_, sa_server_, d));
}

// Send TIM v1 rear PTO request.
void TimClient::send_pto_req_v1(RequestMode mode)
{
  const auto req_counter = next_v1_request_counter();
  std::uint16_t raw = enc_u16(targets_.rear_pto_rpm, 0.125f, -4016.0f);
  if (mode == RequestMode::ReadyToControl) raw = TIM_REQ_READY_TO_CONTROL;
  else if (mode == RequestMode::ReleaseOperatorAction) raw = TIM_REQ_RELEASE_OPERATOR_ACTION;
  else if (mode == RequestMode::ReleaseAcceptIncrease) raw = TIM_REQ_RELEASE_ACCEPT_INCREASE;

  std::array<std::uint8_t,8> d{};
  d.fill(0xFF);
  d[0] = cfg_.rear_pto_fn_id;
  d[1] = static_cast<std::uint8_t>((V1_FUNCTION_REQUEST << 4) | req_counter);
  put_u16_le(d, 4, raw);
  expected_v1_reflected_counter_ = req_counter;
  expected_v1_reflected_since_ms_ = nowMs();
  awaiting_v1_reflection_ = true;

  sendFrame(make_frame(PGN_TIM_V1_C2S, 3, sa_client_, sa_server_, d));
}

// Send TIM v2 rear PTO request.
void TimClient::send_pto_req_v2(RequestMode mode)
{
  std::uint16_t raw = enc_u16(targets_.rear_pto_rpm, 0.125f, -4016.0f);
  if (mode == RequestMode::ReadyToControl) raw = TIM_REQ_READY_TO_CONTROL;
  else if (mode == RequestMode::ReleaseOperatorAction) raw = TIM_REQ_RELEASE_OPERATOR_ACTION;
  else if (mode == RequestMode::ReleaseAcceptIncrease) raw = TIM_REQ_RELEASE_ACCEPT_INCREASE;

  std::array<std::uint8_t,8> d{};
  d.fill(0xFF);
  d[0] = cfg_.rear_pto_fn_id;
  d[1] = static_cast<std::uint8_t>((V2_FUNCTION_REQUEST << 4) | (cfg_.rear_pto_ctrl_mode & 0x0F));
  put_u16_le(d, 4, raw);

  sendFrame(make_frame(PGN_TIM_V2_C2S, 3, sa_client_, sa_server_, d));
}

// Send TIM v1 rear hitch request.
void TimClient::send_hitch_req_v1(RequestMode mode)
{
  const auto req_counter = next_v1_request_counter();
  std::uint16_t raw = enc_u16(targets_.rear_hitch_pct, 0.01f, 0.0f);
  if (mode == RequestMode::ReadyToControl) raw = TIM_REQ_READY_TO_CONTROL;
  else if (mode == RequestMode::ReleaseOperatorAction) raw = TIM_REQ_RELEASE_OPERATOR_ACTION;
  else if (mode == RequestMode::ReleaseAcceptIncrease) raw = TIM_REQ_RELEASE_ACCEPT_INCREASE;

  std::array<std::uint8_t,8> d{};
  d.fill(0xFF);
  d[0] = cfg_.rear_hitch_fn_id;
  d[1] = static_cast<std::uint8_t>((V1_FUNCTION_REQUEST << 4) | req_counter);
  put_u16_le(d, 4, raw);
  expected_v1_reflected_counter_ = req_counter;
  expected_v1_reflected_since_ms_ = nowMs();
  awaiting_v1_reflection_ = true;

  sendFrame(make_frame(PGN_TIM_V1_C2S, 3, sa_client_, sa_server_, d));
}

// Send TIM v2 rear hitch request.
void TimClient::send_hitch_req_v2(RequestMode mode)
{
  std::uint16_t raw = enc_u16(targets_.rear_hitch_pct, 0.01f, 0.0f);
  if (mode == RequestMode::ReadyToControl) raw = TIM_REQ_READY_TO_CONTROL;
  else if (mode == RequestMode::ReleaseOperatorAction) raw = TIM_REQ_RELEASE_OPERATOR_ACTION;
  else if (mode == RequestMode::ReleaseAcceptIncrease) raw = TIM_REQ_RELEASE_ACCEPT_INCREASE;

  std::array<std::uint8_t,8> d{};
  d.fill(0xFF);
  d[0] = cfg_.rear_hitch_fn_id;
  d[1] = static_cast<std::uint8_t>((V2_FUNCTION_REQUEST << 4) | (cfg_.rear_hitch_ctrl_mode & 0x0F));
  put_u16_le(d, 4, raw);

  sendFrame(make_frame(PGN_TIM_V2_C2S, 3, sa_client_, sa_server_, d));
}

// Decode TIM v1 function status payload into Actuals cache.
void TimClient::handle_v1_function_status(const ros2_isobus::msg::IsobusFrame & fr)
{
  const auto fn = fr.data[0];
  const auto msg_type = (fr.data[1] >> 4) & 0x0F;
  if (msg_type != V1_FUNCTION_STATUS) return;
  const std::uint8_t reflected_counter = fr.data[1] & 0x0F;
  const std::uint8_t automation_status = fr.data[2] & 0x0F;

  const auto raw = u16_le(fr.data, 4);

  // AUX function status values.
  for (std::size_t i = 0; i < cfg_.num_aux && i < MAX_AUX; ++i) {
    if (fn == cfg_.aux_fn_id[i]) {
      actuals_.aux_flow_pct[i] = dec_u16(raw, 0.004f, -128.512f);
      actuals_.aux_valid[i] = true;
      actuals_.aux_automation_status[i] = automation_status;
      observe_v1_status(reflected_counter, automation_status);
      publish_function_status(fn);
      return;
    }
  }

  if (cfg_.enable_speed && fn == cfg_.speed_fn_id) {
    actuals_.speed_mps = dec_u16(raw, 0.001f, -32.128f);
    actuals_.speed_valid = true;
    actuals_.speed_automation_status = automation_status;
    observe_v1_status(reflected_counter, automation_status);
    publish_function_status(fn);
    return;
  }

  if (cfg_.enable_curvature && fn == cfg_.curvature_fn_id) {
    actuals_.curvature_km_inv = dec_u16(raw, 0.25f, -8032.0f);
    actuals_.curvature_valid = true;
    actuals_.curvature_automation_status = automation_status;
    observe_v1_status(reflected_counter, automation_status);
    publish_function_status(fn);
    return;
  }

  if (cfg_.enable_rear_pto && fn == cfg_.rear_pto_fn_id) {
    actuals_.rear_pto_rpm = dec_u16(raw, 0.125f, -4016.0f);
    actuals_.rear_pto_valid = true;
    actuals_.rear_pto_automation_status = automation_status;
    observe_v1_status(reflected_counter, automation_status);
    publish_function_status(fn);
    return;
  }

  if (cfg_.enable_rear_hitch && fn == cfg_.rear_hitch_fn_id) {
    actuals_.rear_hitch_pct = dec_u16(raw, 0.01f, 0.0f);
    actuals_.rear_hitch_valid = true;
    actuals_.rear_hitch_automation_status = automation_status;
    observe_v1_status(reflected_counter, automation_status);
    publish_function_status(fn);
    return;
  }
}

// Decode TIM v2 function status payload using explicit guideline message structures.
void TimClient::handle_v2_function_status(const ros2_isobus::msg::IsobusFrame & fr)
{
  const std::uint8_t fn = fr.data[0];
  const std::uint8_t msg_type = (fr.data[1] >> 4) & 0x0F;
  const std::uint8_t control_mode = fr.data[1] & 0x0F;

  // Guideline-based dispatch: parse only known status-family structures.
  for (std::size_t i = 0; i < cfg_.num_aux && i < MAX_AUX; ++i) {
    if (fn != cfg_.aux_fn_id[i]) continue;
    const std::uint8_t expected_cm = cfg_.aux_ctrl_mode[i] ? cfg_.aux_ctrl_mode[i] : CM_AUX_PERCENT_FLOW;
    if (control_mode != expected_cm) return;
    if (msg_type == V2_MSG_AUTOMATION_STATUS) {
      const std::uint8_t automation_status = fr.data[2] & 0x0F;
      actuals_.aux_automation_status[i] = automation_status;
      observe_v2_status(automation_status);
      publish_function_status(fn);
      return;
    }
    if (msg_type == V2_MSG_PROCESS_DATA) {
      actuals_.aux_flow_pct[i] = dec_u16(u16_le(fr.data, 4), 0.004f, -128.512f);
      actuals_.aux_valid[i] = true;
      publish_function_status(fn);
      return;
    }
    return;
  }

  if (cfg_.enable_rear_pto && fn == cfg_.rear_pto_fn_id) {
    if (control_mode != cfg_.rear_pto_ctrl_mode) return;
    if (msg_type == V2_MSG_AUTOMATION_STATUS) {
      const std::uint8_t automation_status = fr.data[2] & 0x0F;
      actuals_.rear_pto_automation_status = automation_status;
      observe_v2_status(automation_status);
      publish_function_status(fn);
    } else if (msg_type == V2_MSG_PROCESS_DATA) {
      actuals_.rear_pto_rpm = dec_u16(u16_le(fr.data, 4), 0.125f, -4016.0f);
      actuals_.rear_pto_valid = true;
      publish_function_status(fn);
    }
    return;
  }

  if (cfg_.enable_rear_hitch && fn == cfg_.rear_hitch_fn_id) {
    if (control_mode != cfg_.rear_hitch_ctrl_mode) return;
    if (msg_type == V2_MSG_AUTOMATION_STATUS) {
      const std::uint8_t automation_status = fr.data[2] & 0x0F;
      actuals_.rear_hitch_automation_status = automation_status;
      observe_v2_status(automation_status);
      publish_function_status(fn);
    } else if (msg_type == V2_MSG_PROCESS_DATA) {
      actuals_.rear_hitch_pct = dec_u16(u16_le(fr.data, 4), 0.01f, 0.0f);
      actuals_.rear_hitch_valid = true;
      publish_function_status(fn);
    }
    return;
  }

  if (cfg_.enable_speed && fn == cfg_.speed_fn_id) {
    if (control_mode != cfg_.speed_ctrl_mode) return;
    if (msg_type == V2_MSG_AUTOMATION_STATUS) {
      const std::uint8_t automation_status = fr.data[2] & 0x0F;
      actuals_.speed_automation_status = automation_status;
      observe_v2_status(automation_status);
      publish_function_status(fn);
    } else if (msg_type == V2_MSG_PROCESS_DATA) {
      actuals_.speed_mps = dec_u16(u16_le(fr.data, 4), 0.001f, -32.128f);
      actuals_.speed_valid = true;
      publish_function_status(fn);
    }
    return;
  }

  if (cfg_.enable_curvature && fn == cfg_.curvature_fn_id) {
    if (control_mode != cfg_.curvature_ctrl_mode) return;
    if (msg_type == V2_MSG_AUTOMATION_STATUS) {
      const std::uint8_t automation_status = fr.data[2] & 0x0F;
      actuals_.curvature_automation_status = automation_status;
      observe_v2_status(automation_status);
      publish_function_status(fn);
    } else if (msg_type == V2_MSG_PROCESS_DATA) {
      actuals_.curvature_km_inv = dec_u16(u16_le(fr.data, 4), 0.25f, -8032.0f);
      actuals_.curvature_valid = true;
      publish_function_status(fn);
    }
    return;
  }
}

// Entry point for incoming single-frame TIM messages.
void TimClient::on_frame(const ros2_isobus::msg::IsobusFrame & fr)
{
  if (auth_provider_) {
    auth_provider_->on_frame(fr);
  }
  const std::uint8_t pf = fr.pf ? fr.pf : static_cast<std::uint8_t>((fr.pgn >> 8) & 0xFFU);
  if ((static_cast<std::uint32_t>(pf) << 8U) == PGN_ACKM) {
    handle_acknowledgement(fr);
    return;
  }

  const bool is_v1_s2c = (fr.pgn == PGN_TIM_V1_S2C) ||
                         (pf == static_cast<std::uint8_t>((PGN_TIM_V1_S2C >> 8) & 0xFFU));
  const bool is_v2_s2c = (fr.pgn == PGN_TIM_V2_S2C) ||
                         (pf == static_cast<std::uint8_t>((PGN_TIM_V2_S2C >> 8) & 0xFFU));
  // Some TIM simulator setups publish v2 function status on the opposite operation PGN.
  const bool is_v2_s2c_alt = (fr.pgn == PGN_TIM_V2_C2S) ||
                             (pf == static_cast<std::uint8_t>((PGN_TIM_V2_C2S >> 8) & 0xFFU));
  if (!is_v1_s2c && !is_v2_s2c && !is_v2_s2c_alt) return;
  if (fr.sa != sa_server_) return;

  // Server status drives communication watchdog.
  if (fr.data[0] == MSG_SERVER_STATUS) {
    handle_server_status(fr);
    return;
  }

  // Version response is accepted only while negotiating.
  if (fr.data[0] == MSG_CONNECTION_VERSION &&
      state_ == State::NegotiateVersion &&
      pending_response_ == PendingResponse::Version)
  {
    handle_connection_version_resp(fr);
    return;
  }

  // Support response only in support phase with matching pending request.
  if (fr.data[0] == MSG_FUNCTIONS_SUPPORT_RESP &&
      state_ == State::Support &&
      pending_response_ == PendingResponse::Support)
  {
    std::vector<std::uint8_t> pl(fr.data.begin(), fr.data.end());
    handle_functions_support_resp_payload(pl);
    return;
  }

  // Assignment response only in assignment phase with matching pending request.
  if (fr.data[0] == MSG_FUNCTIONS_ASSIGN_RESP &&
      state_ == State::Assignment &&
      pending_response_ == PendingResponse::Assignment)
  {
    std::vector<std::uint8_t> pl(fr.data.begin(), fr.data.end());
    handle_functions_assignment_resp_payload(pl);
    return;
  }
  if (fr.data[0] == MSG_FUNCTIONS_ASSIGN_STATUS_RESP &&
      state_ == State::Assignment &&
      pending_response_ == PendingResponse::AssignmentStatus)
  {
    std::vector<std::uint8_t> pl(fr.data.begin(), fr.data.end());
    handle_functions_assignment_status_resp_payload(pl);
    return;
  }

  const bool accept_function_status =
      (state_ == State::Operational);
  if (!accept_function_status) return;

  // Strict version split: parse only negotiated TIM generation status frames.
  const bool expect_v1 = (negotiated_version_ <= 1);
  if (expect_v1 && is_v1_s2c) {
    handle_v1_function_status(fr);
  } else if (!expect_v1 && (is_v2_s2c || is_v2_s2c_alt)) {
    handle_v2_function_status(fr);
  }
}

// Entry point for reassembled TP payloads carrying TIM variable-length responses.
void TimClient::on_tp_payload(std::uint32_t pgn, const std::vector<std::uint8_t> & payload)
{
  if (auth_provider_) {
    auth_provider_->on_tp_payload(pgn, payload);
  }
  if (payload.empty()) return;
  const std::uint8_t pf = static_cast<std::uint8_t>((pgn >> 8) & 0xFFU);
  const bool is_v1_s2c = (pgn == PGN_TIM_V1_S2C) ||
                         (pf == static_cast<std::uint8_t>((PGN_TIM_V1_S2C >> 8) & 0xFFU));
  const bool is_v2_s2c = (pgn == PGN_TIM_V2_S2C) ||
                         (pf == static_cast<std::uint8_t>((PGN_TIM_V2_S2C >> 8) & 0xFFU));
  const bool is_v2_s2c_alt = (pgn == PGN_TIM_V2_C2S) ||
                             (pf == static_cast<std::uint8_t>((PGN_TIM_V2_C2S >> 8) & 0xFFU));
  if (!is_v1_s2c && !is_v2_s2c && !is_v2_s2c_alt) return;

  if (payload[0] == MSG_FUNCTIONS_SUPPORT_RESP &&
      state_ == State::Support &&
      pending_response_ == PendingResponse::Support)
  {
    handle_functions_support_resp_payload(payload);
  } else if (payload[0] == MSG_FUNCTIONS_ASSIGN_RESP &&
             state_ == State::Assignment &&
             pending_response_ == PendingResponse::Assignment)
  {
    handle_functions_assignment_resp_payload(payload);
  } else if (payload[0] == MSG_FUNCTIONS_ASSIGN_STATUS_RESP &&
             state_ == State::Assignment &&
             pending_response_ == PendingResponse::AssignmentStatus)
  {
    handle_functions_assignment_status_resp_payload(payload);
  }
}

// TIM client state machine (AEF 023 RIG 4, 6.2.2/6.4/6.5 with operator gating to active control).
void TimClient::process()
{
  const auto set_state = [this](State new_state) {
    if (state_ == new_state) return;
    
    std::string to_state;
    switch (new_state) {
      case State::Discover: to_state = "Discover"; break;
      case State::NegotiateVersion: to_state = "NegotiateVersion"; break;
      case State::Support: to_state = "Support"; break;
      case State::Auth: to_state = "Auth"; break;
      case State::Assignment: to_state = "Assignment"; break;
      case State::Operational: to_state = "Operational"; break;
      case State::Fault: to_state = "Fault"; break;
      default: to_state = "Unknown"; break;
    }

    logInfo("TIM state transition to: " + to_state);
    state_ = new_state;
  };
  const auto set_diag_state = [this](DiagState new_diag) {
    if (diag_ == new_diag) return;

    std::string to_diag;
    switch (new_diag) {
      case DiagState::Unavailable: to_diag = "Unavailable"; break;
      case DiagState::NotReady: to_diag = "NotReady"; break;
      case DiagState::Ready: to_diag = "Ready"; break;
      case DiagState::Enabled: to_diag = "Enabled"; break;
      case DiagState::Pending: to_diag = "Pending"; break;
      case DiagState::Active: to_diag = "Active"; break;
      case DiagState::ActiveLimitedHigh: to_diag = "ActiveLimitedHigh"; break;
      case DiagState::ActiveLimitedLow: to_diag = "ActiveLimitedLow"; break;
      case DiagState::Error: to_diag = "Error"; break;
      default: to_diag = "Unknown"; break;
    }

    logInfo("TIM diagnostic state transition to: " + to_diag);
    diag_ = new_diag;
  };

  const std::uint32_t now = nowMs();
  latest_automation_status_ = compute_effective_automation_status();
  const bool has_valid_client_sa = (sa_client_ <= 0xFDU);
  const bool has_valid_server_sa = (sa_server_ <= 0xFDU);
  if (!has_valid_client_sa || !has_valid_server_sa) {
    set_diag_state(DiagState::Unavailable);
    set_state(State::Discover);
    return;
  }

  if (graceful_shutdown_requested_) {
    if (!graceful_shutdown_sent_) {
      send_client_status_with_heartbeat(0xFFU);
      graceful_shutdown_sent_ = true;
    }
    return;
  }
  if (nack_cooldown_active_) {
    if (now < nack_cooldown_until_ms_) {
      return;
    }
    nack_cooldown_active_ = false;
    comm_error_ = false;
    server_shutdown_seen_ = false;
    have_server_heartbeat_ = false;
    reset_session_state();
  }

  if (conn_req_sent_once_) {
    send_client_status();
  }

  const bool server_alive = !server_shutdown_seen_ &&
                            ((now - last_server_status_rx_ms_) <= timing_.server_status_timeout_ms);
  if (comm_error_) {
    logWarn("TIM communication error detected, entering cooldown and restarting handshake");
    start_nack_cooldown();
    reset_session_state();
    return;
  }
  if (awaiting_v1_reflection_ &&
      ((now - expected_v1_reflected_since_ms_) > timing_.reflect_timeout_ms))
  {
    logError("TIM handshake failed: v1 reflected request counter timeout");
    comm_error_ = true;
    set_state(State::Fault);
  }
  // Do not enforce status timeout while waiting operator acknowledgement/pending transition.
  const bool expects_function_status =
      (state_ == State::Operational) && has_controlled_functions();
  if (expects_function_status && last_function_req_tx_ms_ != 0U) {
    const std::uint32_t ref =
        (last_function_status_rx_ms_ != 0U) ? last_function_status_rx_ms_ : last_function_req_tx_ms_;
    if ((now - ref) > timing_.function_status_timeout_ms) {
      logError("TIM handshake/control failed: function status timeout");
      comm_error_ = true;
      set_state(State::Fault);
    }
  }
  const std::uint32_t effective_release_timeout_ms =
      std::max<std::uint32_t>(timing_.release_status_timeout_ms, timing_.func_tx_period_ms * 5U);
  if (expects_function_status && awaiting_release_status_ && release_request_since_ms_ != 0U &&
      ((now - release_request_since_ms_) > effective_release_timeout_ms))
  {
    logError("TIM handshake/control failed: release status timeout");
    comm_error_ = true;
    set_state(State::Fault);
  }

  switch (state_) {
    case State::Discover:
      set_diag_state(DiagState::NotReady);
      set_state(State::NegotiateVersion);
      break;

    case State::NegotiateVersion:
      send_connection_version_req();
      if (negotiated_version_ == 1 || negotiated_version_ == 2) {
        set_state(State::Support);
        support_done_ = false;
        assign_done_ = false;
        support_ok_ = false;
        assign_ok_ = false;
        set_diag_state(DiagState::NotReady);
        support_req_attempts_ = 0;
        assign_req_attempts_ = 0;
        assign_status_req_attempts_ = 0;
        pending_response_ = PendingResponse::None;
      }
      break;

    case State::Support:
      if (!server_alive) {
        // Wait for periodic TIM_ServerStatus before support negotiation continues.
        set_diag_state(DiagState::NotReady);
        break;
      }
      if (!support_done_) {
        send_functions_support_req();
        break;
      }
      if (!support_ok_) {
        logError("TIM handshake failed: functions support response incompatible with requested functions");
        set_state(State::Fault);
        set_diag_state(DiagState::Error);
        break;
      }
      set_state(State::Auth);
      set_diag_state(DiagState::NotReady);
      break;

    case State::Auth:
      if (auth_provider_) 
        auth_provider_->process(now);

      if (is_server_authorized()) {
        set_state(State::Assignment);
        assign_req_attempts_ = 0;
        assign_status_req_attempts_ = 0;
        assign_done_ = false;
        assign_status_done_ = false;
        assign_status_ok_ = false;
        pending_response_ = PendingResponse::None;
        break;
      }

      if (auth_provider_->is_failed()) {
        logWarn("TIM handshake blocked: server is not authenticated and lead-authorized");
        set_diag_state(DiagState::Unavailable);
        set_state(State::Discover);
        break;
      }

      set_diag_state(DiagState::NotReady);
      break;

    case State::Assignment:
      if (!is_server_authorized()) {
        logError("TIM handshake/control failed: authorization lost before assignment");
        set_state(State::Fault);
        set_diag_state(DiagState::Error);
        break;
      }
      if (!assign_done_) {
        send_functions_assignment_req();
        break;
      }
      if (!assign_status_done_) {
        send_functions_assignment_status_req();
        break;
      }
      if (!assign_ok_) {
        logWarn(
          "TIM assignment interop: FunctionsAssignment response reported NOT_OK, "
          "continuing based on FunctionsAssignmentStatus ownership check");
      }
      if (!assign_status_ok_) {
        logError("TIM handshake failed: functions assignment status not owned by this client");
        set_state(State::Fault);
        set_diag_state(DiagState::Error);
        break;
      }
      set_state(State::Operational);
      set_diag_state(DiagState::Enabled);
      pending_response_ = PendingResponse::None;
      break;

    case State::Operational:
      if (!is_server_authorized()) {
        logError("TIM control failed: authorization lost in operational state");
        set_state(State::Fault);
        set_diag_state(DiagState::Error);
        break;
      }
      update_function_control_states();
      set_diag_state(diag_from_automation_status(latest_automation_status_));
      send_function_requests();
      break;

    case State::Fault:
    default:
      set_diag_state(DiagState::Error);
      break;
  }
}

}  // namespace ros2_isobus
