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

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ros2_isobus/msg/isobus_frame.hpp"
#include "ros2_isobus/msg/isobus_tp_frame.hpp"

namespace ros2_isobus
{

// TIM client/server PGNs for Gen1/Gen2 message pairs (AEF 023 RIG 4, 7.2).
static constexpr std::uint32_t PGN_TIM_V1_C2S =  9216;   // destination specific
static constexpr std::uint32_t PGN_TIM_V1_S2C =  8960;   // broadcast
static constexpr std::uint32_t PGN_TIM_V2_C2S = 15616;   // destination specific
static constexpr std::uint32_t PGN_TIM_V2_S2C = 15360;   // broadcast

// TIM initialization/diagnostic message codes (AEF 023 RIG 4, 7.2 and 8.x).
static constexpr std::uint8_t MSG_CONNECTION_VERSION = 0xF6;
static constexpr std::uint8_t MSG_CLIENT_STATUS      = 0xF9;
static constexpr std::uint8_t MSG_SERVER_STATUS      = 0xFA; // server->client heartbeat (code)
static constexpr std::uint8_t MSG_FUNCTIONS_ASSIGN_STATUS_REQ  = 0xF3;
static constexpr std::uint8_t MSG_FUNCTIONS_ASSIGN_STATUS_RESP = 0xF3;
static constexpr std::uint8_t MSG_FUNCTIONS_SUPPORT_REQ  = 0xF4;
static constexpr std::uint8_t MSG_FUNCTIONS_SUPPORT_RESP = 0xF4;
static constexpr std::uint8_t MSG_FUNCTIONS_ASSIGN_REQ   = 0xF5;
static constexpr std::uint8_t MSG_FUNCTIONS_ASSIGN_RESP  = 0xF5;
static constexpr std::uint8_t MSG_AUTH_SERVER_STATUS     = 0xF9;
static constexpr std::uint8_t MSG_AUTH_CLIENT_STATUS     = 0xFA;

// AEF 040 authentication PGNs.
static constexpr std::uint32_t PGN_AUTH_S2C = 28672;  // 0x7000 AUTH12
static constexpr std::uint32_t PGN_AUTH_C2S = 28416;  // 0x6F00 AUTH21

// TIM v1 function message type nibble (AEF 023 RIG 4, 7.2.5).
static constexpr std::uint8_t V1_FUNCTION_REQUEST = 0x0;
static constexpr std::uint8_t V1_FUNCTION_STATUS  = 0x1;

// TIM v2 byte2 bits 8..5 message-type nibble (AEF 023 RIG 4, 10.3.2.x).
static constexpr std::uint8_t V2_MSG_AUTOMATION_STATUS = 0x0;
static constexpr std::uint8_t V2_MSG_PROCESS_DATA      = 0x1;
// Backward-compatible aliases used by request path.
static constexpr std::uint8_t V2_FUNCTION_REQUEST = V2_MSG_PROCESS_DATA;
static constexpr std::uint8_t V2_FUNCTION_STATUS  = V2_MSG_PROCESS_DATA;

// Common special 2-byte request values (AEF 023 RIG 4, 10.2.1).
static constexpr std::uint16_t TIM_REQ_RELEASE_OPERATOR_ACTION = 0xFBFD;
static constexpr std::uint16_t TIM_REQ_RELEASE_ACCEPT_INCREASE = 0xFBFE;
static constexpr std::uint16_t TIM_REQ_READY_TO_CONTROL        = 0xFBFF;

// TIM function automation status values (AEF 023 RIG 4, 10.2.2).
static constexpr std::uint8_t TIM_AUTO_UNAVAILABLE       = 0x0;
static constexpr std::uint8_t TIM_AUTO_NOT_READY         = 0x1;
static constexpr std::uint8_t TIM_AUTO_READY_TO_ENABLE   = 0x2;
static constexpr std::uint8_t TIM_AUTO_ENABLED           = 0x3;
static constexpr std::uint8_t TIM_AUTO_PENDING           = 0x4;
static constexpr std::uint8_t TIM_AUTO_ACTIVE            = 0x5;
static constexpr std::uint8_t TIM_AUTO_ACTIVE_LIMIT_HIGH = 0x6;
static constexpr std::uint8_t TIM_AUTO_ACTIVE_LIMIT_LOW  = 0x7;
static constexpr std::uint8_t TIM_AUTO_FAULT             = 0xD;

// Common TIM function IDs used by this client configuration.
static constexpr std::uint8_t FN_REAR_PTO        = 0x41;
static constexpr std::uint8_t FN_REAR_HITCH      = 0x43;
static constexpr std::uint8_t FN_VEHICLE_SPEED   = 0x44;
static constexpr std::uint8_t FN_EXT_GUIDANCE    = 0x46;

// Aux valve N function IDs are 0x01..0x20 for N=1..32; this client uses 0x01..0x08.

// TIM v2 control modes in byte2 low nibble (AEF 023 RIG 4, 10.x control messages).
static constexpr std::uint8_t CM_AUX_PERCENT_FLOW = 0x1;
static constexpr std::uint8_t CM_PTO_SPEED        = 0x1;
static constexpr std::uint8_t CM_HITCH_PERCENT    = 0x1;
static constexpr std::uint8_t CM_SPEED_LINEAR     = 0x1;
static constexpr std::uint8_t CM_GUIDE_CURVATURE  = 0x1;

static constexpr std::size_t MAX_AUX = 8;

// Function set and control-mode mapping requested from TIM server.
struct FunctionConfig
{
  std::size_t num_aux = 0;
  std::array<std::uint8_t, MAX_AUX> aux_fn_id{};          // e.g. 0x21,0x22...
  std::array<std::uint8_t, MAX_AUX> aux_ctrl_mode{};      // v2 low nibble

  bool enable_speed = true;
  std::uint8_t speed_fn_id = FN_VEHICLE_SPEED;
  std::uint8_t speed_ctrl_mode = CM_SPEED_LINEAR;

  bool enable_curvature = true;
  std::uint8_t curvature_fn_id = FN_EXT_GUIDANCE;
  std::uint8_t curvature_ctrl_mode = CM_GUIDE_CURVATURE;

  bool enable_rear_pto = true;
  std::uint8_t rear_pto_fn_id = FN_REAR_PTO;
  std::uint8_t rear_pto_ctrl_mode = CM_PTO_SPEED;

  bool enable_rear_hitch = true;
  std::uint8_t rear_hitch_fn_id = FN_REAR_HITCH;
  std::uint8_t rear_hitch_ctrl_mode = CM_HITCH_PERCENT;
};

// Timing configuration for retries, status traffic and state timeouts.
struct TimingConfig
{
  std::uint32_t client_status_period_ms = 100;
  std::uint32_t func_tx_period_ms = 100;
  std::uint32_t conn_req_period_ms = 1000;
  std::uint32_t support_req_period_ms = 1000;
  std::uint32_t assign_req_period_ms = 1000;
  std::uint32_t server_status_timeout_ms = 2000;
  std::uint32_t function_status_timeout_ms = 6000;
  std::uint32_t release_status_timeout_ms = 1500;
  std::uint32_t reflect_timeout_ms = 500;
  // 0 disables this client-local timeout; guideline leaves this proprietary.
  std::uint32_t pending_to_active_timeout_ms = 0;
};

enum class State : std::uint8_t
{
  // TIM client workflow aligned to AEF 023 initialization/assignment/state model (6.2.2, 6.4, 6.5).
  Discover = 0,
  NegotiateVersion,
  Support,
  Auth,
  Assignment,
  Operational,
  Fault
};

enum class DiagState : std::uint8_t
{
  // Automation state values carried in TIM_ClientStatus (6.2.1, 9.3).
  Unavailable = 0,
  NotReady    = 1,
  Ready       = 2,
  Enabled     = 3,
  Pending     = 4,
  Active      = 5,
  ActiveLimitedHigh = 6,
  ActiveLimitedLow  = 7,
  Error       = 0x0D
};

struct Targets
{
  std::array<float, MAX_AUX> aux_flow_pct{};
  std::array<bool,  MAX_AUX> aux_enable{};

  // Vehicle speed request [m/s], encoded with SLOT scale/offset from TIM control messages (10.x).
  float speed_mps = 0.0f;
  bool  speed_enable = false;

  // Guidance curvature request [km^-1], encoded with TIM SLOT scale/offset (10.x).
  float curvature_km_inv = 0.0f;
  bool  curvature_enable = false;

  // Rear PTO request [rpm], encoded with TIM SLOT scale/offset (10.x).
  float rear_pto_rpm = 0.0f;
  bool  rear_pto_enable = false;

  // Rear hitch request [%], encoded with TIM SLOT scale/offset (10.x).
  float rear_hitch_pct = 0.0f;
  bool  rear_hitch_enable = false;
};

struct Actuals
{
  std::array<float, MAX_AUX> aux_flow_pct{};
  std::array<bool,  MAX_AUX> aux_valid{};
  std::array<std::uint8_t, MAX_AUX> aux_automation_status{};

  float speed_mps = 0.0f;
  bool  speed_valid = false;
  std::uint8_t speed_automation_status = TIM_AUTO_UNAVAILABLE;

  float curvature_km_inv = 0.0f;
  bool  curvature_valid = false;
  std::uint8_t curvature_automation_status = TIM_AUTO_UNAVAILABLE;

  float rear_pto_rpm = 0.0f;
  bool  rear_pto_valid = false;
  std::uint8_t rear_pto_automation_status = TIM_AUTO_UNAVAILABLE;

  float rear_hitch_pct = 0.0f;
  bool  rear_hitch_valid = false;
  std::uint8_t rear_hitch_automation_status = TIM_AUTO_UNAVAILABLE;
};

/*
 *
 * TimClient implements TIM client-side workflow and control messaging:
 *  - Version negotiation, function support discovery and assignment
 *  - TIM client automation state machine and heartbeat/status handling
 *  - TIM v1/v2 function request/status encoding/decoding over ISOBUS/TP
 *
 */
class TimClient
{
public:
  enum class CommandMode : std::uint8_t
  {
    Direct = 0,
    Periodic,
    Both
  };

  struct IAuthProvider
  {
    virtual ~IAuthProvider() = default;
    explicit IAuthProvider(TimClient & owner) : owner_(owner) {}
    virtual void reset(std::uint8_t client_sa, std::uint8_t server_sa) {}
    virtual void on_frame(const ros2_isobus::msg::IsobusFrame & fr) {}
    virtual void on_tp_payload(std::uint32_t pgn, const std::vector<std::uint8_t> & payload) {}
    virtual void process(std::uint32_t now_ms) {}
    virtual bool is_authenticated() const = 0;
    virtual bool is_lead_server() const = 0;
    virtual bool is_failed() const { return false; }

  protected:
    void sendFrame(const ros2_isobus::msg::IsobusFrame & fr) { owner_.sendFrame(fr); }
    void sendTpFrame(const ros2_isobus::msg::IsobusTpFrame & tp) { owner_.sendTpFrame(tp); }
    void logInfo(const std::string & msg) { owner_.logInfo(msg); }
    void logWarn(const std::string & msg) { owner_.logWarn(msg); }
    void logError(const std::string & msg) { owner_.logError(msg); }

  private:
    TimClient & owner_;
  };

  TimClient(std::uint8_t sa_client,
            std::uint8_t sa_server,
            std::uint8_t implemented_version,
            std::uint8_t minimum_version,
            const FunctionConfig & cfg,
            const TimingConfig & timing = TimingConfig{});

  // Set requested values for active TIM functions.
  void set_aux_flow(std::size_t idx, float pct_flow, bool enable);
  void set_speed_mps(float mps, bool enable);
  void set_curvature_km_inv(float km_inv, bool enable);
  void set_rear_pto_rpm(float rpm, bool enable);
  void set_rear_hitch_pct(float pct, bool enable);

  // Operator gating input for transition to active automation.
  void set_operator_enable(bool enable);
  // Runtime timing override (used by ROS2 wrapper parameterization).
  void set_timing_config(const TimingConfig & timing);
  // Command TX policy for function requests: direct/periodic/both.
  void set_command_mode(CommandMode mode) { command_mode_ = mode; }
  // Optional CF NAME for assignment-status ownership checks.
  void set_client_name(std::uint64_t client_name);
  // Hook for authentication/lead-server policy provider.
  void set_auth_provider(std::shared_ptr<IAuthProvider> auth_provider);
  // Update client source address (SA), typically from AddressManager status.
  void set_client_sa(std::uint8_t sa_client);
  std::uint8_t client_sa() const { return sa_client_; }
  // Update TIM server destination/source address.
  void set_server_sa(std::uint8_t sa_server);
  std::uint8_t server_sa() const { return sa_server_; }
  // Trigger graceful shutdown sequence for TIM communication.
  void request_graceful_shutdown();

  // Run periodic TIM state machine, status traffic and function requests.
  void process();

  // Receive single-frame TIM traffic.
  void on_frame(const ros2_isobus::msg::IsobusFrame & fr);

  // Receive reassembled TP payload for var-length TIM messages.
  void on_tp_payload(std::uint32_t pgn, const std::vector<std::uint8_t> & payload);

  State state() const { return state_; }
  DiagState diag() const { return diag_; }
  std::uint8_t negotiated_version() const { return negotiated_version_; }
  std::uint8_t aux_automation_status(std::size_t idx) const
  {
    return (idx < MAX_AUX) ? actuals_.aux_automation_status[idx] : TIM_AUTO_UNAVAILABLE;
  }
  std::uint8_t speed_automation_status() const { return actuals_.speed_automation_status; }
  std::uint8_t curvature_automation_status() const { return actuals_.curvature_automation_status; }
  std::uint8_t rear_pto_automation_status() const { return actuals_.rear_pto_automation_status; }
  std::uint8_t rear_hitch_automation_status() const { return actuals_.rear_hitch_automation_status; }
  const Actuals & actuals() const { return actuals_; }
  const FunctionConfig & function_config() const { return cfg_; }

protected:
  // Platform hooks implemented by ROS2 wrapper.
  virtual std::uint32_t nowMs() const = 0;
  virtual void sendFrame(const ros2_isobus::msg::IsobusFrame & fr) = 0;
  virtual void sendTpFrame(const ros2_isobus::msg::IsobusTpFrame & tp) {}
  virtual void logInfo(const std::string & msg) {}
  virtual void logWarn(const std::string & msg) {}
  virtual void logError(const std::string & msg) {}
  virtual void publish_function_status(std::uint8_t fn) {}

private:
  enum class RequestMode : std::uint8_t;
  enum class FunctionControlState : std::uint8_t;

  // Build one IsobusFrame with correct PDU1/PDU2 addressing rules.
  ros2_isobus::msg::IsobusFrame make_frame(std::uint32_t pgn, std::uint8_t priority,
                                          std::uint8_t sa, std::uint8_t da,
                                          const std::array<std::uint8_t,8> & data) const;

  // Send payload as single frame or TP depending on length (ISO 11783-3 transport usage).
  void send_var(std::uint32_t pgn, std::uint8_t priority, std::uint8_t da,
                const std::vector<std::uint8_t> & payload);

  // Initialization/diagnostic message workflow (AEF 023 RIG 4, 6.3/6.4/8.x/9.x).
  void send_client_status();
  void send_connection_version_req();
  void handle_connection_version_resp(const ros2_isobus::msg::IsobusFrame & fr);
  void handle_server_status(const ros2_isobus::msg::IsobusFrame & fr);
  void handle_acknowledgement(const ros2_isobus::msg::IsobusFrame & fr);

  // Function support discovery and assignment handling (6.5, 8.3, 8.4).
  void send_functions_support_req();
  void handle_functions_support_resp_payload(const std::vector<std::uint8_t> & payload);

  void send_functions_assignment_req();
  void handle_functions_assignment_resp_payload(const std::vector<std::uint8_t> & payload);
  void send_functions_assignment_status_req();
  void handle_functions_assignment_status_resp_payload(const std::vector<std::uint8_t> & payload);

  // TIM function request transmission (Annex D, 10.x).
  void send_function_requests();
  void send_aux_request_immediate(std::size_t idx);
  void send_speed_request_immediate();
  void send_curvature_request_immediate();
  void send_rear_pto_request_immediate();
  void send_rear_hitch_request_immediate();
  RequestMode mode_from_control_state(FunctionControlState s) const;
  void update_release_tracking_after_function_tx(std::uint32_t now_ms);

  void send_aux_req_v1(std::size_t idx, RequestMode mode);
  void send_aux_req_v2(std::size_t idx, RequestMode mode);

  void send_speed_req_v1(RequestMode mode);
  void send_speed_req_v2(RequestMode mode);

  void send_curv_req_v1(RequestMode mode);
  void send_curv_req_v2(RequestMode mode);

  void send_pto_req_v1(RequestMode mode);
  void send_pto_req_v2(RequestMode mode);

  void send_hitch_req_v1(RequestMode mode);
  void send_hitch_req_v2(RequestMode mode);

  // TIM function status decoding (v1/v2).
  void handle_v1_function_status(const ros2_isobus::msg::IsobusFrame & fr);
  void handle_v2_function_status(const ros2_isobus::msg::IsobusFrame & fr);

  // Status/handshake helpers.
  void observe_v1_status(std::uint8_t reflected_counter, std::uint8_t automation_status);
  void observe_v2_status(std::uint8_t automation_status);
  void validate_server_heartbeat(std::uint8_t hb);
  std::uint8_t next_heartbeat_counter();
  std::uint8_t next_v1_request_counter();
  bool is_automation_active(std::uint8_t s) const;
  bool is_automation_pending(std::uint8_t s) const;
  bool is_function_enabled(std::uint8_t fn) const;
  std::uint8_t compute_effective_automation_status() const;
  void update_function_control_states();
  bool has_controlled_functions() const;
  DiagState diag_from_automation_status(std::uint8_t s) const;
  bool should_send_function(std::uint8_t fn) const;
  bool is_special_mode() const;
  bool use_handshake_setpoint_reference() const;
  bool is_server_authorized() const;
  bool use_direct_commands() const
  {
    return (command_mode_ == CommandMode::Direct) || (command_mode_ == CommandMode::Both);
  }
  bool use_periodic_commands() const
  {
    return (command_mode_ == CommandMode::Periodic) || (command_mode_ == CommandMode::Both);
  }
  void log_parse_warn_once(std::uint32_t bit, const std::string & msg);
  void reset_session_state();
  void start_nack_cooldown();
  void send_client_status_with_heartbeat(std::uint8_t hb);

  // SLOT value encoding/decoding helpers for TIM request/status payloads.
  static std::uint16_t enc_u16(float value, float resolution, float offset);
  static float         dec_u16(std::uint16_t raw, float resolution, float offset);

  // Byte-level helpers.
  static std::uint16_t u16_le(const std::array<std::uint8_t,8> & d, std::size_t idx);
  static void          put_u16_le(std::array<std::uint8_t,8> & d, std::size_t idx, std::uint16_t v);

private:
  std::uint8_t sa_client_;
  std::uint8_t sa_server_;

  std::uint8_t implemented_version_;
  std::uint8_t minimum_version_;
  std::uint8_t negotiated_version_ = 0;

  FunctionConfig cfg_;

  State state_ = State::Discover;
  DiagState diag_ = DiagState::Unavailable;

  bool operator_enable_ = false;
  std::shared_ptr<IAuthProvider> auth_provider_;
  std::uint32_t parse_warn_mask_ = 0;
  bool graceful_shutdown_requested_ = false;
  bool graceful_shutdown_sent_ = false;
  bool nack_cooldown_active_ = false;
  std::uint32_t nack_cooldown_until_ms_ = 0;

  // Assignment workflow bookkeeping (6.5).
  bool support_done_ = false;
  bool support_ok_ = false;
  bool assign_done_ = false;
  bool assign_ok_ = false;
  bool assign_status_done_ = false;
  bool assign_status_ok_ = false;
  std::vector<std::uint8_t> supported_fns_;
  std::vector<std::uint8_t> requested_fns_;
  bool client_name_known_ = false;
  std::uint64_t client_name_ = 0;
  std::array<std::uint8_t, MAX_AUX> aux_facility_len_{};
  std::uint8_t speed_facility_len_ = 0;
  std::uint8_t curvature_facility_len_ = 0;
  std::uint8_t rear_pto_facility_len_ = 0;
  std::uint8_t rear_hitch_facility_len_ = 0;

  // Heartbeat/request counters and periodic timers (6.3.3, 10.1).
  std::uint8_t heartbeat_ = 0;
  bool heartbeat_initialized_ = false;
  std::uint8_t v1_req_counter_ = 0;
  bool have_server_heartbeat_ = false;
  std::uint8_t server_heartbeat_prev_ = 0;
  bool server_shutdown_seen_ = false;
  bool comm_error_ = false;
  std::uint8_t latest_automation_status_ = TIM_AUTO_UNAVAILABLE;
  std::uint32_t last_function_status_rx_ms_ = 0;
  std::uint32_t last_function_req_tx_ms_ = 0;
  std::uint32_t pending_since_ms_ = 0;
  bool release_mode_active_ = false;
  bool awaiting_release_status_ = false;
  std::uint32_t release_request_since_ms_ = 0;
  bool ready_to_control_sent_ = false;
  bool awaiting_v1_reflection_ = false;
  std::uint8_t expected_v1_reflected_counter_ = 0;
  std::uint32_t expected_v1_reflected_since_ms_ = 0;

  enum class RequestMode : std::uint8_t
  {
    Setpoint = 0,
    ReadyToControl,
    ReleaseOperatorAction,
    ReleaseAcceptIncrease
  };
  RequestMode request_mode_ = RequestMode::Setpoint;

  enum class FunctionControlState : std::uint8_t
  {
    Disabled = 0,
    Releasing,
    ReadyToEnable,
    Enabled,
    Pending,
    Active,
    Fault
  };
  std::array<FunctionControlState, MAX_AUX> aux_control_state_{};
  FunctionControlState speed_control_state_ = FunctionControlState::Disabled;
  FunctionControlState curvature_control_state_ = FunctionControlState::Disabled;
  FunctionControlState rear_pto_control_state_ = FunctionControlState::Disabled;
  FunctionControlState rear_hitch_control_state_ = FunctionControlState::Disabled;

  enum class PendingResponse : std::uint8_t
  {
    None = 0,
    Version,
    Support,
    Assignment,
    AssignmentStatus
  };
  PendingResponse pending_response_ = PendingResponse::None;
  std::uint8_t version_req_attempts_ = 0;
  std::uint8_t support_req_attempts_ = 0;
  std::uint8_t assign_req_attempts_ = 0;
  std::uint8_t assign_status_req_attempts_ = 0;

  TimingConfig timing_{};
  CommandMode command_mode_ = CommandMode::Periodic;
  std::uint32_t last_client_status_ms_ = 0;
  std::uint32_t last_server_status_rx_ms_ = 0;
  std::uint32_t last_conn_req_ms_ = 0;
  bool conn_req_sent_once_ = false;
  std::uint32_t last_support_req_ms_ = 0;
  std::uint32_t last_assign_req_ms_ = 0;
  std::uint32_t last_assign_status_req_ms_ = 0;
  std::uint32_t last_func_tx_ms_ = 0;

  Targets targets_{};
  Actuals actuals_{};
};

}  // namespace ros2_isobus
