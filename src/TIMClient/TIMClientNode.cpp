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

#include "TIMClientNode.hpp"
#include "AuthLibProvider.hpp"
#include "DummyAuthProvider.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <exception>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ros2_isobus
{
namespace
{

std::string to_hex(const std::array<std::uint8_t, 8> & arr)
{
    static constexpr char kHex[] = "0123456789ABCDEF";
    std::string out;
    out.reserve(arr.size() * 2);
    for (auto byte : arr) {
        out.push_back(kHex[(byte >> 4) & 0x0F]);
        out.push_back(kHex[byte & 0x0F]);
    }
    return out;
}

}  // namespace

// Initialize TimClient with ROS parameters and attach topic/timer interfaces.
// Wrapper keeps transport and orchestration in ROS domain while TimClient owns
// protocol sequencing from AEF 023 RIG 4 (init/auth/assignment/operational).
TIMClientROS2::TIMClientROS2(const rclcpp::NodeOptions & options)
: rclcpp::Node("tim_client", options),
  TimClient(static_cast<std::uint8_t>(this->declare_parameter<int>("sa_client", 0xFE)),
            static_cast<std::uint8_t>(this->declare_parameter<int>("sa_server", 0xFE)),
            static_cast<std::uint8_t>(this->declare_parameter<int>("implemented_version", 2)),
            static_cast<std::uint8_t>(this->declare_parameter<int>("minimum_version", 1)),
            load_function_config(*this))
{
    if (is_valid_node_sa(client_sa())) {
        client_sa_ready_ = true;
        RCLCPP_INFO(get_logger(), "Using TIM client SA from parameter: 0x%02X", client_sa());
    } else {
        RCLCPP_INFO(
            get_logger(),
            "TIM client SA not valid yet (sa_client=0x%02X), waiting AddressManager status",
            client_sa());
    }
    if (is_valid_node_sa(server_sa())) {
        server_sa_ready_ = true;
        RCLCPP_INFO(get_logger(), "Using TIM server SA from parameter: 0x%02X", server_sa());
    } else {
        RCLCPP_INFO(
            get_logger(),
            "TIM server SA not valid yet (sa_server=0x%02X), waiting server broadcast status",
            server_sa());
    }

    const std::string auth_mode_param = declare_parameter<std::string>("auth_mode", "None");
    std::string auth_mode = auth_mode_param;
    std::transform(auth_mode.begin(), auth_mode.end(), auth_mode.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    // command_mode:
    //  - direct: only immediate per-function transmission on set_* updates
    //  - periodic: cyclic full-function transmission (Annex D / 10.x loop semantics)
    //  - both: immediate updates + periodic refresh
    const std::string command_mode_param = declare_parameter<std::string>("command_mode", "direct");
    std::string command_mode = command_mode_param;
    std::transform(command_mode.begin(), command_mode.end(), command_mode.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (command_mode != "direct" && command_mode != "periodic" && command_mode != "both") {
        RCLCPP_WARN(
            get_logger(), "Unknown command_mode='%s', using direct", command_mode_param.c_str());
        command_mode = "direct";
    }
    set_command_mode(
        (command_mode == "direct") ? CommandMode::Direct :
        (command_mode == "both") ? CommandMode::Both :
        CommandMode::Periodic);
    const std::string client_name_hex =
        declare_parameter<std::string>("client_name_hex", "");
    if (!client_name_hex.empty()) {
        try {
            client_name_cached_ = std::stoull(client_name_hex, nullptr, 16);
            set_client_name(client_name_cached_);
            client_name_ready_ = true;
        } catch (const std::exception &) {
            RCLCPP_WARN(get_logger(), "Invalid client_name_hex parameter, ignoring");
        }
    }

    // Runtime timing profile aligns with AEF 023 client/server watchdog, retry and
    // state-transition timing windows (6.x/8.x/9.x workflow and 10.x control supervision).
    TimingConfig timing;
    timing.client_status_period_ms = static_cast<std::uint32_t>(
        declare_parameter<int>("client_status_period_ms", static_cast<int>(timing.client_status_period_ms)));
    timing.func_tx_period_ms = static_cast<std::uint32_t>(
        declare_parameter<int>("func_tx_period_ms", static_cast<int>(timing.func_tx_period_ms)));
    timing.conn_req_period_ms = static_cast<std::uint32_t>(
        declare_parameter<int>("conn_req_period_ms", static_cast<int>(timing.conn_req_period_ms)));
    timing.support_req_period_ms = static_cast<std::uint32_t>(
        declare_parameter<int>("support_req_period_ms", static_cast<int>(timing.support_req_period_ms)));
    timing.assign_req_period_ms = static_cast<std::uint32_t>(
        declare_parameter<int>("assign_req_period_ms", static_cast<int>(timing.assign_req_period_ms)));
    timing.server_status_timeout_ms = static_cast<std::uint32_t>(
        declare_parameter<int>("server_status_timeout_ms", static_cast<int>(timing.server_status_timeout_ms)));
    timing.function_status_timeout_ms = static_cast<std::uint32_t>(
        declare_parameter<int>("function_status_timeout_ms", static_cast<int>(timing.function_status_timeout_ms)));
    timing.release_status_timeout_ms = static_cast<std::uint32_t>(
        declare_parameter<int>("release_status_timeout_ms", static_cast<int>(timing.release_status_timeout_ms)));
    timing.reflect_timeout_ms = static_cast<std::uint32_t>(
        declare_parameter<int>("reflect_timeout_ms", static_cast<int>(timing.reflect_timeout_ms)));
    timing.pending_to_active_timeout_ms = static_cast<std::uint32_t>(
        declare_parameter<int>("pending_to_active_timeout_ms", static_cast<int>(timing.pending_to_active_timeout_ms)));
    set_timing_config(timing);

    const auto dummy_auth_period_ms = static_cast<std::uint32_t>(
        declare_parameter<int>("dummy_auth.period_ms", 1000));
    const auto dummy_auth_implemented_version = static_cast<std::uint8_t>(
        declare_parameter<int>("dummy_auth.implemented_version", 3));
    const auto dummy_auth_minimum_version = static_cast<std::uint8_t>(
        declare_parameter<int>("dummy_auth.minimum_version", 2));
    const auto authlib_period_ms = static_cast<std::uint32_t>(
        declare_parameter<int>("authlib.period_ms", 1000));
    const auto authlib_implemented_version = static_cast<std::uint8_t>(
        declare_parameter<int>("authlib.implemented_version", 3));
    const auto authlib_minimum_version = static_cast<std::uint8_t>(
        declare_parameter<int>("authlib.minimum_version", 2));
    const bool authlib_strict = declare_parameter<bool>("authlib.strict", false);
    // Maximum bytes sent in client certificate TP payload:
    //  - 0: send full DER certificate
    //  - N: truncate payload to N bytes (interop/testing)
    const int authlib_client_cert_payload_max_len_param =
        declare_parameter<int>("authlib.client_cert_payload_max_len", 26);
    const auto authlib_client_cert_payload_max_len = static_cast<std::uint16_t>(
        std::max(0, authlib_client_cert_payload_max_len_param));
    const int authlib_max_slice_iterations_param =
        declare_parameter<int>("authlib.max_slice_iterations", 1024);
    const auto authlib_max_slice_iterations = static_cast<std::uint16_t>(
        std::max<int>(1, authlib_max_slice_iterations_param));
    std::string authlib_cert_base = "third_party/AuthLib/cert";
    try {
        authlib_cert_base =
            ament_index_cpp::get_package_share_directory("ros2_isobus") + "/third_party/AuthLib/cert";
    } catch (const std::exception &) {
        // Keep relative fallback when package share path is unavailable.
    }
    const std::string authlib_root_cert_path = declare_parameter<std::string>(
        "authlib.root_cert_path",
        authlib_cert_base + "/root_ca.der");
    const std::string authlib_client_testlab_cert_path = declare_parameter<std::string>(
        "authlib.client_testlab_cert_path",
        authlib_cert_base + "/aef_testlab.der");
    const std::string authlib_client_manufacturer_cert_path = declare_parameter<std::string>(
        "authlib.client_manufacturer_cert_path",
        authlib_cert_base + "/manu1.der");
    const std::string authlib_client_series_cert_path = declare_parameter<std::string>(
        "authlib.client_series_cert_path",
        authlib_cert_base + "/manu1_series.der");
    const std::string authlib_client_device_cert_path = declare_parameter<std::string>(
        "authlib.client_device_cert_path",
        authlib_cert_base + "/manu1_peer1.der");
    const std::string authlib_client_private_key_hex = declare_parameter<std::string>(
        "authlib.client_private_key_hex",
        authlib_cert_base + "/manu1_peer1.key.hex");
    const std::string authlib_server_public_key_hex = declare_parameter<std::string>(
        "authlib.server_public_key_hex",
        "");

    if (auth_mode != "none" && auth_mode != "dummy" && auth_mode != "authlib") {
        RCLCPP_WARN(
            get_logger(), "Unknown auth_mode='%s', using None", auth_mode_param.c_str());
        auth_mode = "none";
    }

    // auth_mode=None => no auth provider (authentication is bypassed by TimClient)
    set_auth_provider(nullptr);

    if (auth_mode == "dummy") {
        auto provider = std::make_shared<DummyAuthProvider>(
            *this, dummy_auth_period_ms, dummy_auth_implemented_version, dummy_auth_minimum_version);
        provider->reset(client_sa(), server_sa());
        set_auth_provider(provider);
    } else if (auth_mode == "authlib") {
        auto provider = std::make_shared<AuthLibProvider>(
            *this,
            authlib_period_ms,
            authlib_implemented_version,
            authlib_minimum_version,
            authlib_strict,
            authlib_client_cert_payload_max_len,
            authlib_max_slice_iterations,
            authlib_root_cert_path,
            std::array<std::string, 4>{
                authlib_client_testlab_cert_path,
                authlib_client_manufacturer_cert_path,
                authlib_client_series_cert_path,
                authlib_client_device_cert_path},
            authlib_client_private_key_hex,
            authlib_server_public_key_hex);
        provider->reset(client_sa(), server_sa());
        set_auth_provider(provider);
    }

    // process_period_ms is the ROS-side scheduler period for TimClient::process().
    const auto process_period_ms =
        declare_parameter<int>("process_period_ms", 50);
    command_timeout_ms_ = static_cast<std::uint32_t>(
        std::max<int>(0, declare_parameter<int>("command_timeout_ms", 250)));

    tx_pub_ = create_publisher<msg::IsobusFrame>(kBusTxTopic, rclcpp::QoS(100));
    tx_tp_pub_ = create_publisher<msg::IsobusTpFrame>(kBusTxTpTopic, rclcpp::QoS(10));

    // Address manager topics are latched to immediately provide latest SA/NAME context.
    // This follows ISO 11783-5 address-claim lifecycle integration.
    const auto latched_reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    address_book_sub_ = create_subscription<msg::IsobusAddressBook>(
        kAddressManagerAddressBook, latched_reliable_qos,
        [this](const msg::IsobusAddressBook::SharedPtr msg) { update_names_from_address_book(msg); });

    address_status_sub_ = create_subscription<msg::IsobusAddressStatus>(
        kAddressManagerStatus, latched_reliable_qos,
        [this](const msg::IsobusAddressStatus::SharedPtr msg) {
            if (!msg) return;
            if (!is_valid_node_sa(msg->sa)) {
                RCLCPP_WARN(get_logger(), "Ignoring invalid SA from address manager status: 0x%02X", msg->sa);
                return;
            }
            if (!client_sa_ready_ || client_sa() != msg->sa) {
                set_client_sa(msg->sa);
                client_sa_ready_ = true;
                client_name_ready_ = false;
                RCLCPP_INFO(
                    get_logger(), "Updated TIM client SA from address manager status: 0x%02X", msg->sa);
            }
        });

    // ISOBUS single-frame ingress: first try passive server-SA discovery from
    // TIM_ServerStatus broadcasts, then pass frame to TimClient parser.
    bus_sub_ = create_subscription<msg::IsobusFrame>(
        kBusRxTopic, rclcpp::QoS(200),
        [this](const msg::IsobusFrame & fr) {
            try_detect_server_sa_from_broadcast(fr);
            on_frame(fr);
        });

    // TP ingress is accepted only from currently tracked TIM server SA.
    bus_tp_sub_ = create_subscription<msg::IsobusTpFrame>(
        kBusRxTpTopic, rclcpp::QoS(50),
        [this](const msg::IsobusTpFrame & fr) {
            if (!server_sa_ready_) return;
            if (fr.sa != server_sa()) return;
            on_tp_payload(fr.pgn, fr.data);
        });

    cruise_cmd_sub_ = create_subscription<msg::TimCruiseCommand>(
        kTIMCruiseCommandTopic, rclcpp::QoS(20),
        [this](const msg::TimCruiseCommand & cmd) {
            const float speed = static_cast<float>(cmd.speed);
            set_speed_mps(speed, cmd.enable);
            commanded_speed_mps_ = speed;
            speed_enable_cache_ = cmd.enable;
            speed_cmd_seen_ = true;
            speed_last_cmd_ms_ = nowMs();
            refresh_operator_enable();
        });
    curvature_cmd_sub_ = create_subscription<msg::TimCurvatureCommand>(
        kTIMCurvatureCommandTopic, rclcpp::QoS(20),
        [this](const msg::TimCurvatureCommand & cmd) {
            commanded_curvature_km_inv_ = static_cast<float>(cmd.curvature_km_inv);
            set_curvature_km_inv(commanded_curvature_km_inv_, cmd.enable);
            curvature_enable_cache_ = cmd.enable;
            curvature_cmd_seen_ = true;
            curvature_last_cmd_ms_ = nowMs();
            refresh_operator_enable();
        });
    rear_hitch_cmd_sub_ = create_subscription<msg::TimRearHitchCommand>(
        kTIMRearHitchCommandTopic, rclcpp::QoS(20),
        [this](const msg::TimRearHitchCommand & cmd) {
            commanded_rear_hitch_pct_ = static_cast<float>(cmd.position_percent);
            set_rear_hitch_pct(commanded_rear_hitch_pct_, cmd.enable);
            rear_hitch_enable_cache_ = cmd.enable;
            rear_hitch_cmd_seen_ = true;
            rear_hitch_last_cmd_ms_ = nowMs();
            refresh_operator_enable();
        });
    rear_pto_cmd_sub_ = create_subscription<msg::TimRearPtoCommand>(
        kTIMRearPtoCommandTopic, rclcpp::QoS(20),
        [this](const msg::TimRearPtoCommand & cmd) {
            commanded_rear_pto_rpm_ = static_cast<float>(cmd.rpm);
            set_rear_pto_rpm(commanded_rear_pto_rpm_, cmd.engagement);
            rear_pto_enable_cache_ = cmd.engagement;
            rear_pto_cmd_seen_ = true;
            rear_pto_last_cmd_ms_ = nowMs();
            refresh_operator_enable();
        });
    aux_valve_cmd_sub_ = create_subscription<msg::TimAuxValveCommand>(
        kTIMAuxValveCommandTopic, rclcpp::QoS(20),
        [this](const msg::TimAuxValveCommand & cmd) {
            if (cmd.valve_number == 0 || cmd.valve_number > MAX_AUX) return;
            const std::size_t idx = static_cast<std::size_t>(cmd.valve_number - 1U);
            aux_flow_cache_[idx] = static_cast<float>(cmd.flow_percent);
            set_aux_flow(idx, aux_flow_cache_[idx], cmd.enable);
            aux_enable_cache_[idx] = cmd.enable;
            aux_cmd_seen_[idx] = true;
            aux_last_cmd_ms_[idx] = nowMs();
            refresh_operator_enable();
        });

    cruise_status_pub_ = create_publisher<msg::TimCruiseStatus>(kTIMCruiseStatusTopic, rclcpp::QoS(20));
    curvature_status_pub_ = create_publisher<msg::TimCurvatureStatus>(kTIMCurvatureStatusTopic, rclcpp::QoS(20));
    rear_hitch_status_pub_ = create_publisher<msg::TimRearHitchStatus>(kTIMRearHitchStatusTopic, rclcpp::QoS(20));
    rear_pto_status_pub_ = create_publisher<msg::TimRearPtoStatus>(kTIMRearPtoStatusTopic, rclcpp::QoS(20));
    aux_valve_status_pub_ = create_publisher<msg::TimAuxValveStatus>(kTIMAuxValveStatusTopic, rclcpp::QoS(20));

    // Cyclic orchestration tick: timeout supervision + TIM protocol progression + status publishing.
    timer_ = create_wall_timer(
        std::chrono::milliseconds(process_period_ms),
        [this]() {
            apply_command_timeouts();
            process();
        });

    const bool graceful_shutdown_on_exit =
        declare_parameter<bool>("graceful_shutdown_on_exit", true);
    if (graceful_shutdown_on_exit) {
        // Best-effort graceful shutdown announcement when ROS exits cleanly.
        rclcpp::on_shutdown([this]() { request_graceful_shutdown(); });
    }

    if (auth_mode == "none") {
        RCLCPP_WARN(get_logger(), "TIM running in unauthenticated mode (testing only)");
    }
    if (auth_mode == "dummy") {
        RCLCPP_WARN(get_logger(), "TIM dummy-auth enabled (testing only; no real certificate validation)");
    }
    if (auth_mode == "authlib") {
        RCLCPP_INFO(get_logger(), "TIM auth mode: AuthLib provider enabled");
    }
    RCLCPP_INFO(get_logger(), "TIMClient ROS2 wrapper started");
}

FunctionConfig TIMClientROS2::load_function_config(rclcpp::Node & node)
{
    FunctionConfig cfg{};

    cfg.enable_speed = node.declare_parameter<bool>("tim.enable_speed", false);
    cfg.enable_curvature = node.declare_parameter<bool>("tim.enable_curvature", false);
    cfg.enable_rear_pto = node.declare_parameter<bool>("tim.enable_rear_pto", false);
    cfg.enable_rear_hitch = node.declare_parameter<bool>("tim.enable_rear_hitch", false);

    const auto to_u8 = [](int value) -> std::uint8_t { return static_cast<std::uint8_t>(std::clamp(value, 0, 255)); };

    // Function IDs are fixed per guideline-supported mapping in TIMClient.
    cfg.speed_fn_id = FN_VEHICLE_SPEED;
    cfg.curvature_fn_id = FN_EXT_GUIDANCE;
    cfg.rear_pto_fn_id = FN_REAR_PTO;
    cfg.rear_hitch_fn_id = FN_REAR_HITCH;

    cfg.speed_ctrl_mode = to_u8(node.declare_parameter<int>("tim.speed_ctrl_mode", static_cast<int>(CM_SPEED_LINEAR)));
    cfg.curvature_ctrl_mode = to_u8(node.declare_parameter<int>("tim.curvature_ctrl_mode", static_cast<int>(CM_GUIDE_CURVATURE)));
    cfg.rear_pto_ctrl_mode = to_u8(node.declare_parameter<int>("tim.rear_pto_ctrl_mode", static_cast<int>(CM_PTO_SPEED)));
    cfg.rear_hitch_ctrl_mode = to_u8(node.declare_parameter<int>("tim.rear_hitch_ctrl_mode", static_cast<int>(CM_HITCH_PERCENT)));

    const auto aux_fn_ids = node.declare_parameter<std::vector<std::int64_t>>(
        "tim.aux_fn_ids", std::vector<std::int64_t>{});

    const auto aux_ctrl_modes = node.declare_parameter<std::vector<std::int64_t>>(
        "tim.aux_ctrl_modes", std::vector<std::int64_t>{});

    std::vector<std::uint8_t> selected_aux_fns;
    selected_aux_fns.reserve(MAX_AUX);

    std::array<bool, MAX_AUX + 1U> seen{};
    for (const auto raw_id : aux_fn_ids) {
        if (raw_id < 1 || raw_id > static_cast<std::int64_t>(MAX_AUX)) {
            RCLCPP_WARN(
                node.get_logger(),
                "Ignoring invalid tim.aux_fn_ids value: %ld (valid range 1..%u)",
                static_cast<long>(raw_id), static_cast<unsigned>(MAX_AUX));
            continue;
        }
        const auto fn = static_cast<std::uint8_t>(raw_id);
        if (seen[fn]) {
            RCLCPP_WARN(
                node.get_logger(),
                "Ignoring duplicate tim.aux_fn_ids value: %u",
                static_cast<unsigned>(fn));
            continue;
        }
        seen[fn] = true;
        selected_aux_fns.push_back(fn);
        if (selected_aux_fns.size() >= MAX_AUX) break;
    }
    cfg.num_aux = selected_aux_fns.size();

    for (std::size_t i = 0; i < selected_aux_fns.size(); ++i) {
        cfg.aux_fn_id[i] = selected_aux_fns[i];
        cfg.aux_ctrl_mode[i] = (i < aux_ctrl_modes.size())
            ? to_u8(static_cast<int>(aux_ctrl_modes[i]))
            : CM_AUX_PERCENT_FLOW;
    }

    return cfg;
}

bool TIMClientROS2::is_valid_node_sa(std::uint8_t sa)
{
    // J1939/ISO 11783 source addresses 0x00..0xFD are valid node SAs.
    return sa <= 0xFDU;
}

std::uint64_t TIMClientROS2::name_to_u64_le(const std::array<std::uint8_t, 8> & name)
{
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < name.size(); ++i) {
        value |= (static_cast<std::uint64_t>(name[i]) << (8U * i));
    }
    return value;
}

void TIMClientROS2::try_detect_server_sa_from_broadcast(const msg::IsobusFrame & fr)
{
    // AEF 023 server status messages are used as passive discovery source when
    // sa_server parameter is not configured. Accept only valid TIM S2C PGNs.
    if (server_sa_ready_) return;
    const std::uint8_t pf = fr.pf ? fr.pf : static_cast<std::uint8_t>((fr.pgn >> 8) & 0xFFU);
    const bool is_v1_s2c = (fr.pgn == PGN_TIM_V1_S2C) || (pf == static_cast<std::uint8_t>((PGN_TIM_V1_S2C >> 8) & 0xFFU));
    const bool is_v2_s2c = (fr.pgn == PGN_TIM_V2_S2C) || (pf == static_cast<std::uint8_t>((PGN_TIM_V2_S2C >> 8) & 0xFFU));
    if (!is_v1_s2c && !is_v2_s2c) return;
    if (fr.data[0] != MSG_SERVER_STATUS) return;
    if (!is_valid_node_sa(fr.sa)) return;

    set_server_sa(fr.sa);
    server_sa_ready_ = true;
    server_name_ready_ = false;
    RCLCPP_INFO(get_logger(), "Detected TIM server SA from broadcast status: 0x%02X", fr.sa);
}

void TIMClientROS2::update_names_from_address_book(const msg::IsobusAddressBook::SharedPtr & msg)
{
    // NAME tracking keeps server SA aligned with ISO 11783-5 address changes:
    // if server reclaims to another SA with same NAME, update TimClient target SA.
    if (!msg) return;

    for (const auto & entry : msg->entries) {
        if (client_sa_ready_ && entry.sa == client_sa()) {
            const std::uint64_t name_le = name_to_u64_le(entry.name);
            if (!client_name_ready_ || client_name_cached_ != name_le) {
                set_client_name(name_le);
                client_name_cached_ = name_le;
                client_name_ready_ = true;
                RCLCPP_INFO(
                    get_logger(), "Updated TIM client NAME from address manager: %s",
                    to_hex(entry.name).c_str());
            }
            break;
        }
    }

    if (!server_sa_ready_) return;

    if (!server_name_ready_) {
        for (const auto & entry : msg->entries) {
            if (entry.sa == server_sa()) {
                server_name_cached_ = name_to_u64_le(entry.name);
                server_name_ready_ = true;
                RCLCPP_INFO(
                    get_logger(), "Tracked TIM server NAME from address manager: %s (SA 0x%02X)",
                    to_hex(entry.name).c_str(), entry.sa);
                break;
            }
        }
        return;
    }

    for (const auto & entry : msg->entries) {
        if (name_to_u64_le(entry.name) != server_name_cached_) continue;
        if (entry.sa == server_sa()) return;
        if (!is_valid_node_sa(entry.sa)) return;
        set_server_sa(entry.sa);
        RCLCPP_INFO(
            get_logger(), "Updated TIM server SA from tracked server NAME change: 0x%02X", entry.sa);
        return;
    }
}

// TimClient clock hook: return local ROS time in milliseconds.
std::uint32_t TIMClientROS2::nowMs() const
{
    const auto t = this->now();
    return static_cast<std::uint32_t>((t.nanoseconds() / 1000000ULL) & 0xFFFFFFFFu);
}

// TimClient TX hook for single-frame ISOBUS messages.
void TIMClientROS2::sendFrame(const msg::IsobusFrame & fr)
{
    // Block TX until both endpoints are known to avoid undefined destination traffic.
    if (!client_sa_ready_ || !server_sa_ready_) return;
    if (tx_pub_) tx_pub_->publish(fr);
}

// TimClient TX hook for TP payload messages.
void TIMClientROS2::sendTpFrame(const msg::IsobusTpFrame & tp)
{
    // Same endpoint-validity gate for TP transport payload publication.
    if (!client_sa_ready_ || !server_sa_ready_) return;
    if (tx_tp_pub_) tx_tp_pub_->publish(tp);
}

void TIMClientROS2::logInfo(const std::string & msg)
{
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
}

void TIMClientROS2::logWarn(const std::string & msg)
{
    RCLCPP_WARN(get_logger(), "%s", msg.c_str());
}

void TIMClientROS2::logError(const std::string & msg)
{
    RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
}

void TIMClientROS2::apply_command_timeouts()
{
    // Command timeout is a safety fallback:
    // if external command stream stalls, disable affected function request.
    if (command_timeout_ms_ == 0) return;
    const std::uint32_t now = nowMs();

    auto expired = [this, now](bool seen, std::uint32_t last_ms) {
        if (!seen) return false;
        return (now - last_ms) > command_timeout_ms_;
    };

    for (std::size_t i = 0; i < MAX_AUX; ++i) {
        if (aux_enable_cache_[i] && expired(aux_cmd_seen_[i], aux_last_cmd_ms_[i])) {
            aux_enable_cache_[i] = false;
            set_aux_flow(i, aux_flow_cache_[i], false);
        }
    }
    if (speed_enable_cache_ && expired(speed_cmd_seen_, speed_last_cmd_ms_)) {
        speed_enable_cache_ = false;
        set_speed_mps(commanded_speed_mps_, false);
    }
    if (curvature_enable_cache_ && expired(curvature_cmd_seen_, curvature_last_cmd_ms_)) {
        curvature_enable_cache_ = false;
        set_curvature_km_inv(commanded_curvature_km_inv_, false);
    }
    if (rear_hitch_enable_cache_ && expired(rear_hitch_cmd_seen_, rear_hitch_last_cmd_ms_)) {
        rear_hitch_enable_cache_ = false;
        set_rear_hitch_pct(commanded_rear_hitch_pct_, false);
    }
    if (rear_pto_enable_cache_ && expired(rear_pto_cmd_seen_, rear_pto_last_cmd_ms_)) {
        rear_pto_enable_cache_ = false;
        set_rear_pto_rpm(commanded_rear_pto_rpm_, false);
    }
    refresh_operator_enable();
}

void TIMClientROS2::refresh_operator_enable()
{
    // Operator gate follows any pending enable request across all configured functions.
    const bool any_enable = speed_enable_cache_ || curvature_enable_cache_ ||
                            rear_hitch_enable_cache_ || rear_pto_enable_cache_ ||
                            std::any_of(aux_enable_cache_.begin(), aux_enable_cache_.end(), [](bool en) { return en; });
    set_operator_enable(any_enable);
}

void TIMClientROS2::publish_function_status(std::uint8_t fn)
{
    // Publish only the function that was updated from incoming TIM function status.
    const auto & a = actuals();
    const auto & cfg = function_config();
    const auto is_fault = [](std::uint8_t s) {
        return s == TIM_AUTO_FAULT;
    };
    const auto is_active = [](std::uint8_t s) {
        return s == TIM_AUTO_ACTIVE ||
               s == TIM_AUTO_ACTIVE_LIMIT_HIGH ||
               s == TIM_AUTO_ACTIVE_LIMIT_LOW;
    };
    const auto is_limited_high = [](std::uint8_t s) {
        return s == TIM_AUTO_ACTIVE_LIMIT_HIGH;
    };
    const auto is_limited_low = [](std::uint8_t s) {
        return s == TIM_AUTO_ACTIVE_LIMIT_LOW;
    };

    if (cfg.enable_speed && fn == cfg.speed_fn_id && cruise_status_pub_) {
        const auto auto_status = speed_automation_status();
        msg::TimCruiseStatus st;
        st.measured_speed = a.speed_mps;
        st.commanded_speed = commanded_speed_mps_;
        st.automation_status = auto_status;
        st.valid = a.speed_valid;
        st.ok = !is_fault(auto_status);
        st.active = is_active(auto_status);
        st.limited_high = is_limited_high(auto_status);
        st.limited_low = is_limited_low(auto_status);
        st.fault = is_fault(auto_status);
        cruise_status_pub_->publish(st);
        return;
    }
    if (cfg.enable_curvature && fn == cfg.curvature_fn_id && curvature_status_pub_) {
        const auto auto_status = curvature_automation_status();
        msg::TimCurvatureStatus st;
        st.measured_curvature_km_inv = a.curvature_km_inv;
        st.automation_status = auto_status;
        st.valid = a.curvature_valid;
        st.ok = !is_fault(auto_status);
        st.active = is_active(auto_status);
        st.limited_high = is_limited_high(auto_status);
        st.limited_low = is_limited_low(auto_status);
        st.fault = is_fault(auto_status);
        curvature_status_pub_->publish(st);
        return;
    }
    if (cfg.enable_rear_hitch && fn == cfg.rear_hitch_fn_id && rear_hitch_status_pub_) {
        const auto auto_status = rear_hitch_automation_status();
        msg::TimRearHitchStatus st;
        st.position_percent = a.rear_hitch_pct;
        st.automation_status = auto_status;
        st.valid = a.rear_hitch_valid;
        st.ok = !is_fault(auto_status);
        st.active = is_active(auto_status);
        st.limited_high = is_limited_high(auto_status);
        st.limited_low = is_limited_low(auto_status);
        st.fault = is_fault(auto_status);
        rear_hitch_status_pub_->publish(st);
        return;
    }
    if (cfg.enable_rear_pto && fn == cfg.rear_pto_fn_id && rear_pto_status_pub_) {
        const auto auto_status = rear_pto_automation_status();
        msg::TimRearPtoStatus st;
        st.rpm = a.rear_pto_rpm;
        st.automation_status = auto_status;
        st.valid = a.rear_pto_valid;
        st.ok = !is_fault(auto_status);
        st.active = is_active(auto_status);
        st.limited_high = is_limited_high(auto_status);
        st.limited_low = is_limited_low(auto_status);
        st.fault = is_fault(auto_status);
        rear_pto_status_pub_->publish(st);
        return;
    }
    if (aux_valve_status_pub_) {
        for (std::size_t i = 0; i < cfg.num_aux && i < MAX_AUX; ++i) {
            if (fn != cfg.aux_fn_id[i]) continue;
            const auto auto_status = aux_automation_status(i);
            msg::TimAuxValveStatus st;
            st.valve_number = static_cast<std::uint8_t>(i + 1U);
            st.flow_percent = static_cast<float>(a.aux_flow_pct[i]);
            st.automation_status = auto_status;
            st.valid = a.aux_valid[i];
            st.ok = !is_fault(auto_status);
            st.active = is_active(auto_status);
            st.limited_high = is_limited_high(auto_status);
            st.limited_low = is_limited_low(auto_status);
            st.fault = is_fault(auto_status);
            aux_valve_status_pub_->publish(st);
            return;
        }
    }
}

}  // namespace ros2_isobus

int main(int argc, char ** argv)
{
    // Standard ROS2 node bootstrap for TIM client wrapper.
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ros2_isobus::TIMClientROS2>());
    rclcpp::shutdown();
    return 0;
}
