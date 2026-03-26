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
#include <functional>
#include <map>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "ros2_isobus/msg/isobus_frame.hpp"
#include "ros2_isobus/msg/isobus_tp_frame.hpp"

namespace ros2_isobus
{

/*
 *
 * TransportProtocol handles ISO 11783-3 multi-packet transfer:
 *  - TP (5.10): 9..1785 byte payloads using TP.CM / TP.DT
 *  - ETP (5.11): >1785 byte payloads using ETP.CM / ETP.DT
 *  - Reassembly, flow control, retransmit requests and timeout-driven aborts
 *
 */
class TransportProtocol
{
public:
    // Runtime options for TP/ETP behaviour (ISO 11783-3:2018, 5.10/5.11).
    struct Params
    {
        bool enable_tp{true};
        bool enable_etp{true};
        // If true, abort on CTS frames received during active TX transfer.
        // If false, tolerate out-of-order/duplicate CTS for interoperability.
        bool strict_cts_timing{false};
        // Timeout ceiling used by housekeeping; spec timing values are in 5.10.3.5 / 5.13.3.
        std::uint32_t rx_timeout_ms{1250};
        std::uint32_t tx_timeout_ms{1250};
        // TP.CM_RTS byte 5 peer-window cap (5.10.4.2, 5.10.4.3).
        std::uint16_t tx_window_default{8};
        // Local source address filter for destination-specific sessions.
        std::uint8_t local_sa{0xFF};
    };

    // CAN transport hooks provided by the caller.
    using SendCallback = std::function<void(const msg::IsobusFrame &)>;
    using PublishTpCallback = std::function<void(const msg::IsobusTpFrame &)>;

    TransportProtocol(const Params & params,
                      SendCallback send_cb,
                      PublishTpCallback publish_cb,
                      rclcpp::Logger logger,
                      rclcpp::Clock::SharedPtr clock);

    // Entry point for incoming TP.CM/TP.DT and ETP.CM/ETP.DT frames.
    void handleRx(const msg::IsobusFrame & frame);
    // Entry point for outgoing payloads that may require TP/ETP packetization.
    void handleTxRequest(const msg::IsobusTpFrame & frame);
    // Timeout and keep-alive processing (T1/T2/T3/T4/Th from 5.10.3.5, 5.13.3).
    void tick();
    // Update local source address used for destination-specific TP/ETP session handling.
    void setSA(std::uint8_t sa);

private:
    // Protocol mode selection per payload size threshold in 5.10.1 and 5.11.2.
    enum class Protocol { TP, ETP };

    // Receiver-side state for one virtual connection (RTS/BAM or ETP_RTS based).
    struct RxSession
    {
        Protocol proto{Protocol::TP};
        bool bam{false};
        std::uint8_t sa{0};
        std::uint8_t da{0};
        std::uint32_t pgn{0};
        bool page{false};
        std::uint8_t priority{6};
        std::uint32_t total_bytes{0};
        std::uint32_t total_packets{0};
        std::uint32_t next_seq{1};     // Absolute sequence index (1-based).
        std::uint16_t window{0};
        std::uint32_t packet_offset{0}; // ETP DPO offset (5.11.5.5).
        std::uint8_t retransmit_attempts{0};
        std::vector<std::uint8_t> buffer;
        rclcpp::Time last_activity;
        rclcpp::Time start_time;
        rclcpp::Time last_keepalive;    // CTS(0) keep-alive cadence (5.10.3.4.1).
    };

    // Sender-side state for one TP/ETP transmission session.
    struct TxSession
    {
        Protocol proto{Protocol::TP};
        bool bam{false};
        std::uint8_t sa{0};
        std::uint8_t da{0};
        std::uint32_t pgn{0};
        bool page{false};
        std::uint8_t priority{6};
        std::vector<std::uint8_t> payload;
        std::uint32_t total_packets{0};
        std::uint32_t total_bytes{0};
        std::uint32_t next_seq{1};
        std::uint16_t window{0};
        bool awaiting_cts{false};       // Waiting CTS/ACK from receiver.
        bool on_hold{false};            // Hold state after CTS(0).
        std::uint32_t packet_offset{0}; // Current ETP DPO offset base.
        bool sent_dpo{false};           // Avoid duplicate DPO for same window.
        rclcpp::Time last_activity;
        rclcpp::Time start_time;
    };

    // Session map keys: protocol + peer addresses + transported PGN.
    using RxKey = std::tuple<Protocol, std::uint8_t, std::uint8_t, std::uint32_t>;
    using TxKey = std::tuple<Protocol, std::uint8_t, std::uint8_t, std::uint32_t>;

    RxKey makeRxKey(Protocol proto, std::uint8_t sa, std::uint8_t da, std::uint32_t pgn) const;
    TxKey makeTxKey(Protocol proto, std::uint8_t sa, std::uint8_t da, std::uint32_t pgn) const;

    // TP.DT and ETP.DT carry 1-byte sequence + 7-byte payload (5.10.2.2, 5.11.3).
    std::size_t payloadBytesPerPacket(Protocol proto) const;

    // Incoming transport handling.
    void handleControl(const msg::IsobusFrame & frame);
    void handleData(const msg::IsobusFrame & frame);
    void handleTxControl(const msg::IsobusFrame & frame, Protocol proto);

    // Receiver-side session control and CM responses.
    void startRxSession(const msg::IsobusFrame & frame, Protocol proto, bool bam, std::uint32_t total_bytes, std::uint16_t packets, std::uint16_t window, std::uint32_t target_pgn);
    void publishCompleted(RxSession & session);
    void sendCts(const RxSession & session, std::uint16_t next_seq, std::uint16_t allowed);
    void sendEomAck(const RxSession & session);
    void abortRx(const RxSession & session, std::uint8_t reason);

    // Outgoing CM/DT helpers.
    void sendControlFrame(Protocol proto, std::uint8_t sa, std::uint8_t da, const std::array<std::uint8_t, 8> & data, std::uint8_t priority);
    void sendDataFrame(Protocol proto, const TxSession & session, std::uint32_t seq, const std::uint8_t *payload, std::size_t len);
    void sendDpo(const TxSession & session, std::uint32_t offset, std::uint8_t count);

    // Outgoing transfer orchestration (BAM and RTS/CTS paths).
    void sendBamSequence(TxSession & session);
    void sendRts(const TxSession & session);
    void continueRtsCts(TxSession & session, std::uint32_t allowed, std::uint32_t next_seq);
    void finishTxSession(const TxKey & key);

    // Utilities.
    Protocol detectProtoForPayload(std::size_t len) const;
    bool acceptUnicast(std::uint8_t da) const;

    Params params_;
    SendCallback send_cb_;
    PublishTpCallback publish_cb_;
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;

    std::map<RxKey, RxSession> rx_sessions_;
    std::map<TxKey, TxSession> tx_sessions_;
};

}  // namespace ros2_isobus
