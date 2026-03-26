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

#include "TransportProtocol.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>

namespace ros2_isobus
{

namespace
{
using ByteArray8 = std::array<std::uint8_t, 8>;

ByteArray8 make_byte_array(const std::uint8_t * data, std::size_t len)
{
    ByteArray8 result{};
    const auto copy_len = std::min(result.size(), len);
    std::copy_n(data, copy_len, result.begin());
    return result;
}

// TP/ETP connection management and data transfer PGNs (ISO 11783-3:2018, 5.10.4/5.10.5/5.11.5/5.11.6).
constexpr std::uint32_t kPgnTpCm = 0x00EC00;
constexpr std::uint32_t kPgnTpDt = 0x00EB00;
constexpr std::uint32_t kPgnEtpCm = 0x00C800;
constexpr std::uint32_t kPgnEtpDt = 0x00C700;

// TP.CM control bytes (ISO 11783-3:2018, 5.10.4.1).
constexpr std::uint8_t kCtrlRts = 16;
constexpr std::uint8_t kCtrlCts = 17;
constexpr std::uint8_t kCtrlEomAck = 19;
constexpr std::uint8_t kCtrlBam = 32;
constexpr std::uint8_t kCtrlAbort = 255;
// ETP.CM control bytes (ISO 11783-3:2018, 5.11.5).
constexpr std::uint8_t kCtrlEtpRts = 20;
constexpr std::uint8_t kCtrlEtpCts = 21;
constexpr std::uint8_t kCtrlDpo = 22;
constexpr std::uint8_t kCtrlEoma = 23;

// Abort reasons (ISO 11783-3:2018, Table 8).
constexpr std::uint8_t kAbortBusy = 1;
constexpr std::uint8_t kAbortTimeout = 3;
constexpr std::uint8_t kAbortSeqError = 7;
constexpr std::uint8_t kAbortSizeError = 8;
constexpr std::uint8_t kAbortCtsDuringTransfer = 4;
constexpr std::uint8_t kAbortRetransmitLimit = 5;

// Timing constants in seconds (ISO 11783-3:2018, 5.10.3.5 and 5.13.3).
constexpr double kTh = 0.5;   // Keep-alive interval (Th).
constexpr double kT1 = 0.75;  // Missing packet timeout at receiver (T1).
constexpr double kT2 = 1.25;  // Sender timeout after CTS transmitted (T2).
constexpr double kT3 = 1.25;  // Timeout after last DT sent while waiting CTS/ACK (T3).
constexpr double kT4 = 1.05;  // Timeout while connection held open by CTS(0) (T4).

std::uint32_t extract_target_pgn_tp(const msg::IsobusFrame & frame)
{
    return static_cast<std::uint32_t>(frame.data[5]) |
           (static_cast<std::uint32_t>(frame.data[6]) << 8) |
           (static_cast<std::uint32_t>(frame.data[7]) << 16);
}

std::uint32_t extract_target_pgn_etp(const msg::IsobusFrame & frame)
{
    return static_cast<std::uint32_t>(frame.data[5]) |
           (static_cast<std::uint32_t>(frame.data[6]) << 8) |
           (static_cast<std::uint32_t>(frame.data[7]) << 16);
}

std::uint8_t destination_from_frame(const msg::IsobusFrame & frame)
{
    return (frame.pf < 240) ? frame.ps : 0xFF;
}

std::uint16_t min_u16(std::uint16_t a, std::uint16_t b)
{
    return a < b ? a : b;
}

}  // namespace

TransportProtocol::TransportProtocol(const Params & params,
                                     SendCallback send_cb,
                                     PublishTpCallback publish_cb,
                                     rclcpp::Logger logger,
                                     rclcpp::Clock::SharedPtr clock)
: params_(params),
  send_cb_(std::move(send_cb)),
  publish_cb_(std::move(publish_cb)),
  logger_(logger),
  clock_(std::move(clock))
{}

void TransportProtocol::setSA(std::uint8_t sa)
{
    if (sa > 0xFDU)
        return;
    params_.local_sa = sa;
}

// Build a unique key for one RX connection-managed session.
TransportProtocol::RxKey TransportProtocol::makeRxKey(Protocol proto, std::uint8_t sa, std::uint8_t da, std::uint32_t pgn) const
{
    return std::make_tuple(proto, sa, da, pgn);
}

// Build a unique key for one TX connection-managed session.
TransportProtocol::TxKey TransportProtocol::makeTxKey(Protocol proto, std::uint8_t sa, std::uint8_t da, std::uint32_t pgn) const
{
    return std::make_tuple(proto, sa, da, pgn);
}

// TP is used for 9..1785 bytes, ETP for payloads >1785 bytes (5.10.1, 5.11.2).
TransportProtocol::Protocol TransportProtocol::detectProtoForPayload(std::size_t len) const
{
    if (len > 1785)
        return Protocol::ETP;
    return Protocol::TP;
}

// Accept global traffic and destination-specific traffic directed to local SA.
bool TransportProtocol::acceptUnicast(std::uint8_t da) const
{
    return (da == 0xFF) || (params_.local_sa == 0xFF) || (da == params_.local_sa);
}

std::size_t TransportProtocol::payloadBytesPerPacket(Protocol proto) const
{
    return 7; // Both TP.DT and ETP.DT carry 7 data bytes
}

static std::uint8_t tp_default_priority()
{
    return 7;
}

void TransportProtocol::handleRx(const msg::IsobusFrame & frame)
{
    if (!params_.enable_tp && !params_.enable_etp)
        return;

    if (frame.pf == 0xEC || frame.pgn == kPgnTpCm)
    {
        handleControl(frame);
    }
    else if (frame.pf == 0xEB || frame.pgn == kPgnTpDt)
    {
        handleData(frame);
    }
    else if (frame.pf == 0xC8 || frame.pgn == kPgnEtpCm)
    {
        handleControl(frame);
    }
    else if (frame.pf == 0xC7 || frame.pgn == kPgnEtpDt)
    {
        handleData(frame);
    }
    else
    {
        handleTxControl(frame, Protocol::TP);
    }
}

// Handle connection-management frames for TP (5.10.4) and ETP (5.11.5).
void TransportProtocol::handleControl(const msg::IsobusFrame & frame)
{
    const std::uint8_t ctrl = frame.data[0];
    const std::uint8_t da = destination_from_frame(frame);
    const bool is_tp = (frame.pf == 0xEC || frame.pgn == kPgnTpCm);
    const Protocol proto = is_tp ? Protocol::TP : Protocol::ETP;
    const auto now = clock_->now();

    if (!acceptUnicast(da))
        return;

    if (ctrl == kCtrlAbort)
    {
        const auto target_pgn = is_tp ? extract_target_pgn_tp(frame) : extract_target_pgn_etp(frame);
        const auto key = makeRxKey(proto, frame.sa, da, target_pgn);
        rx_sessions_.erase(key);
        return;
    }

    if ((is_tp && ctrl == kCtrlRts) || (is_tp && ctrl == kCtrlBam) ||
        (!is_tp && ctrl == kCtrlEtpRts))
    {
        const std::uint32_t total_bytes = is_tp
            ? static_cast<std::uint32_t>(frame.data[1] | (frame.data[2] << 8))
            : static_cast<std::uint32_t>(frame.data[1] |
                                         (frame.data[2] << 8) |
                                         (frame.data[3] << 16) |
                                         (frame.data[4] << 24));

        const std::uint32_t total_packets = is_tp
            ? static_cast<std::uint32_t>(frame.data[3])
            : static_cast<std::uint32_t>(std::ceil(total_bytes / static_cast<double>(payloadBytesPerPacket(proto))));

        const std::uint16_t window = is_tp ? frame.data[4] : params_.tx_window_default;
        const auto target_pgn = is_tp ? extract_target_pgn_tp(frame) : extract_target_pgn_etp(frame);
        const bool bam = (is_tp && ctrl == kCtrlBam);

        // ETP supports only destination-specific transfer (no global BAM), see 5.11.4.1.
        if (proto == Protocol::ETP && bam)
        {
            RCLCPP_WARN(logger_, "Ignoring BAM for ETP PGN 0x%06X", target_pgn);
            return;
        }
        // TP size and packet limits: 9..1785 bytes, 1..255 sequence domain (5.10.1, 5.10.2.2).
        if (proto == Protocol::TP && (total_bytes < 9 || total_bytes > 1785 || total_packets < 2))
        {
            RCLCPP_WARN(logger_, "TP RTS size/packet count invalid, ignoring PGN 0x%06X", target_pgn);
            if (da != 0xFF)
            {
                RxSession tmp{};
                tmp.proto = proto;
                tmp.sa = frame.sa;
                tmp.da = da;
                tmp.pgn = target_pgn;
                tmp.priority = frame.priority;
                abortRx(tmp, kAbortSizeError);
            }
            return;
        }
        // ETP is only for >1785-byte messages (5.11.2).
        if (proto == Protocol::ETP && (total_bytes <= 1785 || total_packets < 2))
        {
            RCLCPP_WARN(logger_, "ETP RTS below threshold/packet count invalid, ignoring PGN 0x%06X", target_pgn);
            if (da != 0xFF)
            {
                RxSession tmp{};
                tmp.proto = proto;
                tmp.sa = frame.sa;
                tmp.da = da;
                tmp.pgn = target_pgn;
                tmp.priority = frame.priority;
                abortRx(tmp, kAbortSizeError);
            }
            return;
        }
        if (proto == Protocol::TP && total_packets > 255)
        {
            RCLCPP_WARN(logger_, "TP RTS packet count >255, aborting PGN 0x%06X", target_pgn);
            if (da != 0xFF)
            {
                RxSession tmp{};
                tmp.proto = proto;
                tmp.sa = frame.sa;
                tmp.da = da;
                tmp.pgn = target_pgn;
                tmp.priority = frame.priority;
                abortRx(tmp, kAbortSizeError);
            }
            return;
        }
        // ETP packet-numbering uses 24-bit domain (5.11.3, 5.11.5).
        if (proto == Protocol::ETP && (total_packets > 16777215 ||
                                       total_bytes > static_cast<std::uint32_t>((0x01000000 - 1) * payloadBytesPerPacket(proto))))
        {
            RCLCPP_WARN(logger_, "ETP RTS packet count too large, aborting PGN 0x%06X", target_pgn);
            if (da != 0xFF)
            {
                RxSession tmp{};
                tmp.proto = proto;
                tmp.sa = frame.sa;
                tmp.da = da;
                tmp.pgn = target_pgn;
                tmp.priority = frame.priority;
                abortRx(tmp, kAbortSizeError);
            }
            return;
        }

        startRxSession(frame, proto, bam, total_bytes, total_packets, window, target_pgn);
        if (!bam)
        {
            auto key = makeRxKey(proto, frame.sa, da, target_pgn);
            const auto it = rx_sessions_.find(key);
            if (it != rx_sessions_.end())
            {
                const std::uint16_t allow = min_u16(it->second.window ? it->second.window : params_.tx_window_default,
                                                    static_cast<std::uint16_t>(it->second.total_packets));
                sendCts(it->second, 1, allow);
            }
        }
        return;
    }

    if (ctrl == kCtrlCts || (!is_tp && ctrl == kCtrlEtpCts))
    {
        handleTxControl(frame, proto);
        return;
    }

    if (ctrl == kCtrlEomAck || ctrl == kCtrlEoma)
    {
        handleTxControl(frame, proto);
        return;
    }

    if (ctrl == kCtrlDpo && proto == Protocol::ETP)
    {
        const auto target_pgn = extract_target_pgn_etp(frame);
        const auto key = makeRxKey(proto, frame.sa, da, target_pgn);
        const auto it = rx_sessions_.find(key);
        if (it != rx_sessions_.end())
        {
            // DPO offset establishes absolute packet numbering base (5.11.5.5).
            std::uint32_t offset = static_cast<std::uint32_t>(frame.data[2]) |
                                   (static_cast<std::uint32_t>(frame.data[3]) << 8) |
                                   (static_cast<std::uint32_t>(frame.data[4]) << 16);
            if (offset >= it->second.total_packets)
            {
                abortRx(it->second, kAbortSeqError);
                rx_sessions_.erase(it);
                return;
            }
            it->second.packet_offset = offset;
            it->second.next_seq = offset + 1;
        }
        return;
    }
}

// Handle TP.DT/ETP.DT data packets and session reassembly (5.10.5, 5.11.6).
void TransportProtocol::handleData(const msg::IsobusFrame & frame)
{
    const bool is_tp_dt = (frame.pf == 0xEB || frame.pgn == kPgnTpDt);
    const Protocol proto = is_tp_dt ? Protocol::TP : Protocol::ETP;
    const std::uint8_t da = destination_from_frame(frame);
    if (!acceptUnicast(da))
        return;

    const std::uint32_t rel_seq = frame.data[0]; // ETP.DT uses 1-byte seq, TP.DT also 1-byte seq
    const std::uint32_t seq = rel_seq;

    std::uint8_t payload_offset = 1;
    std::array<std::uint8_t, 8> data{};
    std::copy(frame.data.begin(), frame.data.end(), data.begin());

    // Find matching session
    for (auto it = rx_sessions_.begin(); it != rx_sessions_.end(); ++it)
    {
        auto & session = it->second;
        if (session.proto != proto || session.sa != frame.sa || session.da != da)
            continue;

        const auto abs_seq = session.packet_offset + seq;
        if (seq == 0 || (session.proto == Protocol::ETP && rel_seq > 255))
        {
            abortRx(session, kAbortSeqError);
            rx_sessions_.erase(it);
            return;
        }

        if (abs_seq != session.next_seq || abs_seq > session.total_packets || abs_seq > 0xFFFFFF)
        {
            // Sequence error: request retransmit or abort with Table 8 reason 5 after retries.
            if (!session.bam && session.retransmit_attempts < 2)
            {
                session.retransmit_attempts++;
                const std::uint16_t remaining = static_cast<std::uint16_t>(session.total_packets - (session.next_seq - 1));
                const std::uint16_t allowed = min_u16(session.window ? session.window : params_.tx_window_default, remaining);
                // For ETP, request resend from current absolute offset using DPO (5.11.5.5).
                if (session.proto == Protocol::ETP)
                {
                    TxSession tmp{};
                    tmp.proto = Protocol::ETP;
                    tmp.sa = session.sa;
                    tmp.da = session.sa;
                    tmp.pgn = session.pgn;
                    tmp.priority = session.priority;
                    sendDpo(tmp, session.next_seq - 1, static_cast<std::uint8_t>(std::min<std::uint16_t>(allowed, 255)));
                }
                sendCts(session, session.next_seq, allowed);
                session.last_activity = clock_->now();
                return;
            }
            abortRx(session, kAbortRetransmitLimit); // retransmit limit reached
            rx_sessions_.erase(it);
            return;
        }

        const auto remaining = session.total_bytes - static_cast<std::uint32_t>(session.buffer.size());
        const auto per_packet = payloadBytesPerPacket(proto);
        const std::size_t copy_len = static_cast<std::size_t>(std::min<std::uint32_t>(remaining, static_cast<std::uint32_t>(per_packet)));
        session.buffer.insert(session.buffer.end(), frame.data.begin() + payload_offset, frame.data.begin() + payload_offset + copy_len);
        session.next_seq = static_cast<std::uint32_t>(session.next_seq + 1);
        session.last_activity = clock_->now();
        session.retransmit_attempts = 0;

        if (session.buffer.size() >= session.total_bytes)
        {
            publishCompleted(session);
            if (!session.bam)
                sendEomAck(session);
            rx_sessions_.erase(it);
            return;
        }

        if (!session.bam)
        {
            const std::uint16_t received = session.next_seq - 1;
            const std::uint16_t window = session.window ? session.window : params_.tx_window_default;
            if ((received % window) == 0)
            {
                const auto remaining_packets = static_cast<std::uint16_t>(session.total_packets - received);
                const auto allowed = min_u16(window, remaining_packets);
                sendCts(session, session.next_seq, allowed);
            }
        }
        return;
    }
}

void TransportProtocol::publishCompleted(RxSession & session)
{
    if (!publish_cb_)
        return;

    msg::IsobusTpFrame tp;
    tp.priority = session.priority;
    tp.page = session.page;
    tp.pgn = session.pgn;
    tp.sa = session.sa;
    tp.pf = static_cast<std::uint8_t>((session.pgn >> 8) & 0xFF);
    tp.ps = (tp.pf < 240) ? session.da : static_cast<std::uint8_t>(session.pgn & 0xFF);
    tp.data = session.buffer;
    publish_cb_(tp);
}

// Send TP.CM_CTS / ETP.CM_CTS flow-control response (5.10.4.3, 5.11.5.4).
void TransportProtocol::sendCts(const RxSession & session, std::uint16_t next_seq, std::uint16_t allowed)
{
    const std::uint8_t local_sa = (params_.local_sa == 0xFF) ? 0xFE : params_.local_sa;
    std::array<std::uint8_t, 8> data{};
    if (session.proto == Protocol::TP)
    {
        data[0] = kCtrlCts;
        data[1] = static_cast<std::uint8_t>(allowed & 0xFF);
        data[2] = static_cast<std::uint8_t>(next_seq & 0xFF);
        data[3] = 0xFF;
        data[4] = 0xFF; // reserved
        data[5] = static_cast<std::uint8_t>(session.pgn & 0xFF);
        data[6] = static_cast<std::uint8_t>((session.pgn >> 8) & 0xFF);
        data[7] = static_cast<std::uint8_t>((session.pgn >> 16) & 0xFF);
    }
    else
    {
        data[0] = kCtrlEtpCts;
        data[1] = static_cast<std::uint8_t>(allowed & 0xFF);
        data[2] = static_cast<std::uint8_t>(next_seq & 0xFF);
        data[3] = static_cast<std::uint8_t>((next_seq >> 8) & 0xFF);
        data[4] = static_cast<std::uint8_t>((next_seq >> 16) & 0xFF);
        data[5] = static_cast<std::uint8_t>(session.pgn & 0xFF);
        data[6] = static_cast<std::uint8_t>((session.pgn >> 8) & 0xFF);
        data[7] = static_cast<std::uint8_t>((session.pgn >> 16) & 0xFF);
    }
    // Reply to sender SA of the active RX session.
    sendControlFrame(session.proto, local_sa, session.sa, data, session.priority);
}

// Send TP.CM_EndOfMsgACK / ETP.CM_EOMA on successful destination-specific completion.
void TransportProtocol::sendEomAck(const RxSession & session)
{
    const std::uint8_t local_sa = (params_.local_sa == 0xFF) ? 0xFE : params_.local_sa;
    std::array<std::uint8_t, 8> data{};
    if (session.proto == Protocol::TP)
    {
        data[0] = kCtrlEomAck;
        data[1] = static_cast<std::uint8_t>(session.total_bytes & 0xFF);
        data[2] = static_cast<std::uint8_t>((session.total_bytes >> 8) & 0xFF);
        data[3] = static_cast<std::uint8_t>(session.total_packets & 0xFF);
        data[4] = 0xFF; // reserved
        data[5] = static_cast<std::uint8_t>(session.pgn & 0xFF);
        data[6] = static_cast<std::uint8_t>((session.pgn >> 8) & 0xFF);
        data[7] = static_cast<std::uint8_t>((session.pgn >> 16) & 0xFF);
    }
    else
    {
        data[0] = kCtrlEoma;
        data[1] = static_cast<std::uint8_t>(session.total_bytes & 0xFF);
        data[2] = static_cast<std::uint8_t>((session.total_bytes >> 8) & 0xFF);
        data[3] = static_cast<std::uint8_t>((session.total_bytes >> 16) & 0xFF);
        data[4] = static_cast<std::uint8_t>((session.total_bytes >> 24) & 0xFF);
        data[5] = static_cast<std::uint8_t>(session.pgn & 0xFF);
        data[6] = static_cast<std::uint8_t>((session.pgn >> 8) & 0xFF);
        data[7] = static_cast<std::uint8_t>((session.pgn >> 16) & 0xFF);
    }
    // Reply to sender SA of the active RX session.
    sendControlFrame(session.proto, local_sa, session.sa, data, session.priority);
}

// Send TP/ETP Connection Abort (control byte 255) with Table 8 reason code.
void TransportProtocol::abortRx(const RxSession & session, std::uint8_t reason)
{
    const std::uint8_t local_sa = (params_.local_sa == 0xFF) ? 0xFE : params_.local_sa;
    std::array<std::uint8_t, 8> data{};
    data[0] = kCtrlAbort;
    data[1] = reason;
    // TP/ETP Abort: bytes 2-4 reserved FFh; bytes 5-7 PGN (3 bytes)
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = static_cast<std::uint8_t>(session.pgn & 0xFF);
    data[6] = static_cast<std::uint8_t>((session.pgn >> 8) & 0xFF);
    data[7] = static_cast<std::uint8_t>((session.pgn >> 16) & 0xFF);
    // Reply to sender SA of the active RX session.
    sendControlFrame(session.proto, local_sa, session.sa, data, session.priority);
}

// Packetize outgoing payload and start BAM or RTS/CTS transfer state machine.
void TransportProtocol::handleTxRequest(const msg::IsobusTpFrame & frame)
{
    const auto len = frame.data.size();
    if (len <= 8)
    {
        msg::IsobusFrame out;
        out.priority = frame.priority;
        out.page = frame.page;
        out.pgn = frame.pgn;
        out.sa = (params_.local_sa == 0xFF) ? frame.sa : params_.local_sa;
        out.pf = frame.pf;
        out.ps = frame.ps;
        out.data = make_byte_array(frame.data.data(), frame.data.size());
        send_cb_(out);
        return;
    }

    const Protocol proto = detectProtoForPayload(len);
    if (proto == Protocol::ETP && !params_.enable_etp)
    {
        RCLCPP_WARN(logger_, "ETP disabled, dropping payload len=%zu", len);
        return;
    }
    if (proto == Protocol::TP && !params_.enable_tp)
    {
        RCLCPP_WARN(logger_, "TP disabled, dropping payload len=%zu", len);
        return;
    }

    if (proto == Protocol::ETP && frame.ps == 0xFF)
    {
        RCLCPP_WARN(logger_, "ETP does not support BAM/global, dropping payload len=%zu", len);
        return;
    }

    TxSession session;
    session.proto = proto;
    session.sa = (params_.local_sa == 0xFF) ? frame.sa : params_.local_sa;
    session.priority = frame.priority;
    session.page = frame.page;
    session.pgn = frame.pgn;
    session.da = (frame.pf < 240) ? frame.ps : 0xFF;
    session.bam = (session.da == 0xFF);
    session.payload.assign(frame.data.begin(), frame.data.end());
    session.total_bytes = static_cast<std::uint32_t>(session.payload.size());
    const auto per_packet = payloadBytesPerPacket(proto);
    session.total_packets = static_cast<std::uint32_t>(std::ceil(session.payload.size() / static_cast<double>(per_packet)));
    session.window = params_.tx_window_default;
    auto now = clock_->now();
    session.last_activity = now;
    session.start_time = now;
    session.packet_offset = 0;
    session.on_hold = false;

    const auto key = makeTxKey(proto, session.sa, session.da, session.pgn);
    tx_sessions_[key] = session;

    if (session.bam)
    {
        sendBamSequence(tx_sessions_[key]);
        finishTxSession(key);
    }
    else
    {
        sendRts(tx_sessions_[key]);
    }
}

// TP broadcast path: BAM announce + DT sequence (5.10.3.2, 5.10.4.6, 5.10.5).
void TransportProtocol::sendBamSequence(TxSession & session)
{
    std::array<std::uint8_t, 8> cm{};
    cm[0] = kCtrlBam;
    cm[1] = static_cast<std::uint8_t>(session.total_bytes & 0xFF);
    cm[2] = static_cast<std::uint8_t>((session.total_bytes >> 8) & 0xFF);
    cm[3] = static_cast<std::uint8_t>(session.total_packets & 0xFF);
    cm[4] = 0xFF;
    cm[5] = static_cast<std::uint8_t>(session.pgn & 0xFF);
    cm[6] = static_cast<std::uint8_t>((session.pgn >> 8) & 0xFF);
    cm[7] = static_cast<std::uint8_t>((session.pgn >> 16) & 0xFF);
    sendControlFrame(session.proto, session.sa, session.da, cm, session.priority);

    const auto per_packet = payloadBytesPerPacket(session.proto);
    for (std::uint16_t seq = 1; seq <= session.total_packets; ++seq)
    {
        const auto offset = static_cast<std::size_t>((seq - 1) * per_packet);
        const auto remaining = session.payload.size() - offset;
        const auto len = std::min<std::size_t>(per_packet, remaining);
        sendDataFrame(session.proto, session, seq, session.payload.data() + offset, len);
        // BAM spacing shall be 10..200 ms; use minimum compliant 10 ms (5.10.2.4, 5.13.3).
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
}

// Send TP.CM_RTS or ETP.CM_RTS session open request.
void TransportProtocol::sendRts(const TxSession & session)
{
    std::array<std::uint8_t, 8> cm{};
    if (session.proto == Protocol::TP)
    {
        cm[0] = kCtrlRts;
        cm[1] = static_cast<std::uint8_t>(session.total_bytes & 0xFF);
        cm[2] = static_cast<std::uint8_t>((session.total_bytes >> 8) & 0xFF);
        cm[3] = static_cast<std::uint8_t>(session.total_packets & 0xFF);
        cm[4] = static_cast<std::uint8_t>(session.window & 0xFF);
        cm[5] = static_cast<std::uint8_t>(session.pgn & 0xFF);
        cm[6] = static_cast<std::uint8_t>((session.pgn >> 8) & 0xFF);
        cm[7] = static_cast<std::uint8_t>((session.pgn >> 16) & 0xFF);
    }
    else
    {
        cm[0] = kCtrlEtpRts;
        cm[1] = static_cast<std::uint8_t>(session.total_bytes & 0xFF);
        cm[2] = static_cast<std::uint8_t>((session.total_bytes >> 8) & 0xFF);
        cm[3] = static_cast<std::uint8_t>((session.total_bytes >> 16) & 0xFF);
        cm[4] = static_cast<std::uint8_t>((session.total_bytes >> 24) & 0xFF);
        cm[5] = static_cast<std::uint8_t>(session.pgn & 0xFF);
        cm[6] = static_cast<std::uint8_t>((session.pgn >> 8) & 0xFF);
        cm[7] = static_cast<std::uint8_t>((session.pgn >> 16) & 0xFF);
    }
    sendControlFrame(session.proto, session.sa, session.da, cm, session.priority);
}

// Continue sender-side transfer after CTS grant (and ETP DPO windowing when needed).
void TransportProtocol::continueRtsCts(TxSession & session, std::uint32_t allowed, std::uint32_t next_seq)
{
    session.awaiting_cts = false;
    session.next_seq = next_seq;
    session.last_activity = clock_->now();
    session.sent_dpo = false;

    const auto per_packet = payloadBytesPerPacket(session.proto);

    for (std::uint32_t i = 0; i < allowed && session.next_seq <= session.total_packets; ++i)
    {
        if (session.proto == Protocol::ETP)
        {
            const auto rel_seq = session.next_seq - session.packet_offset;
            if (rel_seq == 0 || rel_seq > 255)
            {
                // request new offset window
                if (!session.sent_dpo)
                {
                    const std::uint32_t offset = (session.next_seq > 0) ? (session.next_seq - 1) : 0;
                    const std::uint8_t count = static_cast<std::uint8_t>(std::min<std::uint32_t>(allowed - i, 255));
                    sendDpo(session, offset, count);
                    session.sent_dpo = true;
                }
                session.awaiting_cts = true;
                session.on_hold = true;
                break;
            }
        }
        const auto seq = session.next_seq++;
        const auto offset = static_cast<std::size_t>((seq - 1) * per_packet);
        const auto remaining = session.payload.size() - offset;
        const auto len = std::min<std::size_t>(per_packet, remaining);
        sendDataFrame(session.proto, session, seq, session.payload.data() + offset, len);
    }
}

// Process incoming CTS/EOMACK/ABORT (and DPO for ETP) for active TX sessions.
void TransportProtocol::handleTxControl(const msg::IsobusFrame & frame, Protocol proto)
{
    const std::uint8_t ctrl = frame.data[0];
    const std::uint8_t da = destination_from_frame(frame); // our SA when we are sender
    const auto target_pgn = (proto == Protocol::TP) ? extract_target_pgn_tp(frame) : extract_target_pgn_etp(frame);
    auto it = tx_sessions_.end();
    for (auto iter = tx_sessions_.begin(); iter != tx_sessions_.end(); ++iter)
    {
        const auto & sess = iter->second;
        if (sess.proto == proto && sess.da == frame.sa && sess.pgn == target_pgn)
        {
            it = iter;
            break;
        }
    }
    if (it == tx_sessions_.end())
        return;

    auto & session = it->second;
    const auto key = makeTxKey(proto, session.sa, session.da, session.pgn);
    session.last_activity = clock_->now();

    if (ctrl == kCtrlCts || ctrl == kCtrlEtpCts)
    {
        const std::uint32_t allowed = (proto == Protocol::TP)
            ? frame.data[1]
            : static_cast<std::uint32_t>(frame.data[1]);
        const std::uint32_t next_seq = (proto == Protocol::TP)
            ? frame.data[2]
            : static_cast<std::uint32_t>(frame.data[2] | (frame.data[3] << 8) | (frame.data[4] << 16));
        std::uint32_t effective_next_seq = next_seq;

        // CTS during active transfer is protocol error (Table 8 reason 4, 5.10.4.3).
        if (!session.awaiting_cts && session.next_seq > 1 && !session.on_hold)
        {
            if (params_.strict_cts_timing)
            {
                RxSession tmp{};
                tmp.proto = session.proto;
                // Address abort to the control frame sender (peer), not to local SA.
                tmp.sa = frame.sa;
                tmp.da = da;
                tmp.pgn = session.pgn;
                tmp.priority = session.priority;
                abortRx(tmp, kAbortCtsDuringTransfer);
                tx_sessions_.erase(it);
                return;
            }
            if (effective_next_seq < session.next_seq)
            {
                RCLCPP_WARN(
                    logger_,
                    "Ignoring stale CTS during active transfer PGN 0x%06X (next=%u, expected>=%u)",
                    session.pgn,
                    static_cast<unsigned>(effective_next_seq),
                    static_cast<unsigned>(session.next_seq));
                return;
            }
            // Interop mode: tolerate CTS during active transfer without extra log spam.
        }

        if (effective_next_seq == 0 || effective_next_seq > (session.total_packets + 1))
        {
            if (params_.strict_cts_timing)
            {
                RxSession tmp{};
                tmp.proto = session.proto;
                tmp.sa = frame.sa;
                tmp.da = da;
                tmp.pgn = session.pgn;
                tmp.priority = session.priority;
                abortRx(tmp, kAbortCtsDuringTransfer);
                tx_sessions_.erase(it);
                return;
            }
            RCLCPP_WARN(
                logger_,
                "Ignoring invalid CTS next-sequence PGN 0x%06X (next=%u, max=%u)",
                session.pgn,
                static_cast<unsigned>(effective_next_seq),
                static_cast<unsigned>(session.total_packets + 1));
            return;
        }
        if (effective_next_seq < session.next_seq)
        {
            if (params_.strict_cts_timing)
            {
                RxSession tmp{};
                tmp.proto = session.proto;
                tmp.sa = frame.sa;
                tmp.da = da;
                tmp.pgn = session.pgn;
                tmp.priority = session.priority;
                abortRx(tmp, kAbortCtsDuringTransfer);
                tx_sessions_.erase(it);
                return;
            }
            RCLCPP_WARN(
                logger_,
                "Ignoring regressing CTS sequence PGN 0x%06X (next=%u, current=%u)",
                session.pgn,
                static_cast<unsigned>(effective_next_seq),
                static_cast<unsigned>(session.next_seq));
            return;
        }

        if (allowed == 0)
        {
            session.awaiting_cts = true;
            session.on_hold = true;
            return;
        }

        if (proto == Protocol::ETP)
        {
            session.packet_offset = effective_next_seq - 1;
            session.next_seq = effective_next_seq;
        }

        session.on_hold = false;
        continueRtsCts(session, allowed, effective_next_seq);
        if (session.next_seq > session.total_packets)
        {
            finishTxSession(key);
        }
        return;
    }

    if (ctrl == kCtrlDpo && proto == Protocol::ETP)
    {
        const std::uint32_t count = frame.data[1];
        const std::uint32_t offset = static_cast<std::uint32_t>(frame.data[2]) |
                                     (static_cast<std::uint32_t>(frame.data[3]) << 8) |
                                     (static_cast<std::uint32_t>(frame.data[4]) << 16);
        session.packet_offset = offset;
        session.next_seq = offset + 1;
        session.on_hold = (count == 0);
        if (!session.on_hold && count > 0)
        {
            continueRtsCts(session, count, session.next_seq);
            if (session.next_seq > session.total_packets)
            {
                finishTxSession(key);
            }
        }
        return;
    }

    if (ctrl == kCtrlEomAck || ctrl == kCtrlEoma)
    {
        finishTxSession(key);
        return;
    }

    if (ctrl == kCtrlAbort)
    {
        tx_sessions_.erase(it);
        return;
    }
}

void TransportProtocol::finishTxSession(const TxKey & key)
{
    tx_sessions_.erase(key);
}

// Compose and send TP/ETP connection-management CAN frame.
void TransportProtocol::sendControlFrame(Protocol proto, std::uint8_t sa, std::uint8_t da, const std::array<std::uint8_t, 8> & data, std::uint8_t priority)
{
    msg::IsobusFrame out;
    out.priority = tp_default_priority();
    out.page = false;
    out.sa = (params_.local_sa == 0xFF) ? sa : params_.local_sa;
    if (proto == Protocol::TP)
    {
        out.pf = 0xEC;
        out.ps = da;
        out.pgn = kPgnTpCm;
    }
    else
    {
        out.pf = 0xC8;
        out.ps = da;
        out.pgn = kPgnEtpCm;
    }
    out.data = make_byte_array(data.data(), data.size());
    send_cb_(out);
}

// Compose and send TP.DT / ETP.DT packet with sequence byte and up to 7 payload bytes.
void TransportProtocol::sendDataFrame(Protocol proto, const TxSession & session, std::uint32_t seq, const std::uint8_t *payload, std::size_t len)
{
    const auto per_packet = payloadBytesPerPacket(proto);
    msg::IsobusFrame out;
    out.priority = tp_default_priority();
    out.page = false;
    out.sa = session.sa;
    if (proto == Protocol::TP)
    {
        out.pf = 0xEB;
        out.ps = session.da;
        out.pgn = kPgnTpDt;
    }
    else
    {
        out.pf = 0xC7;
        out.ps = session.da;
        out.pgn = kPgnEtpDt;
    }
    auto data = ByteArray8{};
    std::fill(data.begin(), data.end(), 0xFF);
    if (proto == Protocol::TP)
    {
        data[0] = static_cast<std::uint8_t>(seq & 0xFF);
        std::copy_n(payload, len, data.begin() + 1);
    }
    else
    {
        const auto rel_seq = static_cast<std::uint8_t>((seq - session.packet_offset) & 0xFF);
        data[0] = rel_seq == 0 ? 1 : rel_seq; // ETP.DT sequence in one CTS window is 1..255.
        std::copy_n(payload, std::min<std::size_t>(per_packet, len), data.begin() + 1);
    }
    out.data = data;
    send_cb_(out);
}

// Send ETP.CM_DPO to define absolute packet offset for the next ETP.DT block (5.11.5.5).
void TransportProtocol::sendDpo(const TxSession & session, std::uint32_t offset, std::uint8_t count)
{
    std::array<std::uint8_t, 8> data{};
    data[0] = kCtrlDpo;
    data[1] = count;
    data[2] = static_cast<std::uint8_t>(offset & 0xFF);
    data[3] = static_cast<std::uint8_t>((offset >> 8) & 0xFF);
    data[4] = static_cast<std::uint8_t>((offset >> 16) & 0xFF);
    data[5] = static_cast<std::uint8_t>(session.pgn & 0xFF);
    data[6] = static_cast<std::uint8_t>((session.pgn >> 8) & 0xFF);
    data[7] = static_cast<std::uint8_t>((session.pgn >> 16) & 0xFF);
    sendControlFrame(Protocol::ETP, session.sa, session.da, data, session.priority);
}

// Initialize (or replace) RX session after RTS/BAM/ETP_RTS, per 5.10.4.2 latest-RTS rule.
void TransportProtocol::startRxSession(const msg::IsobusFrame & frame, Protocol proto, bool bam, std::uint32_t total_bytes, std::uint16_t packets, std::uint16_t window, std::uint32_t target_pgn)
{
    RxSession session;
    session.proto = proto;
    session.bam = bam;
    session.sa = frame.sa;
    session.da = destination_from_frame(frame);
    session.pgn = target_pgn;
    session.page = (target_pgn & 0x010000) != 0;
    session.priority = frame.priority;
    session.total_bytes = total_bytes;
    session.total_packets = packets;
    session.window = window;
    session.next_seq = 1;
    session.packet_offset = 0;
    session.buffer.reserve(total_bytes);
    auto now = clock_->now();
    session.last_activity = now;
    session.start_time = now;
    session.last_keepalive = now;

    const auto key = makeRxKey(proto, session.sa, session.da, session.pgn);
    // Per spec: if multiple RTS for same SA/PGN arrive, replace previous without abort.
    rx_sessions_.erase(key);
    rx_sessions_[key] = std::move(session);
}

// Periodic housekeeping: CTS(0) keep-alive and T1/T2/T3/T4 timeout closure (5.10.3.5).
void TransportProtocol::tick()
{
    const auto now = clock_->now();
    const auto rx_timeout = rclcpp::Duration::from_seconds(params_.rx_timeout_ms / 1000.0);
    const auto tx_timeout = rclcpp::Duration::from_seconds(params_.tx_timeout_ms / 1000.0);

    for (auto it = rx_sessions_.begin(); it != rx_sessions_.end(); )
    {
        const bool awaiting_more = (it->second.next_seq <= it->second.total_packets);
        if (!it->second.bam && awaiting_more && (now - it->second.last_keepalive) > rclcpp::Duration::from_seconds(kTh))
        {
            // keep-alive CTS(0) per 5.10.3.4.1
            const std::uint16_t remaining = static_cast<std::uint16_t>(it->second.total_packets - (it->second.next_seq - 1));
            sendCts(it->second, static_cast<std::uint16_t>(it->second.next_seq), 0);
            it->second.last_keepalive = now;
            ++it;
            continue;
        }

        // Receiver timeout T1 (missing packet)
        if ((now - it->second.last_activity) > rclcpp::Duration::from_seconds(kT1))
        {
            if (it->second.da != 0xFF)
            {
                abortRx(it->second, kAbortTimeout); // timeout
            }
            it = rx_sessions_.erase(it);
        }
        else
        {
            ++it;
        }
    }

    for (auto it = tx_sessions_.begin(); it != tx_sessions_.end(); )
    {
        double limit_sec = kT2;
        if (it->second.on_hold || it->second.awaiting_cts)
        {
            limit_sec = kT4;
        }
        else if (it->second.next_seq > it->second.total_packets)
        {
            limit_sec = kT3;
        }
        const auto limit = rclcpp::Duration::from_seconds(limit_sec);

        if ((now - it->second.last_activity) > limit)
        {
            // Notify peer if unicast
            if (it->second.da != 0xFF)
            {
                RxSession tmp{};
                tmp.proto = it->second.proto;
                tmp.sa = it->second.sa;
                tmp.da = it->second.da;
                tmp.pgn = it->second.pgn;
                tmp.priority = it->second.priority;
                abortRx(tmp, kAbortTimeout);
            }
            it = tx_sessions_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

}  // namespace ros2_isobus
