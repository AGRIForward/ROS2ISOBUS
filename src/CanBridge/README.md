CanBridge Node (ISO 11783‑3 / SocketCAN)
========================================

Purpose  
- Bridges ROS ISOBUS frames to a SocketCAN interface (29‑bit extended CAN IDs) and back.  
- Publishes bus_rx frames and forwards bus_tx frames onto the physical bus.

Standards / PGNs  
- Converts between SocketCAN 29‑bit Extended IDs and ISOBUS PGN/PF/PS/SA per ISO 11783‑3 (see Table 1 arbitration/control fields). No PGN payload interpretation here.

Topics  
- Subscribes: `ISOBUS/bus_tx_frames` (`IsobusFrame`)  
- Subscribes: `ISOBUS/bus_tx_tp_frames` (`IsobusTpFrame`)  
- Publishes: `ISOBUS/bus_rx_frames` (`IsobusFrame`)
- Publishes (reassembled TP/ETP payloads): `ISOBUS/bus_rx_tp_frames` (`IsobusTpFrame`) when TP/ETP enabled

Messages (custom)
- `IsobusFrame`: `priority`, `page`, `pgn`, `sa`, `pf`, `ps`, `data[8]` — transport-neutral ISOBUS frame wrapper.
- `IsobusTpFrame`: wraps a full payload that was fragmented via TP/ETP (or to be fragmented for TX).

Parameters  
- `interface` (string, default `can0`)  
- `disable_loopback` (bool, default true)
- `enable_tp` / `enable_etp` (bool, default true): enable ISO11783 TP or Extended TP
- `tp_rx_timeout_ms` / `tp_tx_timeout_ms` (int, default 1000): timeouts; internally mapped to spec T1–T4-like limits
- `tp_tx_window_size_default` (int, default 8): CTS window if peer doesn’t specify one
- `tp_local_sa` (int, default 0xFF): initial local source address to accept unicasts for; updated from `ISOBUS/address_manager/status` when available
- `tp_strict_cts_timing` (bool, default false): if true, abort on CTS during active transfer; if false, tolerate duplicate/out-of-order CTS for interop

Transport Protocol behaviour (summary)
- Uses fixed TP/ETP control PGNs with default priority 7; ETP BAM is blocked per spec.
- Supports RTS/CTS/EOMACK/EOMA for TP/ETP, CTS(0) keep-alive, and T1–T4-ish timeouts.
- Handles sequence errors with limited retransmit attempts; issues DPO on ETP wrap/seq errors.

Build/Run  
```bash
ros2 run ros2_isobus can_bridge_node --ros-args -p interface:=can0
```
