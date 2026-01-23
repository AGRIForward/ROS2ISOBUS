CanBridge Node (ISO 11783‑3 / SocketCAN)
========================================

Purpose  
- Bridges ROS ISOBUS frames to a SocketCAN interface (29‑bit extended CAN IDs) and back.  
- Publishes bus_rx frames and forwards bus_tx frames onto the physical bus.

Standards / PGNs  
- Converts between SocketCAN 29‑bit Extended IDs and ISOBUS PGN/PF/PS/SA per ISO 11783‑3 (see Table 1 arbitration/control fields). No PGN payload interpretation here.

Topics  
- Subscribes: `ISOBUS/bus_tx_frames` (`IsobusFrame`)  
- Publishes: `ISOBUS/bus_frames` (`IsobusFrame`)

Messages (custom)
- `IsobusFrame`: `priority`, `page`, `pgn`, `sa`, `pf`, `ps`, `data[8]` — transport-neutral ISOBUS frame wrapper.

Parameters  
- `interface` (string, default `can0`)  
- `disable_loopback` (bool, default true)

Build/Run  
```bash
ros2 run ros2_isobus can_bridge_node --ros-args -p interface:=can0
```
