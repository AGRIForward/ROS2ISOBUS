ROS2ISOBUS – ROS 2 adapters for ISO 11783 / NMEA 2000
======================================================

Experimental ROS 2 nodes that translate key parts of the ISO 11783 (ISOBUS) and NMEA 2000 standards into ROS topics and custom messages.  

**Research and testing only**: not AEF‑conformant; cannot be used in commercial products as such.  

Based on legacy ISOBUS research code used in earlier projects; ROS 2 adaptation and refactor were assisted by OpenAI Codex/ChatGPT.
ISOBUS components have been used e.g. in the robot described in https://doi.org/10.1016/j.ifacol.2022.11.106.


- Authors: Juha Backman et al. (Luonnonvarakeskus / Natural Resources Institute Finland)  
- Contact: juha.backman@luke.fi  
- License: LGPL-3.0 (see LICENSE)

Overview
--------
- **CanBridge** (ISO 11783‑3): SocketCAN ↔ ROS bridge for bus_rx/bus_tx frames (29‑bit extended IDs). Docs: [src/CanBridge/README.md](src/CanBridge/README.md)
- **AddressManager** (ISO 11783‑5): address claim / request, maintains NAME/SA book. Docs: [src/AddressManager/README.md](src/AddressManager/README.md)
- **TECU Class2/Class3 (legacy) Clients** (ISO 11783‑7/9): speed/hitch/PTO/guidance/cruise/AUX valve, custom ROS msgs. Docs: [src/TECUClient/README.md](src/TECUClient/README.md)
- **NMEA2000Client** (NMEA 2000): parses GNSS/COG/SOG/Attitude PGNs to ROS standard messages. Docs: [src/NMEA2000Client/README.md](src/NMEA2000Client/README.md)
- **TestPanel**: keyboard‑driven debug UI that subscribes to all telemetry and can emit TECU/AUX commands on demand. Docs: [src/TestPanel/README.md](src/TestPanel/README.md)

Dependencies
------------
- ROS 2 (tested on Jazzy) with `rclcpp`, `sensor_msgs`, `geometry_msgs`, `diagnostic_msgs`, `std_srvs`.
- SocketCAN for CAN I/O (`canX` interface).
- C++17 toolchain, `colcon` build.

Building
--------
```bash
colcon build --packages-select ros2_isobus
source install/setup.bash
```

Launching / Running
-------------------
Bring up SocketCAN (example):
```bash
sudo ./start_can.sh # default can0 can be changed by argument
```

All nodes at once:
```bash
ros2 launch ros2_isobus all_nodes.launch.py
```
Individual nodes:
```bash
ros2 run ros2_isobus can_bridge_node      # needs CAN interface (default can0)
ros2 run ros2_isobus address_manager_node
ros2 run ros2_isobus nmea2000_node
ros2 run ros2_isobus tecu_node -- --class3 | --class2
```
Test panel:
```bash
ros2 run ros2_isobus test_panel_node
```


Key Parameters
--------------
- CanBridge: `interface` (string, default can0), `disable_loopback` (bool).
- AddressManager: `ecu_name_hex`, `preferred_address`.
- TECU Class3: `command_mode` (direct/periodic/both), `valve_count`, `periodic_timeout_ticks`, `esp_name_hex`, `esp_sa`.
- NMEA2000: `frame_id` for published messages.
- TestPanel: interactive; adjust command parameters via ROS params (`guidance_curvature`, `cruise_speed`, etc.).


Contributing
------------
Patches welcome—extend PGN coverage, improve robustness, or add AEF‑compliant behaviours. Please send PRs with clear test notes.
