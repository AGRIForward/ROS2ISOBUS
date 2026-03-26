ROS2ISOBUS – ROS 2 adapters for ISO 11783 / NMEA 2000
======================================================

Experimental ROS 2 nodes that translate key parts of the ISO 11783 (ISOBUS) and NMEA 2000 standards into ROS topics and custom messages.  

**Research and testing only**: not AEF‑conformant; cannot be used in commercial products as such.  

Based on legacy ISOBUS research code used in earlier projects; ROS 2 adaptation and refactor were assisted by OpenAI Codex/ChatGPT.
ISOBUS components have been used e.g. in the robot described in https://doi.org/10.1016/j.ifacol.2022.11.106.


- Authors: Juha Backman et al. (Luonnonvarakeskus / Natural Resources Institute Finland)  
- Contact: juha.backman@luke.fi  
- License: LGPL-3.0 (see LICENSE)
- Change history: [CHANGELOG.md](CHANGELOG.md)

Overview
--------
- **CanBridge** (ISO 11783‑3): SocketCAN ↔ ROS bridge for bus_rx/bus_tx frames (29‑bit extended IDs). Docs: [src/CanBridge/README.md](src/CanBridge/README.md)
- **AddressManager** (ISO 11783‑5): address claim / request, maintains NAME/SA book. Docs: [src/AddressManager/README.md](src/AddressManager/README.md)
- **Diagnostics** (ISO 11783‑12 Annex B minimum): ECU/SW/protocol identification, DM1/DM2/DM3, ISOBUS certification, functionalities, product identification. Docs: [src/Diagnostics/README.md](src/Diagnostics/README.md)
- **TECU Class2/Class3 (legacy) Clients** (ISO 11783‑7/9): speed/hitch/PTO/guidance/cruise/AUX valve, custom ROS msgs. Docs: [src/TECUClient/README.md](src/TECUClient/README.md)
- **TIMClient** (AEF TIM): TIM handshake/state machine + TIM function command/status ROS interfaces. Docs: [src/TIMClient/README.md](src/TIMClient/README.md)
- **NMEA2000Client** (NMEA 2000): parses GNSS/COG/SOG/Attitude PGNs to ROS standard messages. Docs: [src/NMEA2000Client/README.md](src/NMEA2000Client/README.md)
- **TestPanel**: keyboard‑driven debug UI that subscribes to telemetry and can emit either TECU or TIM commands. Docs: [src/TestPanel/README.md](src/TestPanel/README.md)

Dependencies
------------
- ROS 2 (tested on Jazzy) with `rclcpp`, `sensor_msgs`, `geometry_msgs`, `diagnostic_msgs`, `std_srvs`.
- SocketCAN for CAN I/O (`canX` interface).
- C++17 toolchain, `colcon` build.
- TIM authentication (`auth_mode=AuthLib`) requires the proprietary AEF AuthLib (not bundled/open source in general use). AuthLib availability/licensing is handled via AEF/ITK. `aef-support@itk-engineering.de`.

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

All nodes (legacy Class3 stack):
```bash
ros2 launch ros2_isobus all_nodes_class3.launch.py
```
All nodes with TIM enabled (TECU Class2 + TIM):
```bash
ros2 launch ros2_isobus all_nodes_tim.launch.py
```
TIM launch uses one shared parameter file for all launched nodes:
- `config/all_nodes_tim_params.yaml` (installed to `share/ros2_isobus/config/all_nodes_tim_params.yaml`)
- In that file, update `tim_client_node.authlib.*` certificate/key paths for your own setup.

Run TIM client with the same parameter file manually:
```bash
ros2 run ros2_isobus tim_client_node --ros-args --params-file src/Ros2ISOBUS/config/all_nodes_tim_params.yaml
```

Individual nodes:
```bash
ros2 run ros2_isobus can_bridge_node      # needs CAN interface (default can0)
ros2 run ros2_isobus address_manager_node
ros2 run ros2_isobus diagnostics_node
ros2 run ros2_isobus nmea2000_node
ros2 run ros2_isobus tecu_node -- --class3 | --class2
ros2 run ros2_isobus tim_client_node
```
Test panel:
```bash
ros2 run ros2_isobus test_panel_node
ros2 run ros2_isobus test_panel_node --ros-args -p control_interface:=tim
```


Key Parameters
--------------
- CanBridge: `interface` (string, default can0), `disable_loopback` (bool).
- AddressManager: `ecu_name_hex`, `preferred_address`.
- Diagnostics: Annex B parameter groups (`ecu_*`, `software_ident_fields`, `diagnostic_protocol_id`, `active_dtc_*`, `previously_active_dtc_*`, `compliance_*`, `cf_functionality_*`, `product_ident_*`).
- TECU Class3: `command_mode` (direct/periodic/both), `valve_count`.
- TIMClient: `auth_mode` (`None`/`Dummy`/`AuthLib`), `tim.enable_*`, `tim.aux_*`, `command_mode` (direct/periodic/both).
- NMEA2000: `frame_id` for published messages.
- TestPanel: `control_interface` (`tecu`/`tim`).


Contributing
------------
Patches welcome—extend PGN coverage, improve robustness, or add AEF‑compliant behaviours. Please send PRs with clear test notes.
