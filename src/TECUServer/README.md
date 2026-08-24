TECU Server Node (Class 1, Class 2 and Class 3)
================================================

Purpose
- Emulate the tractor-side Tractor ECU described by ISO 11783-7/9.
- Encode tractor measurements and actuator states as ISOBUS PGNs.
- Decode Class 3 implement commands and publish them as ROS 2 commands.
- Use Ros2ISOBUS `AddressManager` and `CanBridge`; the server does not claim an address or access SocketCAN directly.

Standards / PGNs (coverage)
- Facilities request 0xFE08 and response 0xFE09.
- Default language 0xFE0F.
- Engine speed 0xF004.
- Wheel speed/distance 0xFE48 and ground speed/distance 0xFE49.
- Rear hitch status 0xFE45 and rear PTO status 0xFE43.
- Time/date 0xFEE6 (Class 2 and Class 3).
- Lighting command 0xFE51 and lighting data 0xFE50 (Class 2 and Class 3).
- AUX valve estimated flow 0xFE10–0xFE1F (Class 2 and Class 3).
- Rear hitch/PTO command 0xFE42 (Class 3).
- AUX valve command 0xFE30–0xFE3F (Class 3).
- Guidance status/command 0xACxx/0xADxx and legacy cruise status/command 0xFE0A/0xFE0B when enabled (Class 3).
- Unsupported or disabled command groups receive a J1939 negative acknowledgement.

Class selection
- `tecu_class: 1` enables Class 1 facilities and measurements.
- `tecu_class: 2` adds Class 2 distance, direction, time/date, lighting and AUX valve status.
- `tecu_class: 3` adds enabled implement command groups.
- One executable implements all three classes. The facilities response is generated from the selected class and enabled hardware.

Topics
- Subscribes:
  - `ISOBUS/bus_rx_frames` (`IsobusFrame`)
  - `ISOBUS/address_manager/status` (`IsobusAddressStatus`)
  - `ISOBUS/tecu/server/inputs/twist_measured` (`geometry_msgs/TwistStamped`)
  - `ISOBUS/tecu/server/inputs/rear_hitch_status` (`TecuRearHitchStatus`)
  - `ISOBUS/tecu/server/inputs/rear_pto_status` (`TecuRearPtoStatus`)
  - `ISOBUS/tecu/server/inputs/aux_valve_status` (`AuxValveStatus`)
  - `ISOBUS/tecu/server/inputs/engine_speed_rpm` (`std_msgs/Float64`)
  - `ISOBUS/tecu/server/inputs/maximum_power_time_min` (`std_msgs/UInt8`)
  - `ISOBUS/tecu/server/inputs/key_switch_active` (`std_msgs/Bool`)
  - `ISOBUS/tecu/server/inputs/guidance_ready` (`std_msgs/Bool`)
  - `ISOBUS/tecu/server/inputs/mechanical_lockout` (`std_msgs/Bool`)
- Publishes:
  - `ISOBUS/bus_tx_frames` (`IsobusFrame`)
  - `ISOBUS/tecu/server/commands/twist` (`geometry_msgs/TwistStamped`)
  - `ISOBUS/tecu/server/commands/cruise` (`TecuCruiseCommand`)
  - `ISOBUS/tecu/server/commands/curvature` (`TecuGuidanceCommand`)
  - `ISOBUS/tecu/server/commands/rear_hitch` (`TecuRearHitchCommand`)
  - `ISOBUS/tecu/server/commands/rear_pto` (`TecuRearPtoCommand`)
  - `ISOBUS/tecu/server/commands/aux_valve` (`AuxValveCommand`)

Measurement availability
- Every physical measurement starts as unavailable.
- Reception of a valid input message makes only the corresponding ISO fields available.
- If no new message arrives within `input_timeout_ms`, those fields return to the ISO `not available` encoding.
- Twist availability controls wheel/ground speed, distance, direction and measured curvature.
- Rear hitch, rear PTO and each AUX valve are tracked independently.
- Both `guidance_ready` and `mechanical_lockout` must be current before a guidance command is accepted.
- Time/date comes from the host UTC clock and therefore has no measurement topic.

Parameters
- `tecu_class` (int, 1–3): implemented T-ECU class.
- `enable_guidance` (bool): advertise and process guidance commands in Class 3.
- `enable_cruise` (bool): process legacy cruise commands in Class 3.
- `enable_rear_hitch` (bool): advertise hitch facilities, transmit status and process Class 3 commands.
- `enable_rear_pto` (bool): advertise PTO facilities, transmit status and process Class 3 commands.
- `enable_lighting` (bool): advertise and respond to lighting commands in Class 2/3.
- `enable_power_management` (bool): advertise power-management facilities.
- `aux_valve_count` (int, 0–16): number of available auxiliary valves.
- `input_timeout_ms` (int, minimum 100): measurement age after which its ISO field is reported as not available.

Limitations
- Lighting data currently reports all lamp states as not available.
- A measurement is encoded as the ISO 11783 `not available` value until its
  input topic has published and again after `input_timeout_ms` without an update.
- Guidance is available only while both guidance-ready and mechanical-lockout
  inputs are current.
- The server requires a successfully claimed address from `AddressManager` before transmitting.

Build/Run
```bash
ros2 launch ros2_isobus tecu_server.launch.py can_interface:=can0
```

Example measurement inputs
```bash
ros2 topic pub -r 10 ISOBUS/tecu/server/inputs/engine_speed_rpm \
  std_msgs/msg/Float64 "{data: 1500.0}"
ros2 topic pub -r 10 ISOBUS/tecu/server/inputs/key_switch_active \
  std_msgs/msg/Bool "{data: true}"
ros2 topic pub -r 2 ISOBUS/tecu/server/inputs/maximum_power_time_min \
  std_msgs/msg/UInt8 "{data: 120}"
```
