TECUClient Nodes (Class2 + legacy Class3)
=========================================

Purpose
- Parse TECU ISOBUS PGNs and publish them as ROS topics; accept ROS commands and encode them back to ISOBUS frames.
- Class2 implements ISO 11783‑7/9 speed, hitch and PTO. Class3 adds legacy guidance/cruise/AUX valve control.

Standards / PGNs (coverage)  
- Wheel speed 0xFE48 (65192) → `TecuWheelSpeed`  
- Ground speed 0xFE49 (65193) → `TecuGroundSpeed`  
- Rear hitch status 0xFE45 (65157) → `TecuRearHitchStatus`  
- Rear PTO status 0xFE43 (65155) → `TecuRearPtoStatus`  
- Guidance/curvature cmd 0xACxx → `TecuGuidanceCommand`; status 0xAD00 → `TecuGuidanceStatus` (Class3)  
- Cruise cmd 0xFE0B / status 0xFE0A → `TecuCruiseCommand` / `TecuCruiseStatus` (Class3)  
- AUX valve cmd 0xFE30 / status 0xFE10 → `AuxValveCommand` / `AuxValveStatus` (Class3)  
- Steering wheel angle 0x0003 → `steering_wheel` topic (Class3)  

Limitations  
- Assumes only one TECU is present on the bus; incoming frames are not filtered by source address, so multiple TECUs would collide/overwrite in parsing.

Topics  
- Subscribes (both classes): `ISOBUS/bus_rx_frames` (`IsobusFrame`).  
- Publishes (Class2):
  - `ISOBUS/tecu/wheel_speed` (`TecuWheelSpeed`)
  - `ISOBUS/tecu/ground_speed` (`TecuGroundSpeed`)
  - `ISOBUS/tecu/rear_hitch_status` (`TecuRearHitchStatus`)
  - `ISOBUS/tecu/rear_pto_status` (`TecuRearPtoStatus`)
  - `ISOBUS/tecu/twist_measured` (`geometry_msgs/TwistStamped`)
- Publishes (Class3 additions):
  - `ISOBUS/tecu/steering_wheel`
  - `ISOBUS/tecu/steering_valve_status`
  - `ISOBUS/tecu/guidance_status`
  - `ISOBUS/tecu/cruise_status`
  - `ISOBUS/tecu/aux_valve_status`
- Subscribes (Class3 commands):
  - `ISOBUS/tecu/commands/curvature` (`TecuGuidanceCommand`)
  - `ISOBUS/tecu/commands/cruise` (`TecuCruiseCommand`)
  - `ISOBUS/tecu/commands/rear_hitch` (`TecuRearHitchCommand`)
  - `ISOBUS/tecu/commands/rear_pto` (`TecuRearPtoCommand`)
  - `ISOBUS/tecu/aux_valve_command` (`AuxValveCommand`)
  - `ISOBUS/tecu/twist_command` (`geometry_msgs/TwistStamped`)

Parameters (Class3)
- `command_mode` (`direct`/`periodic`/`both`)
- `valve_count` (int)
- `periodic_timeout_ticks` (int)
- `esp_name_hex` (string), `esp_sa` (int)
- Class2 uses defaults; no extra parameters.

Messages (custom)
- `TecuWheelSpeed`: `speed_ms` (signed m/s), `distance_m` (cumulative m), `key_switch_active` (safety OK), `forward` (direction), `start_stop_state` (0=field,1=transport,2=park,3=no-action), `max_time_of_tractor_power` (minutes remaining).
- `TecuGroundSpeed`: `speed_ms` (signed m/s), `distance_m` (m), `forward` (direction).
- `TecuRearHitchStatus`: `position_percent` (0–100%), `in_work` (0=out,1=in,2=error,3=NA), `position_limit_status` (0=none,1=high,2=low,3=both,7=NA), `nominal_lower_link_force` (%), `draft_n` (N), `exit_code` (6‑bit reason).
- `TecuRearPtoStatus`: `rpm`, `setpoint_rpm`, `engagement` (0=disengaged,1=engaged,2=error,3=NA), `mode` (0=540,1=1000,2=error,3=NA), `economy_mode` (0=off,1=on,2=error,3=NA), `engagement_request`, `mode_request`, `economy_request` (0=accepted,1=override,2=error,3=NA), `speed_limit_status` (0=not limited,1=limited,2=error,3=NA).
- `TecuGuidanceStatus` (Class3): `measured_curvature` (1/m), `command_device` (source enum).
- `TecuCruiseStatus` (Class3): `command_speed` (m/s set), `command_status` (raw TECU ack/status code).
- `AuxValveStatus` (Class3): `valve_number`, `extend_flow_percent`, `retract_flow_percent`, `state` (0=blocked,1=extend,2=retract,3=floating,14=error,15=NA), `failsafe` (bool).
- `TecuGuidanceCommand` (Class3): `curvature` (1/m, left +).
- `TecuCruiseCommand` (Class3): `speed` (m/s request), `max_speed` (m/s cap).
- `TecuRearHitchCommand` (Class3): `position_percent` (0–100% request).
- `TecuRearPtoCommand` (Class3): `rpm` (requested), `engagement` (bool).
- `AuxValveCommand` (Class3): `valve_number`, `flow_percent` (0–100%, sign = direction), `floating` (bool), `failsafe` (bool).

Build/Run
```bash
ros2 run ros2_isobus tecu_node -- --class2
ros2 run ros2_isobus tecu_node -- --class3
```
