# Changelog

All notable public-facing changes to this project are documented in this file.

## 2026-08-24
- New node: `NMEA2000Server`, providing event-driven ROS measurement to NMEA 2000 PGN encoding with fast-packet support for multi-frame PGNs.
- Added standard ROS NMEA 2000 measurement interfaces for velocity and orientation (`TwistWithCovarianceStamped` and `sensor_msgs/Imu`).
- New node: `TECUServer`, a configurable tractor-side Class 1, 2 or 3 server with measurement availability timeouts and Class 3 command interfaces.

## 2026-05-20
- New node: `VTClient`.
- Added VT client helper interfaces (C++ and Python).

## 2026-04-22
- NMEA2000: rapid GNSS position (`publishRapidPosition`) now publishes to a dedicated topic `ISOBUS/nmea2000/gnss_position_data_rapid` (separate from `ISOBUS/nmea2000/gnss_position_data`).
- TIM AuthLib: added parameterized auth payload debug logging (`authlib.debug_auth_payloads`) for troubleshooting.
- CAN bridge: `IsobusFrame` RX timestamp support added and CAN bridge now stamps received frames.
- Added TIM Auth compatibility checker script (`scripts/tim_auth_compat_check.py`) with text/JSON output and CI-friendly exit codes.

## 2026-03-30
- TIMClient status topic publishing changed to event-driven mode: status is published only when a TIM function status frame is received from CAN.

## 2026-03-26
- New nodes: `tim_client_node` and `diagnostics_node`.
- Existing released nodes updated: `can_bridge_node` (TP/ETP multi-packet handling) and `address_manager_node` (SA validity/status handling).

## 2026-01-18
- First public version released.
