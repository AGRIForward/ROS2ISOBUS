NMEA2000Server Node
===================

Purpose
- Converts ROS measurements to every NMEA 2000 PGN decoded by NMEA2000Client.
- Transmits only when the corresponding input topic receives a valid sample.
- Uses separate TX topics from the Client outputs to prevent a feedback loop.

Topics / PGNs
- `ISOBUS/nmea2000/tx/attitude` (`Vector3Stamped`, x/y/z = roll/pitch/yaw) → 127257
- `ISOBUS/nmea2000/tx/gnss_position_data_rapid` (`NavSatFix`) → 129025
- `ISOBUS/nmea2000/tx/cog_sog` (`TwistStamped`, angular.z/linear.x = COG/SOG) → 129026
- `ISOBUS/nmea2000/tx/velocity` (`TwistWithCovarianceStamped`, ENU x/y velocity) → 129026
- `ISOBUS/nmea2000/tx/imu` (`sensor_msgs/Imu`, quaternion orientation) → 127257
- `ISOBUS/nmea2000/tx/position_delta` (`NavSatFix`, latitude/longitude are degree deltas) → 129027
- `ISOBUS/nmea2000/tx/gnss_position_data` (`NavSatFix`) → 129029
- `ISOBUS/nmea2000/tx/gnss_pseudo_noise_statistics` (`DiagnosticArray`) → 129539

The Server publishes encoded frames to `ISOBUS/bus_tx_frames` (`IsobusFrame`).
PGNs 129029 and 129539 use NMEA 2000 fast-packet framing. All transmission
requires a claimed source address from AddressManager.

The pseudo-noise diagnostic must contain the same keys published by Client:
`RMS_uncertainty`, `STD_major`, `STD_minor`, `Orientation_major_rad`,
`STD_lat`, `STD_lon`, and `STD_alt`.

Parameters
- `priority` (int, default `2`, clamped to `0`–`7`): transmitted NMEA 2000 priority.

Behavior and limitations
- The Server has no periodic measurement timer.
- A measurement PGN is sent only in response to a valid input sample.
- The Server requires a successfully claimed address from AddressManager before transmitting.

Build/Run
```bash
ros2 run ros2_isobus nmea2000_server
```
