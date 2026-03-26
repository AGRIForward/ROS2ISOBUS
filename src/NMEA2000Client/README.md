NMEA2000Client Node
===================

Purpose  
- Parses selected NMEA 2000 PGNs from raw ISOBUS/N2K frames and republishes as standard ROS messages.

Standards / PGNs (coverage)  
- GNSS Position Data 129029 (0xF805) → `sensor_msgs/NavSatFix` (+ covariance from STD/HDOP/PDOP)  
- GNSS Pseudo Noise Stats 129539 (0xFA06) → `diagnostic_msgs/DiagnosticArray`  
- Position Rapid Update 129025 (0xF801) → `NavSatFix` rapid topic  
- COG & SOG Rapid Update 129026 (0xF802) → `geometry_msgs/TwistStamped`  
- Attitude 127257 (0xF119) → `geometry_msgs/Vector3Stamped`  
- Position Delta 129025 delta variant (0xF803) → updates rapid lat/lon  

Limitations  
- Assumes a single GNSS/NMEA2000 talker on the bus; frames are not filtered by source address (SA), so multiple GNSS devices would overwrite each other.

Topics  
- Publishes:  
  - `ISOBUS/nmea2000/gnss_position_data` (`NavSatFix`)  
  - `ISOBUS/nmea2000/gnss_pseudo_noise_statistics` (`DiagnosticArray`)  
  - `ISOBUS/nmea2000/cog_sog` (`TwistStamped`)  
  - `ISOBUS/nmea2000/rapid_position` (`NavSatFix`)  
  - `ISOBUS/nmea2000/attitude` (`Vector3Stamped`)  
- Subscribes: `ISOBUS/bus_rx_frames` (`IsobusFrame`)

Messages (custom/used)
- `sensor_msgs/NavSatFix`, `diagnostic_msgs/DiagnosticArray`, `geometry_msgs/TwistStamped`, `geometry_msgs/Vector3Stamped`: standard ROS outputs.
- `IsobusFrame`: transport wrapper for incoming N2K/ISOBUS frames.

Parameters  
- `frame_id` (string, default `map`)

Build/Run  
```bash
ros2 run ros2_isobus nmea2000_node --ros-args -p frame_id:=map
```
