TestPanel Node
==============

Purpose
- Interactive keyboard UI for monitoring ISOBUS telemetry and sending control commands.
- Command interface is selectable:
  - `tecu` (legacy TECU Class3 command topics/messages),
  - `tim` (TIM command topics/messages).

Control interface selection
- Parameter: `control_interface`
  - `tecu` (default)
  - `tim`

In `tim` mode the panel publishes `Tim*Command` messages and subscribes `Tim*Status`.
In `tecu` mode the panel publishes `Tecu*Command`/`AuxValveCommand` and subscribes TECU status topics.

Default behavior
- All command functions are enabled in panel logic.
- AUX command channels are fixed to 9 (keys 1..9 and Q/A..O/L mappings).

Monitored data
- AddressManager: own SA and address-book entries.
- NMEA2000: GNSS position, COG/SOG, attitude.
- Control status topics based on selected interface (`tecu` or `tim`).

Keyboard controls
- Arrow Up/Down: cruise speed command.
- Arrow Left/Right: curvature command.
- Space: PTO toggle.
- PageUp/PageDown: rear hitch.
- AUX valves:
  - adjust flow with Q/A, W/S, E/D, R/F, T/G, Y/H, U/J, I/K, O/L
  - toggle valve command enable with keys 1..9

Run
```bash
ros2 run ros2_isobus test_panel_node
ros2 run ros2_isobus test_panel_node --ros-args -p control_interface:=tim
```
