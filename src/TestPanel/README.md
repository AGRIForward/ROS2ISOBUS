TestPanel Node
==============

Purpose  
- Interactive keyboard UI for monitoring ISOBUS/TECU telemetry and issuing one-shot or periodic commands (Class3 legacy).  

Monitored topics  
- TECU telemetry: wheel/ground speed, hitch, PTO, curvature/cruise status, steering valve status, AUX valve status.  
- AddressManager: SA and address book entries.  
- NMEA2000: GNSS position, COG/SOG, attitude.

Messages (custom/used)
- `IsobusAddressStatus`, `IsobusAddressBook`: address manager SA and book snapshot.
- `TecuWheelSpeed`, `TecuGroundSpeed`, `TecuRearHitchStatus`, `TecuRearPtoStatus`, `TecuGuidanceStatus`, `TecuCruiseStatus`, `AuxValveStatus`: monitored TECU telemetry.
- `TecuGuidanceCommand`, `TecuCruiseCommand`, `TecuRearHitchCommand`, `TecuRearPtoCommand`, `AuxValveCommand`: commands the panel can publish.
- `geometry_msgs/TwistStamped`: twist commands.

Commands (services/keys)  
- Cruise speed: Up/Down arrows adjust; periodic sends until timeout or stop.  
- Curvature: Left/Right arrows adjust; continuous send.  
- PTO ON/OFF toggle: Space.  
- Hitch up/down: PageUp/PageDown.  
- AUX valves 1–9: Q/A, W/S, E/D, R/F, T/G, Y/H, U/J, I/K, O/L; enable per valve with number keys 1–9.  

Parameters (examples)  
- `guidance_curvature`, `cruise_speed`, `cruise_max_speed`, `rear_hitch_position`, `rear_pto_rpm`, `rear_pto_engage`, `valve_number`, `valve_flow_percent`, `valve_state`, `valve_failsafe` (set before triggering services).  

Run  
```bash
ros2 run ros2_isobus test_panel_node
# adjust params via ros2 param set ..., call services or use keyboard
```
