# Auxiliary valve status/estimated flow (ISO 11783-7 AUX Valve Estimated Flow, PGN 0xFE10-0xFE1F)
uint8 valve_number
float32 extend_flow_percent   # % of max flow, positive extend
float32 retract_flow_percent  # % of max flow, positive retract
uint8 state                   # valve state (0=blocked,1=extend,2=retract,3=floating,14=error,15=NA)
bool failsafe                 # fail-safe mode (true when fail-safe active)
