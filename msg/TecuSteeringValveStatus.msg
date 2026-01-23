# Steering valve status (ISO 11783-7 Class3 legacy, PGN 0xFF00/0xFF01 vendor-specific)
float64 measured_steering         # raw valve position/flow (vendor units)
float64 measured_steering_scaled  # scaled steering measurement (vendor units)
uint8 mode                        # valve control mode/state
uint8 command_device              # command source (operator/autonomous/etc.)
