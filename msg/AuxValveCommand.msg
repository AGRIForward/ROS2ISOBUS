# Auxiliary valve command (ISO 11783-7 AUX Valve Command, PGN 0xFE30-0xFE3F)
uint8 valve_number
float32 flow_percent   # 0..100 % magnitude, 0.4 %/bit (sign determines direction)
bool floating          # sets floating mode (overrides flow to float)
bool failsafe          # sets fail-safe bit in command
