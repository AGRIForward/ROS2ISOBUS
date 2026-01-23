# Wheel speed (ISO 11783-7, PGN 0xFE48 / 65192)
float64 speed_ms          # m/s, signed
float64 distance_m        # m, cumulative distance
bool key_switch_active    # true when key switch OK (safety)
bool forward              # true = forward, false = reverse (bits 2..1)
uint8 start_stop_state    # implement start/stop (bits 6..5): 0=field,1=transport,2=park,3=no-action
uint8 max_time_of_tractor_power  # minutes remaining of tractor power (1 min/bit, 0-250)
