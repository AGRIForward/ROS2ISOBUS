# Rear hitch status (ISO 11783-7, PGN 0xFE45 / 65157)
float64 position_percent         # %, 0=down, 100=up (0.4 %/bit)
uint8 in_work                    # 2-bit field: 0=out-of-work,1=in-work,2=error,3=NA
uint8 position_limit_status      # 3-bit field: 0=not limited,1=limited high,2=limited low,3=both,7=NA
float64 nominal_lower_link_force # %, -100..100 (0.8 %/bit, -100 offset)
float64 draft_n                  # N, -320000..322550 (10 N/bit, -320000 offset)
uint8 exit_code                  # 6-bit rear hitch exit/reason code
