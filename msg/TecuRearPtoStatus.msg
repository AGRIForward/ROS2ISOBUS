# Rear PTO status (ISO 11783-7, PGN 0xFE43 / 65155)
float64 rpm                 # PTO shaft speed (0.125 rpm/bit)
float64 setpoint_rpm        # PTO speed setpoint (0.125 rpm/bit)
uint8 engagement            # 2-bit field: 0=disengaged,1=engaged,2=error,3=NA
uint8 mode                  # 2-bit field: 0=540 rpm,1=1000 rpm,2=error,3=NA
uint8 economy_mode          # 2-bit field: 0=off,1=on,2=error,3=NA
uint8 engagement_request    # 2-bit status: 0=accepted,1=override,2=error,3=NA
uint8 mode_request          # 2-bit status: 0=accepted,1=override,2=error,3=NA
uint8 economy_request       # 2-bit status: 0=accepted,1=override,2=error,3=NA
uint8 speed_limit_status    # 2-bit status: 0=not limited,1=limited,2=error,3=NA
