# ISOBUS CAN frame wrapper (transport-agnostic helper)
# Standard: ISO 11783 / SAE J1939 framing (PGN/priority/SA/data)
uint8 priority          # J1939/ISOBUS priority (0=highest)
bool page               # True when extended PGN (PDU2)
uint32 pgn              # Parameter Group Number
uint8 sa                # Source address (SA)
uint8 pf                # Protocol format (PF)
uint8 ps                # Protocol specific (PS)
uint8[8] data           # 8-byte CAN payload
