# ISO11783 Transport Protocol / Extended Transport Protocol payload wrapper
uint8 priority          # J1939/ISOBUS priority (0=highest)
bool page               # True when extended PGN (PDU2)
uint32 pgn              # Parameter Group Number
uint8 sa                # Source address (SA)
uint8 pf                # Protocol format (PF)
uint8 ps                # Protocol specific (PS)
uint8[] data            # Full payload (reassembled or to be fragmented)
