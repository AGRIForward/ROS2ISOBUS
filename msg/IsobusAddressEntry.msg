# ISOBUS Address Book entry (NAME/SA pair)
# Standard: ISO 11783-5 Address Claim (PGN 60928) derived
uint8[8] name   # 64-bit ISO11783 NAME (big-endian)
uint8 sa        # Source address (0-253, 254=NA)
