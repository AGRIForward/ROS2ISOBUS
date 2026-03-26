AddressManager Node (ISO 11783‑5)
=================================

Purpose  
- Implements the ISO 11783‑5 address claim state machine (PGN 60928) and PGN 59904 requests.  
- Maintains an address book (NAME/SA pairs) and publishes it latched so late‑joining nodes get the latest mapping.

Standards / PGNs  
- PGN 60928 Address Claim (send/respond)  
- PGN 59904 Request (responds to 60928 requests)  
- Keeps an internal NAME vs SA map

Topics  
- `ISOBUS/address_manager/status` (`IsobusAddressStatus`): current SA and state.  
- `ISOBUS/address_manager/address_book` (`IsobusAddressBook`): full NAME/SA list (latched).  
- `ISOBUS/bus_tx_frames` / `ISOBUS/bus_rx_frames`: raw ISOBUS frames (via CanBridge).

Messages (custom)
- `IsobusAddressStatus`: `sa`, `state` — our claimed address and FSM state. 
- `IsobusAddressEntry`: 8-byte `name`, `sa` — single NAME/SA pair (big-endian NAME). 
- `IsobusAddressBook`: array of `IsobusAddressEntry` records (latched snapshot).

Key Parameters  
- `ecu_name_hex` (string, default 0100E0FF00820180)  
- `preferred_address` (int, default 0x1C)

Build/Run  
See root README for build. Run single node:  
```bash
ros2 run ros2_isobus address_manager_node
```
