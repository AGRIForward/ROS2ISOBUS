Diagnostics Node
================

Purpose
- Provide ISO 11783-12 diagnostics minimum Annex B message set over ISOBUS bus topics.

Implemented messages (on request unless noted)
- `PGN 0xFDC5` ECU identification (B.1)
- `PGN 0xFEDA` software identification (B.2)
- `PGN 0xFD42` ISOBUS certification (B.3)
- `PGN 0xFD32` diagnostic protocol identification (B.5)
- `PGN 0xFECA` DM1 active DTC (B.6, periodic + on request)
- `PGN 0xFECB` DM2 previously active DTC (B.7)
- `PGN 0xFECC` DM3 clear previously active DTC (B.8)
- `PGN 0xFC8E` control function functionalities (B.9)
- `PGN 0xFEEB` product identification (B.10)

Topics
- Subscribes:
  - `ISOBUS/bus_rx_frames` (`IsobusFrame`)
  - `ISOBUS/address_manager/status` (`IsobusAddressStatus`, optional SA source)
- Publishes:
  - `ISOBUS/bus_tx_frames` (`IsobusFrame`)
  - `ISOBUS/bus_tx_tp_frames` (`IsobusTpFrame`) for payloads longer than 8 bytes

Parameters
- `source_address` (default `254`)
- `priority` (default `6`)
- `periodic_dm1_enabled` (default `true`)
- `dm1_period_ms` (default `1000`)
- `dm1_byte1`, `dm1_byte2`, `dm2_byte1`, `dm2_byte2` (default `255`)
- `respond_to_requests` (default `true`)
- `nack_unsupported_request` (default `true`)
- `dm3_clear_dm2` (default `true`)
- `use_address_manager_sa` (default `true`)
- `ecu_*` fields for ECU identification payload
- `software_ident_fields` for software identification payload
- `diagnostic_protocol_id` for PGN `0xFD32` byte 1
- `active_dtc_*` and `previously_active_dtc_*` arrays (`spn`, `fmi`, `oc`, `cm`)
- `compliance_*` fields for dynamic `0xFD42` payload construction
- `cf_functionality_ids`, `cf_functionality_generations`, `cf_functionality_options`
- `product_ident_code`, `product_ident_brand`, `product_ident_model`

Notes
- Diagnostic payloads are formed from parameter fields, not hardcoded frame payloads.
- DM1 is sent once per second when active DTCs exist, and when active DTC content changes.
