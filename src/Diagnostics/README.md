Diagnostics Node
================

Purpose
- Provide ISO 11783-12 diagnostics minimum Annex B message set over ISOBUS bus topics.

Implemented messages (on request unless noted)
- `PGN 0xFDC5` ECU identification (B.1)
- `PGN 0xFEDA` software identification (B.2)
- `PGN 0xFD42` ISOBUS certification (B.3), only when explicitly enabled with valid certification data
- `PGN 0xFD32` diagnostic protocol identification (B.5)
- `PGN 0xFECA` DM1 active DTC (B.6, periodic + on request)
- `PGN 0xFECB` DM2 previously active DTC (B.7)
- `PGN 0xFECC` DM3 clear previously active DTC (B.8)
- `PGN 0xFC8E` control function functionalities (B.9)
- `PGN 0xFC8D` product identification (B.10)

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
- `active_dtc_*` and `previously_active_dtc_*` arrays (`spn`, `fmi`, `oc`; `cm` is retained for configuration compatibility but encoded as zero as required by ISO 11783-12:2019)
- `compliance_certification_enabled` (default `false`): enables `0xFD42` responses only after all required certification fields validate
- `compliance_*` fields for dynamic `0xFD42` payload construction; validated against ISO 11783-7:2015 A.29 and B.27 when enabled
  - `compliance_cert_year`: `2000..2061`
  - `compliance_cert_revision`: `0..30`
  - `compliance_lab_type`: `0` (self-certification) or `1` (AEF-certified laboratory)
  - `compliance_test_lab_id`: 11-bit assigned laboratory/manufacturer ID (`0..2047`)
  - `compliance_reference_number`: `0..64255`
  - `compliance_message_revision`: `1` (third-edition format implemented here)
  - `compliance_type_byte4` and `compliance_type_byte5`: `0` (reserved)
- `cf_functionality_ids`, `cf_functionality_generations`, `cf_functionality_options`
- `product_ident_code`, `product_ident_brand`, `product_ident_model`

Notes
- Diagnostic payloads are formed from parameter fields, not hardcoded frame payloads.
- Public configurations leave certification information disabled. Enabling the
  message requires actual certification data and does not itself establish ISO
  conformity or AEF certification.
- ECU, software and product identification payloads include the required trailing `*` delimiter.
- Trailing zero functionality-option bytes are omitted from PGN `0xFC8E` as required by A.12.
- DM1 is sent once per second when active DTCs exist, and when active DTC content changes.
