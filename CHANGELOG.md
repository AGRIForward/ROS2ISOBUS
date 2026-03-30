# Changelog

All notable public-facing changes to this project are documented in this file.

## 2026-03-30
- TIMClient status topic publishing changed to event-driven mode: status is published only when a TIM function status frame is received from CAN.

## 2026-03-26
- New nodes: `tim_client_node` and `diagnostics_node`.
- Existing released nodes updated: `can_bridge_node` (TP/ETP multi-packet handling) and `address_manager_node` (SA validity/status handling).

## 2026-01-18
- First public version released.
