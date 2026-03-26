TIMClient Node
==============

Purpose
- Implement TIM client-side workflow and expose TIM command/status interfaces as ROS topics.

Interoperability
- Tested to interoperate with Vector CANoe TIM Server in the current project test setup.
- Handles version negotiation, function support/assignment, control-state transitions, and periodic function requests.

Guideline References
- `AEF 007 RIG 3` — ISOBUS Automation.
- `AEF 023 RIG 4` — ISOBUS Automation Principles.
- `AEF 040 RIG 1` — ISOBUS Security Principles.

Standards / PGNs (coverage)
- TIM client-to-server (destination specific):
  - `PGN_TIM_V1_C2S = 9216`
  - `PGN_TIM_V2_C2S = 15616`
- TIM server-to-client (broadcast):
  - `PGN_TIM_V1_S2C = 8960`
  - `PGN_TIM_V2_S2C = 15360`
- Implemented workflow/messages include:
  - Connection version request/response (`MSG_CONNECTION_VERSION`)
  - Client/server status (`MSG_CLIENT_STATUS`, `MSG_SERVER_STATUS`)
  - Functions support request/response (`MSG_FUNCTIONS_SUPPORT_REQ/RESP`)
  - Functions assignment request/response (`MSG_FUNCTIONS_ASSIGN_REQ/RESP`)
  - Functions assignment status request/response (`MSG_FUNCTIONS_ASSIGN_STATUS_REQ/RESP`)
  - Function request/status handling for configured TIM functions (AUX/speed/curvature/rear PTO/rear hitch)

Limitations
- Authentication/lead-server verification requires an auth provider integration; use `auth_mode=None` only for testing.
- Supports one active server target at a time.
- Server SA auto-detect is based on TIM server broadcast status; multi-server arbitration beyond tracked NAME/SA updates is not implemented.

Topics
- Subscribes:
  - `ISOBUS/bus_rx_frames` (`IsobusFrame`)
  - `ISOBUS/bus_rx_tp_frames` (`IsobusTpFrame`)
  - `ISOBUS/address_manager/status` (`IsobusAddressStatus`)
  - `ISOBUS/address_manager/address_book` (`IsobusAddressBook`)
  - `ISOBUS/tim/commands/cruise` (`TimCruiseCommand`)
  - `ISOBUS/tim/commands/curvature` (`TimCurvatureCommand`)
  - `ISOBUS/tim/commands/rear_hitch` (`TimRearHitchCommand`)
  - `ISOBUS/tim/commands/rear_pto` (`TimRearPtoCommand`)
  - `ISOBUS/tim/commands/aux_valve` (`TimAuxValveCommand`)
- Publishes:
  - `ISOBUS/bus_tx_frames` (`IsobusFrame`)
  - `ISOBUS/bus_tx_tp_frames` (`IsobusTpFrame`)
  - `ISOBUS/tim/cruise_status` (`TimCruiseStatus`)
  - `ISOBUS/tim/curvature_status` (`TimCurvatureStatus`)
  - `ISOBUS/tim/rear_hitch_status` (`TimRearHitchStatus`)
  - `ISOBUS/tim/rear_pto_status` (`TimRearPtoStatus`)
  - `ISOBUS/tim/aux_valve_status` (`TimAuxValveStatus`)

Parameters
- Addressing/version:
  - `sa_client` (default `0xFE`, invalid default; updated from AddressManager)
  - `sa_server` (default `0xFE`, invalid default; auto-detect from TIM server broadcast if invalid)
  - `implemented_version` (default `2`)
  - `minimum_version` (default `1`)
  - `client_name_hex` (optional CF NAME override)
  - `auth_mode` (default `None`): `None`, `Dummy`, `AuthLib`
  - `dummy_auth.period_ms`, `dummy_auth.implemented_version`, `dummy_auth.minimum_version`
  - `authlib.period_ms`, `authlib.implemented_version`, `authlib.minimum_version`
  - `authlib.strict` (default `false`): fail authentication on protocol violations; if `false`, log warnings and continue when possible
  - `authlib.max_slice_iterations` (default `1024`): max slice-continue iterations for AuthLib long-running operations
  - `authlib.root_cert_path`
  - `authlib.client_testlab_cert_path`
  - `authlib.client_manufacturer_cert_path`
  - `authlib.client_series_cert_path`
  - `authlib.client_device_cert_path`
  - `authlib.client_private_key_hex` (EC private key as hex string OR path to `.key.hex` file)
  - `authlib.server_public_key_hex` (optional override; if empty, auto-extracted from server device certificate)
- Command TX mode:
  - `command_mode` (default `direct`): `direct`, `periodic`, `both`
  - `command_timeout_ms` (default `250`): if no new setpoint for a function within timeout, that function `enable=false`
- Function selection/mapping:
  - `tim.enable_speed`, `tim.enable_curvature`, `tim.enable_rear_pto`, `tim.enable_rear_hitch`
  - `tim.speed_ctrl_mode`, `tim.curvature_ctrl_mode`
  - `tim.rear_pto_ctrl_mode`, `tim.rear_hitch_ctrl_mode`
  - AUX selection:
    - `tim.aux_fn_ids` = explicit AUX function IDs list (e.g. `[1,3,5]`)
    - `tim.aux_ctrl_modes` = control modes in the same order as selected AUX IDs
- Timing:
  - `process_period_ms`
  - `client_status_period_ms`, `func_tx_period_ms`
  - `conn_req_period_ms`, `support_req_period_ms`, `assign_req_period_ms`
  - `server_status_timeout_ms`, `function_status_timeout_ms`
  - `release_status_timeout_ms`, `reflect_timeout_ms`, `pending_to_active_timeout_ms`
  - `graceful_shutdown_on_exit`

Messages (custom)
- `TimCruiseCommand`: `speed` (m/s), `enable` (bool).
- `TimCruiseStatus`: `measured_speed`, `commanded_speed`, `automation_status`, `valid`, `ok`, `active`, `limited_high`, `limited_low`, `fault`.
- `TimCurvatureCommand`: `curvature_km_inv` (1/km), `enable`.
- `TimCurvatureStatus`: `measured_curvature_km_inv`, `automation_status`, `valid`, `ok`, `active`, `limited_high`, `limited_low`, `fault`.
- `TimRearHitchCommand`: `position_percent`, `enable`.
- `TimRearHitchStatus`: `position_percent`, `automation_status`, `valid`, `ok`, `active`, `limited_high`, `limited_low`, `fault`.
- `TimRearPtoCommand`: `rpm`, `engagement`.
- `TimRearPtoStatus`: `rpm`, `automation_status`, `valid`, `ok`, `active`, `limited_high`, `limited_low`, `fault`.
- `TimAuxValveCommand`: `valve_number`, `flow_percent`, `enable`.
  - `valve_number` is node-internal valve number (1..N), mapped to TIM AUX function IDs via `tim.aux_fn_ids`.
- `TimAuxValveStatus`: `valve_number`, `flow_percent`, `automation_status`, `valid`, `ok`, `active`, `limited_high`, `limited_low`, `fault`.

Build/Run
```bash
ros2 run ros2_isobus tim_client_node
```

Parameter file
- TIM default launch parameters are stored in shared file `config/all_nodes_tim_params.yaml`.
- Update `authlib.root_cert_path`, `authlib.client_*_cert_path`, and `authlib.client_private_key_hex` to match your certificate set.
- Use with node run:
```bash
ros2 run ros2_isobus tim_client_node --ros-args --params-file src/Ros2ISOBUS/config/all_nodes_tim_params.yaml
```
