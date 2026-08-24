VTClient Node (ISO 11783-6 Virtual Terminal Client)
=====================================================

See also
- [VTClientInterface.md](VTClientInterface.md) for application-side C++ wrappers (`vt_client_interface.hpp`) and usage examples.
- [VTClientInterfacePython.md](VTClientInterfacePython.md) for application-side Python wrappers (`vt_client_interface.py`) and usage examples.

Purpose
- Implements ISO 11783-6 Virtual Terminal client-side session and object pool publishing workflow.
- Converts PoolEdit XML into VT object pool binary and exchanges VT function messages over ISOBUS TP.

Creating the VT object pool
1. Create and edit the VT object pool with the external
   [PoolEdit editor](https://github.com/moehman/PoolEditDist).
2. Select **Export** in the PoolEdit user interface. The export produces the
   `.xml` object-pool file consumed by `vt_client_node`.
3. Set the node's `xml_file` parameter to the exported file. The path can be
   absolute or relative to the installed `ros2_isobus` package share directory.
4. Use the XML object `name` values as `<name_token>` values in ROS topic paths
   and in the C++/Python VT Client interface wrappers.

The XML is converted to the binary VT object pool at runtime. A modified copy
of [PoolEditParser](https://github.com/moehman/PoolEditParser) is embedded under
`src/VTClient/pooledit_parser`; the local changes integrate the parser with
VTClient and extend its object handling, including AUX-N objects. The embedded
parser retains its GNU GPL version 3-or-later license notices and is licensed
differently from the LGPL-licensed ROS2ISOBUS code.

Example parameter configuration:
```yaml
vt_client_node:
  ros__parameters:
    xml_file: "config/my_object_pool.xml"
```

Standards / PGNs (coverage)
- VT-to-WorkingSet: `PGN 58880` (`0xE600`)
- WorkingSet-to-VT: `PGN 59136` (`0xE700`)
- Addressing source: ISO 11783-5 Address Claim (`PGN 60928`) via AddressManager topics.

Session workflow (implemented)
- Wait for valid local SA from AddressManager.
- Send Working Set Maintenance (ISO 11783-7).
- Request VT version (fn `0xC7`).
- Request memory (fn `0xC0`).
- Build object pool from XML after version response (VT-reported profile preferred when available).
- Transfer pool with fn `0x11` + raw object records.
- Send End of Object Pool fn `0x12` and wait VT fn `0x12` response.
- Activate mask with Change Active Mask fn `0xAD`.
- Enter active state and route VT interaction/events to ROS topics.

Topics
- Subscribes:
  - `ISOBUS/bus_rx_frames` (`IsobusFrame`)
  - `ISOBUS/bus_rx_tp_frames` (`IsobusTpFrame`)
  - `ISOBUS/address_manager/status` (`IsobusAddressStatus`)
  - `ISOBUS/address_manager/address_book` (`IsobusAddressBook`)
  - `ISOBUS/vt/number/<name_token>/set` (`std_msgs/msg/Float64`)
  - `ISOBUS/vt/input_number/<name_token>/set` (`std_msgs/msg/Float64`)
  - `ISOBUS/vt/string/<name_token>/set` (`std_msgs/msg/String`)
  - `ISOBUS/vt/input_string/<name_token>/set` (`std_msgs/msg/String`)
  - `ISOBUS/vt/list/<name_token>/set` (`std_msgs/msg/Int32`)
  - `ISOBUS/vt/input_bool/<name_token>/set` (`std_msgs/msg/Bool`)
  - `ISOBUS/vt/button/<name_token>/enabled/set` (`std_msgs/msg/Bool`)
  - `ISOBUS/vt/input_number/<name_token>/enabled/set` (`std_msgs/msg/Bool`)
  - `ISOBUS/vt/input_string/<name_token>/enabled/set` (`std_msgs/msg/Bool`)
  - `ISOBUS/vt/input_bool/<name_token>/enabled/set` (`std_msgs/msg/Bool`)
  - `ISOBUS/vt/container/<name_token>/visible/set` (`std_msgs/msg/Bool`)
  - `ISOBUS/vt/active_mask/<mask_name>/set` (`std_msgs/msg/Empty`)
  - `ISOBUS/vt/softkey_mask/<mask_name>/set` (`std_msgs/msg/Empty`)
  - `ISOBUS/vt/update` (`std_msgs/msg/String`, XML (element or objectpool) parsed with pooledit_parser and sent as fn11 + fn12)
- Publishes:
  - `ISOBUS/bus_tx_frames` (`IsobusFrame`)
  - `ISOBUS/bus_tx_tp_frames` (`IsobusTpFrame`)
  - `ISOBUS/vt/status` (`ros2_isobus/msg/VTStatus`)
  - `ISOBUS/vt/session/state` (`ros2_isobus/msg/VTSessionState`)
  - `ISOBUS/vt/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)
  - `ISOBUS/vt/event/button/<name_token>` (`std_msgs/msg/UInt8`, KeyActivationCode)
  - `ISOBUS/vt/event/softkey/<name_token>` (`std_msgs/msg/UInt8`, KeyActivationCode)
  - `ISOBUS/vt/event/pointing` (`ros2_isobus/msg/VTPointingEvent`)
  - `ISOBUS/vt/event/navigation` (`ros2_isobus/msg/VTNavigationEvent`)
  - `ISOBUS/vt/number/<name_token>/value` (`std_msgs/msg/Float64`)
  - `ISOBUS/vt/input_number/<name_token>/value` (`std_msgs/msg/Float64`)
  - `ISOBUS/vt/string/<name_token>/value` (`std_msgs/msg/String`)
  - `ISOBUS/vt/input_string/<name_token>/value` (`std_msgs/msg/String`)
  - `ISOBUS/vt/list/<name_token>/value` (`std_msgs/msg/Int32`)
  - `ISOBUS/vt/input_bool/<name_token>/value` (`std_msgs/msg/Bool`)
  - `ISOBUS/vt/button/<name_token>/enabled/value|result`
  - `ISOBUS/vt/input_number/<name_token>/enabled/value|result`
  - `ISOBUS/vt/input_string/<name_token>/enabled/value|result`
  - `ISOBUS/vt/input_bool/<name_token>/enabled/value|result`
  - `ISOBUS/vt/container/<name_token>/visible/value|result`
  - `ISOBUS/vt/active_mask/<mask_name>/value|result`
  - `ISOBUS/vt/softkey_mask/<mask_name>/value|result`
  - `ISOBUS/vt/aux/status` (`ros2_isobus/msg/VTAuxStatus`)
  - `ISOBUS/vt/aux/<name_token>/input/value` (`std_msgs/msg/Float64`, normalized AUX-N value)
  - `ISOBUS/vt/aux/<name_token>/input/raw` (`ros2_isobus/msg/VTAuxInputRaw`)
  - `ISOBUS/vt/aux/<name_token>/assignment/value` (`ros2_isobus/msg/VTAuxAssignment`)
  - `ISOBUS/vt/aux/<name_token>/assignment/result` (`ros2_isobus/msg/CommandResult`)
  - `ISOBUS/vt/update_result` (`ros2_isobus/msg/VTUpdateResult`)

Addressing behavior
- `sa_local` is a startup value only; runtime SA is updated from `ISOBUS/address_manager/status`.
- VT SA is detected from `ISOBUS/address_manager/address_book` by NAME Function ID 29 (Virtual Terminal).
- `sa_vt` is retained as fallback if VT entry is not yet visible in the address book.

Parameters
- Core:
  - `xml_file` (string, required): PoolEdit XML path.
  - `sa_local` (int, default `254`): startup local SA before AddressManager update.
  - `sa_vt` (int, default `38`): fallback VT SA.
- XML -> POOL scaling profile:
  - `vt_dimension` (int, default `200`)
  - `vt_softkey_width` (int, default `60`)
  - `vt_softkey_height` (int, default `32`)
  - `vt_colors` (int, default `256`)
  - `vt_use_reported_display_profile` (bool, default `true`): use VT-reported profile when present/valid.
- Pool transfer / activation:
  - `vt_working_set_id` (int, default `0`)
  - `vt_object_pool_id` (int, default `0`)
  - `vt_working_set_version` (int, default `6`)
- Session timing:
  - `vt_session_timeout_ms` (int, default `1000`)
  - `vt_session_retries` (int, default `3`)
  - `vt_session_tick_ms` (int, default `100`)
  - `vt_wait_address_claim` (bool, default `false`)
  - `vt_ws_maintenance_period_ms` (int, default `1000`)
  - `vt_status_timeout_ms` (int, default `3000`)
- AUX-N:
  - `vt_aux_n_support` (bool, default `false`): enable AUX-N function-object bindings, assignments and input-status routing.
  - `vt_aux_preferred_store_override` (bool, default `false`): persist incoming AUX assignment as preferred even if VT sends "do not store as preferred".
  - `vt_aux_preferred_assignment_file` (string, default empty): optional file for loading/saving AUX preferred assignments.
    - File format: one mapping per line:
      - `<aux_input_name_hex> <aux_input_object_id_hex> <aux_function_object_id_hex>`

AUX-N behavior
- AUX-N support must be enabled with `vt_aux_n_support:=true`.
- Each PoolEdit XML `auxiliaryfunction` object creates topics under `ISOBUS/vt/aux/<name_token>`.
- The VT assignment command is accepted only when the function object exists, the AUX input NAME resolves through the AddressManager address book and the input object/type is valid.
- Input status is routed only when its source address, AUX input NAME and input object ID match the active assignment.
- The normalized value is published as `Float64`; the original values and state bits remain available in `VTAuxInputRaw`.
- Preferred assignments can be loaded from and saved to `vt_aux_preferred_assignment_file`.

Notes
- VT function IDs are fixed in code to ISO 11783-6 values (not runtime-parameterized).
- If VT does not expose usable display profile fields in version response, fallback scaling parameters are used.
- PoolEdit parser code is embedded under `src/VTClient/pooledit_parser`.

Custom Message Structures
- `ros2_isobus/msg/VTStatus`
  - `uint8 active_ws_sa`
  - `uint16 visible_data_alarm_mask_id`
  - `uint16 visible_soft_key_mask_id`
  - `uint8 busy_codes`
  - `uint8 current_command_function`
  - `bool busy_updating_visible_mask`
  - `bool busy_saving_to_nonvolatile`
  - `bool busy_executing_command`
  - `bool busy_executing_macro`
  - `bool parsing_active`
  - `bool aux_learn_mode_active`
  - `bool out_of_memory`
- `ros2_isobus/msg/VTSessionState`
  - `string state`
  - `uint8 retry_count`
  - `uint8 max_retries`
  - `bool pool_ready`
- `ros2_isobus/msg/VTPointingEvent`
  - `uint8 event_code`
  - `uint16 object_id`
  - `int32 x`
  - `int32 y`
- `ros2_isobus/msg/VTNavigationEvent`
  - `uint8 function_code`
  - `uint16 object_id`
  - `bool pressed`
- `ros2_isobus/msg/CommandResult`
  - `bool success`
  - `uint8 error_code`
  - `string target`
  - `string message`
- `ros2_isobus/msg/VTUpdateResult`
  - `bool success`
  - `uint32 pending_updates`
  - `uint32 in_progress_updates`
  - `string detail`
- `ros2_isobus/msg/VTAuxAssignment`
  - Identifies the AUX function token/type, assigned input NAME and object ID, and preferred-storage request.
- `ros2_isobus/msg/VTAuxInputRaw`
  - Contains the normalized and raw input values plus learn, lock and interaction state.
- `ros2_isobus/msg/VTAuxStatus`
  - Summarizes AUX-N enable/learn state and the most recently routed input.

Runtime pool update notes
- `ISOBUS/vt/update` accepts:
  - one XML element fragment, or
  - full `<objectpool ...>` XML.
- You can update one or multiple objects in one call.
- You can also send multiple separate update calls.
- Recommended: include all related object changes in one XML update payload to minimize transfer/ack cycles.


Launch
- Recommended launch set:
```bash
ros2 launch ros2_isobus all_nodes_vt.launch.py
```
- Config file for this launch:
  - `config/all_nodes_vt_params.yaml`
