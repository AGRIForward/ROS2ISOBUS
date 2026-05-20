VT Client Interface Helper (`vt_client_interface.py`)
=====================================================

Purpose
- Provides high-level Python wrappers for nodes that communicate with `vt_client_node`.
- Hides raw ROS topic publish/subscribe details behind typed classes and methods.

Module
- `scripts/ros2_isobus/vt_client_interface.py`
- Import path after build/install: `from ros2_isobus import ...`

Classes
- `VTContainer`
  - Container visibility control only.
  - Topics: `ISOBUS/vt/container/<name_token>/visible/(set|value|result)`.
  - Methods: `set_visible(bool)`, `visible()`, `visible_result()`.
- `VTNumber`
  - Number value set/get via `ISOBUS/vt/number/<name_token>/(set|value)`.
- `VTInputNumber`
  - Input-number value set/get via `ISOBUS/vt/input_number/<name_token>/(set|value)`.
  - Enable control: `ISOBUS/vt/input_number/<name_token>/enabled/(set|value|result)`.
  - Methods: `set(float)`, `value()`, `on_change(...)`, `set_enabled(bool)`, `enabled()`, `enabled_result()`.
- `VTString`
  - String value set/get via `ISOBUS/vt/string/<name_token>/(set|value)`.
- `VTInputString`
  - Input-string value set/get via `ISOBUS/vt/input_string/<name_token>/(set|value)`.
  - Enable control: `ISOBUS/vt/input_string/<name_token>/enabled/(set|value|result)`.
- `VTInputBoolean`
  - Input-boolean value set/get via `ISOBUS/vt/input_bool/<name_token>/(set|value)`.
  - Enable control: `ISOBUS/vt/input_bool/<name_token>/enabled/(set|value|result)`.
- `VTList`
  - List index set/get via `ISOBUS/vt/list/<name_token>/(set|value)`.
- `VTButton`
  - Event input via `ISOBUS/vt/event/button/<name_token>`.
  - Enable control: `ISOBUS/vt/button/<name_token>/enabled/(set|value|result)`.
  - Callback registration:
    - `on_event(callback)` receives raw key activation code (`0/1/2`).
    - `on_pressed(callback)` for press only (`code=1`).
    - `on_released(callback)` for release only (`code=0`).
- `VTSoftkey`
  - Event input via `ISOBUS/vt/event/softkey/<name_token>`.
  - Same callback API as `VTButton`.
- `VTAuxFunction[TValue]`
  - AUX function streams:
    - `ISOBUS/vt/aux/<name_token>/input/value`
    - `ISOBUS/vt/aux/<name_token>/input/raw`
    - `ISOBUS/vt/aux/<name_token>/assignment/value|result`
- `VTWorkingSet`
  - Working-set level communication:
    - active/softkey mask triggers
    - status, session state
    - pointing/navigation events
    - diagnostics
    - runtime pool updates (`send_pool_update_xml(...)`)
    - runtime pool update result callback (`on_updated(...)`)

Pool Creation And XML Usage
1. Create/edit the VT object pool with PoolEdit.
2. Export XML from PoolEdit (`Export XML`).
3. Configure that XML path as `vt_client_node` parameter `xml_file`.
4. Use XML `name` values as wrapper `name_token` constructor arguments.

Usage Example
```python
import rclpy
from rclpy.node import Node

from ros2_isobus import VTButton, VTContainer, VTString, VTWorkingSet


class MyVtNode(Node):
    def __init__(self) -> None:
        super().__init__("my_vt_node")
        self.label = VTString(self, "label0")
        self.panel = VTContainer(self, "main_panel")
        self.ok_button = VTButton(self, "ok_button")
        self.ws = VTWorkingSet(self)

        self.ok_button.on_pressed(self._on_ok_pressed)
        self.ok_button.on_event(self._on_ok_event_code)
        self.ws.on_updated(self._on_pool_update_result)

        self.ws.trigger_active_mask("main_mask")

    def _on_ok_pressed(self) -> None:
        self.label.set("OK")
        self.panel.set_visible(True)

    def _on_ok_event_code(self, code: int) -> None:
        self.get_logger().info(f"ok_button event code={code}")

    def _on_pool_update_result(self, result) -> None:
        self.get_logger().info(
            f"update: success={result.success} detail={result.detail} "
            f"pending={result.pending_updates} in_progress={result.in_progress_updates}"
        )


def main() -> None:
    rclpy.init()
    node = MyVtNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

Notes
- Each object class instance is bound to one XML `name_token`.
- This helper assumes `vt_client_node` is running and has created matching topics from loaded XML.
- `VTWorkingSet.send_pool_update_xml(...)` can update one or many objects in one XML payload.
