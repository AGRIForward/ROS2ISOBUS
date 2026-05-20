from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Callable, Dict, Generic, List, Optional, TypeVar

from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from ros2_isobus.msg import (
    CommandResult,
    VTAuxAssignment,
    VTAuxInputRaw,
    VTAuxStatus,
    VTNavigationEvent,
    VTPointingEvent,
    VTSessionState,
    VTStatus,
    VTUpdateResult,
)
from std_msgs.msg import Bool, Empty, Float64, Int32, String, UInt8


class VTContainer:
    """Wrap container visibility control.

    Topics:
    - publish:   ISOBUS/vt/container/<name_token>/visible/set
    - subscribe: ISOBUS/vt/container/<name_token>/visible/value|result
    """

    def __init__(self, node: Node, name_token: str) -> None:
        self._node = node
        self._name_token = name_token
        self._base_topic = f"ISOBUS/vt/container/{name_token}"
        self._lock = Lock()
        self._visible = False
        self._visible_result = CommandResult()

        self._visible_set_pub = node.create_publisher(Bool, f"{self._base_topic}/visible/set", 10)
        self._visible_value_sub = node.create_subscription(
            Bool, f"{self._base_topic}/visible/value", self._on_visible_value, 10
        )
        self._visible_result_sub = node.create_subscription(
            CommandResult, f"{self._base_topic}/visible/result", self._on_visible_result, 10
        )

    def name_token(self) -> str:
        """Return XML token used in topic paths."""
        return self._name_token

    def set_visible(self, value: bool) -> None:
        """Publish visible set request."""
        self._visible_set_pub.publish(Bool(data=value))

    def visible(self) -> bool:
        """Return latest visible state from VT value topic."""
        with self._lock:
            return self._visible

    def visible_result(self) -> CommandResult:
        """Return latest command result for visible set request."""
        with self._lock:
            return self._visible_result

    def _on_visible_value(self, msg: Bool) -> None:
        with self._lock:
            self._visible = msg.data

    def _on_visible_result(self, msg: CommandResult) -> None:
        with self._lock:
            self._visible_result = msg


class VTNumber:
    """Wrap NumberVariable value I/O.

    Topics:
    - publish:   ISOBUS/vt/number/<name_token>/set
    - subscribe: ISOBUS/vt/number/<name_token>/value
    """

    def __init__(self, node: Node, name_token: str) -> None:
        self._node = node
        self._name_token = name_token
        self._lock = Lock()
        self._value = 0.0
        self._callbacks: List[Callable[[float], None]] = []
        self._set_pub = node.create_publisher(Float64, f"ISOBUS/vt/number/{name_token}/set", 10)
        self._value_sub = node.create_subscription(
            Float64, f"ISOBUS/vt/number/{name_token}/value", self._on_value, 10
        )

    def name_token(self) -> str:
        return self._name_token

    def set(self, value: float) -> None:
        self._set_pub.publish(Float64(data=value))

    def value(self) -> float:
        with self._lock:
            return self._value

    def on_change(self, callback: Callable[[float], None]) -> None:
        with self._lock:
            self._callbacks.append(callback)

    def _on_value(self, msg: Float64) -> None:
        with self._lock:
            self._value = msg.data
            callbacks = list(self._callbacks)
            value = self._value
        for cb in callbacks:
            cb(value)


class VTInputNumber:
    """Wrap Input Number value and enabled control."""

    def __init__(self, node: Node, name_token: str) -> None:
        self._node = node
        self._name_token = name_token
        self._lock = Lock()
        self._value = 0.0
        self._enabled = False
        self._enabled_result = CommandResult()
        self._callbacks: List[Callable[[float], None]] = []

        base = f"ISOBUS/vt/input_number/{name_token}"
        self._set_pub = node.create_publisher(Float64, f"{base}/set", 10)
        self._value_sub = node.create_subscription(Float64, f"{base}/value", self._on_value, 10)
        self._enabled_set_pub = node.create_publisher(Bool, f"{base}/enabled/set", 10)
        self._enabled_value_sub = node.create_subscription(Bool, f"{base}/enabled/value", self._on_enabled, 10)
        self._enabled_result_sub = node.create_subscription(
            CommandResult, f"{base}/enabled/result", self._on_enabled_result, 10
        )

    def name_token(self) -> str:
        return self._name_token

    def set(self, value: float) -> None:
        self._set_pub.publish(Float64(data=value))

    def value(self) -> float:
        with self._lock:
            return self._value

    def on_change(self, callback: Callable[[float], None]) -> None:
        with self._lock:
            self._callbacks.append(callback)

    def set_enabled(self, value: bool) -> None:
        self._enabled_set_pub.publish(Bool(data=value))

    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def enabled_result(self) -> CommandResult:
        with self._lock:
            return self._enabled_result

    def _on_value(self, msg: Float64) -> None:
        with self._lock:
            self._value = msg.data
            callbacks = list(self._callbacks)
            value = self._value
        for cb in callbacks:
            cb(value)

    def _on_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._enabled = msg.data

    def _on_enabled_result(self, msg: CommandResult) -> None:
        with self._lock:
            self._enabled_result = msg


class VTString:
    """Wrap StringVariable value I/O."""

    def __init__(self, node: Node, name_token: str) -> None:
        self._node = node
        self._name_token = name_token
        self._lock = Lock()
        self._value = ""
        self._callbacks: List[Callable[[str], None]] = []
        self._set_pub = node.create_publisher(String, f"ISOBUS/vt/string/{name_token}/set", 10)
        self._value_sub = node.create_subscription(String, f"ISOBUS/vt/string/{name_token}/value", self._on_value, 10)

    def name_token(self) -> str:
        return self._name_token

    def set(self, value: str) -> None:
        self._set_pub.publish(String(data=value))

    def value(self) -> str:
        with self._lock:
            return self._value

    def on_change(self, callback: Callable[[str], None]) -> None:
        with self._lock:
            self._callbacks.append(callback)

    def _on_value(self, msg: String) -> None:
        with self._lock:
            self._value = msg.data
            callbacks = list(self._callbacks)
            value = self._value
        for cb in callbacks:
            cb(value)


class VTInputString:
    """Wrap Input String value and enabled control."""

    def __init__(self, node: Node, name_token: str) -> None:
        self._node = node
        self._name_token = name_token
        self._lock = Lock()
        self._value = ""
        self._enabled = False
        self._enabled_result = CommandResult()
        self._callbacks: List[Callable[[str], None]] = []

        base = f"ISOBUS/vt/input_string/{name_token}"
        self._set_pub = node.create_publisher(String, f"{base}/set", 10)
        self._value_sub = node.create_subscription(String, f"{base}/value", self._on_value, 10)
        self._enabled_set_pub = node.create_publisher(Bool, f"{base}/enabled/set", 10)
        self._enabled_value_sub = node.create_subscription(Bool, f"{base}/enabled/value", self._on_enabled, 10)
        self._enabled_result_sub = node.create_subscription(
            CommandResult, f"{base}/enabled/result", self._on_enabled_result, 10
        )

    def name_token(self) -> str:
        return self._name_token

    def set(self, value: str) -> None:
        self._set_pub.publish(String(data=value))

    def value(self) -> str:
        with self._lock:
            return self._value

    def on_change(self, callback: Callable[[str], None]) -> None:
        with self._lock:
            self._callbacks.append(callback)

    def set_enabled(self, value: bool) -> None:
        self._enabled_set_pub.publish(Bool(data=value))

    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def enabled_result(self) -> CommandResult:
        with self._lock:
            return self._enabled_result

    def _on_value(self, msg: String) -> None:
        with self._lock:
            self._value = msg.data
            callbacks = list(self._callbacks)
            value = self._value
        for cb in callbacks:
            cb(value)

    def _on_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._enabled = msg.data

    def _on_enabled_result(self, msg: CommandResult) -> None:
        with self._lock:
            self._enabled_result = msg


class VTInputBoolean:
    """Wrap Input Boolean value and enabled control."""

    def __init__(self, node: Node, name_token: str) -> None:
        self._node = node
        self._name_token = name_token
        self._lock = Lock()
        self._value = False
        self._enabled = False
        self._enabled_result = CommandResult()
        self._callbacks: List[Callable[[bool], None]] = []

        base = f"ISOBUS/vt/input_bool/{name_token}"
        self._set_pub = node.create_publisher(Bool, f"{base}/set", 10)
        self._value_sub = node.create_subscription(Bool, f"{base}/value", self._on_value, 10)
        self._enabled_set_pub = node.create_publisher(Bool, f"{base}/enabled/set", 10)
        self._enabled_value_sub = node.create_subscription(Bool, f"{base}/enabled/value", self._on_enabled, 10)
        self._enabled_result_sub = node.create_subscription(
            CommandResult, f"{base}/enabled/result", self._on_enabled_result, 10
        )

    def name_token(self) -> str:
        return self._name_token

    def set(self, value: bool) -> None:
        self._set_pub.publish(Bool(data=value))

    def value(self) -> bool:
        with self._lock:
            return self._value

    def on_change(self, callback: Callable[[bool], None]) -> None:
        with self._lock:
            self._callbacks.append(callback)

    def set_enabled(self, value: bool) -> None:
        self._enabled_set_pub.publish(Bool(data=value))

    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def enabled_result(self) -> CommandResult:
        with self._lock:
            return self._enabled_result

    def _on_value(self, msg: Bool) -> None:
        with self._lock:
            self._value = msg.data
            callbacks = list(self._callbacks)
            value = self._value
        for cb in callbacks:
            cb(value)

    def _on_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._enabled = msg.data

    def _on_enabled_result(self, msg: CommandResult) -> None:
        with self._lock:
            self._enabled_result = msg


class VTList:
    """Wrap list index value I/O."""

    def __init__(self, node: Node, name_token: str) -> None:
        self._node = node
        self._name_token = name_token
        self._lock = Lock()
        self._value = 0
        self._set_pub = node.create_publisher(Int32, f"ISOBUS/vt/list/{name_token}/set", 10)
        self._value_sub = node.create_subscription(Int32, f"ISOBUS/vt/list/{name_token}/value", self._on_value, 10)

    def name_token(self) -> str:
        return self._name_token

    def set(self, value: int) -> None:
        self._set_pub.publish(Int32(data=int(value)))

    def value(self) -> int:
        with self._lock:
            return self._value

    def _on_value(self, msg: Int32) -> None:
        with self._lock:
            self._value = msg.data


class VTButton:
    """Wrap button event stream and enabled control."""

    def __init__(self, node: Node, name_token: str) -> None:
        self._node = node
        self._name_token = name_token
        self._lock = Lock()
        self._pressed = False
        self._enabled = False
        self._enabled_result = CommandResult()
        self._on_event_callbacks: List[Callable[[int], None]] = []
        self._on_pressed_callbacks: List[Callable[[], None]] = []
        self._on_released_callbacks: List[Callable[[], None]] = []

        self._event_sub = node.create_subscription(UInt8, f"ISOBUS/vt/event/button/{name_token}", self._on_event, 10)
        base = f"ISOBUS/vt/button/{name_token}/enabled"
        self._enabled_set_pub = node.create_publisher(Bool, f"{base}/set", 10)
        self._enabled_value_sub = node.create_subscription(Bool, f"{base}/value", self._on_enabled, 10)
        self._enabled_result_sub = node.create_subscription(CommandResult, f"{base}/result", self._on_enabled_result, 10)

    def name_token(self) -> str:
        return self._name_token

    def on_event(self, callback: Callable[[int], None]) -> None:
        """Register callback for every activation code event (0/1/2)."""
        with self._lock:
            self._on_event_callbacks.append(callback)

    def on_pressed(self, callback: Callable[[], None]) -> None:
        """Register callback for activation code 0x01."""
        with self._lock:
            self._on_pressed_callbacks.append(callback)

    def on_released(self, callback: Callable[[], None]) -> None:
        """Register callback for activation code 0x00."""
        with self._lock:
            self._on_released_callbacks.append(callback)

    def pressed(self) -> bool:
        with self._lock:
            return self._pressed

    def set_enabled(self, value: bool) -> None:
        self._enabled_set_pub.publish(Bool(data=value))

    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def enabled_result(self) -> CommandResult:
        with self._lock:
            return self._enabled_result

    def _on_event(self, msg: UInt8) -> None:
        code = int(msg.data)
        with self._lock:
            self._pressed = (code != 0)
            all_callbacks = list(self._on_event_callbacks)
            pressed_callbacks = list(self._on_pressed_callbacks) if code == 1 else []
            released_callbacks = list(self._on_released_callbacks) if code == 0 else []
        for cb in all_callbacks:
            cb(code)
        for cb in pressed_callbacks:
            cb()
        for cb in released_callbacks:
            cb()

    def _on_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._enabled = msg.data

    def _on_enabled_result(self, msg: CommandResult) -> None:
        with self._lock:
            self._enabled_result = msg


class VTSoftkey:
    """Wrap softkey event stream."""

    def __init__(self, node: Node, name_token: str) -> None:
        self._node = node
        self._name_token = name_token
        self._lock = Lock()
        self._pressed = False
        self._on_event_callbacks: List[Callable[[int], None]] = []
        self._on_pressed_callbacks: List[Callable[[], None]] = []
        self._on_released_callbacks: List[Callable[[], None]] = []

        self._event_sub = node.create_subscription(UInt8, f"ISOBUS/vt/event/softkey/{name_token}", self._on_event, 10)

    def name_token(self) -> str:
        return self._name_token

    def on_event(self, callback: Callable[[int], None]) -> None:
        with self._lock:
            self._on_event_callbacks.append(callback)

    def on_pressed(self, callback: Callable[[], None]) -> None:
        with self._lock:
            self._on_pressed_callbacks.append(callback)

    def on_released(self, callback: Callable[[], None]) -> None:
        with self._lock:
            self._on_released_callbacks.append(callback)

    def pressed(self) -> bool:
        with self._lock:
            return self._pressed

    def _on_event(self, msg: UInt8) -> None:
        code = int(msg.data)
        with self._lock:
            self._pressed = (code != 0)
            all_callbacks = list(self._on_event_callbacks)
            pressed_callbacks = list(self._on_pressed_callbacks) if code == 1 else []
            released_callbacks = list(self._on_released_callbacks) if code == 0 else []
        for cb in all_callbacks:
            cb(code)
        for cb in pressed_callbacks:
            cb()
        for cb in released_callbacks:
            cb()


TValue = TypeVar("TValue", bool, int, float)


class VTAuxFunction(Generic[TValue]):
    """Wrap AUX function topics for one token."""

    def __init__(self, node: Node, name_token: str, value_type: type = float) -> None:
        self._node = node
        self._name_token = name_token
        self._value_type = value_type
        self._base_topic = f"ISOBUS/vt/aux/{name_token}"
        self._lock = Lock()
        self._input_value: TValue = self._convert_value(0.0)
        self._input_value_raw = 0.0
        self._input_raw = VTAuxInputRaw()
        self._assignment = VTAuxAssignment()
        self._assignment_result = CommandResult()

        self._on_input_callbacks: List[Callable[[TValue], None]] = []
        self._on_input_raw_callbacks: List[Callable[[VTAuxInputRaw], None]] = []
        self._on_assignment_callbacks: List[Callable[[VTAuxAssignment], None]] = []
        self._on_assignment_result_callbacks: List[Callable[[CommandResult], None]] = []

        self._input_value_sub = node.create_subscription(Float64, f"{self._base_topic}/input/value", self._on_input_value, 10)
        self._input_raw_sub = node.create_subscription(VTAuxInputRaw, f"{self._base_topic}/input/raw", self._on_input_raw, 10)
        self._assignment_value_sub = node.create_subscription(
            VTAuxAssignment, f"{self._base_topic}/assignment/value", self._on_assignment, 10
        )
        self._assignment_result_sub = node.create_subscription(
            CommandResult, f"{self._base_topic}/assignment/result", self._on_assignment_result, 10
        )

    def name_token(self) -> str:
        return self._name_token

    def input_value(self) -> TValue:
        with self._lock:
            return self._input_value

    def input_value_raw(self) -> float:
        with self._lock:
            return self._input_value_raw

    def input_raw(self) -> VTAuxInputRaw:
        with self._lock:
            return self._input_raw

    def assignment(self) -> VTAuxAssignment:
        with self._lock:
            return self._assignment

    def assignment_result(self) -> CommandResult:
        with self._lock:
            return self._assignment_result

    def on_input(self, callback: Callable[[TValue], None]) -> None:
        with self._lock:
            self._on_input_callbacks.append(callback)

    def on_input_raw(self, callback: Callable[[VTAuxInputRaw], None]) -> None:
        with self._lock:
            self._on_input_raw_callbacks.append(callback)

    def on_assignment(self, callback: Callable[[VTAuxAssignment], None]) -> None:
        with self._lock:
            self._on_assignment_callbacks.append(callback)

    def on_assignment_result(self, callback: Callable[[CommandResult], None]) -> None:
        with self._lock:
            self._on_assignment_result_callbacks.append(callback)

    def _convert_value(self, raw: float) -> TValue:
        if self._value_type is bool:
            return bool(raw)  # type: ignore[return-value]
        if self._value_type is int:
            return int(raw)  # type: ignore[return-value]
        return float(raw)  # type: ignore[return-value]

    def _on_input_value(self, msg: Float64) -> None:
        typed = self._convert_value(msg.data)
        with self._lock:
            self._input_value = typed
            self._input_value_raw = msg.data
            callbacks = list(self._on_input_callbacks)
        for cb in callbacks:
            cb(typed)

    def _on_input_raw(self, msg: VTAuxInputRaw) -> None:
        with self._lock:
            self._input_raw = msg
            callbacks = list(self._on_input_raw_callbacks)
        for cb in callbacks:
            cb(msg)

    def _on_assignment(self, msg: VTAuxAssignment) -> None:
        with self._lock:
            self._assignment = msg
            callbacks = list(self._on_assignment_callbacks)
        for cb in callbacks:
            cb(msg)

    def _on_assignment_result(self, msg: CommandResult) -> None:
        with self._lock:
            self._assignment_result = msg
            callbacks = list(self._on_assignment_result_callbacks)
        for cb in callbacks:
            cb(msg)


class VTWorkingSet:
    """Wrap VT working-set level status, events and control topics."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._lock = Lock()

        self._status = VTStatus()
        self._state = VTSessionState()
        self._diagnostics = DiagnosticArray()
        self._pointing = VTPointingEvent()
        self._navigation = VTNavigationEvent()
        self._update_result = VTUpdateResult()
        self._aux_status = VTAuxStatus()

        self._status_callbacks: List[Callable[[VTStatus], None]] = []
        self._status_timeout_callbacks: List[Callable[[VTStatus], None]] = []
        self._state_callbacks: List[Callable[[VTSessionState], None]] = []
        self._diagnostics_callbacks: List[Callable[[DiagnosticArray], None]] = []
        self._pointing_callbacks: List[Callable[[VTPointingEvent], None]] = []
        self._navigation_callbacks: List[Callable[[VTNavigationEvent], None]] = []
        self._updated_callbacks: List[Callable[[VTUpdateResult], None]] = []
        self._aux_status_callbacks: List[Callable[[VTAuxStatus], None]] = []

        self._named_empty_set_pubs: Dict[str, object] = {}
        self._update_pub = None

        self._status_sub = node.create_subscription(VTStatus, "ISOBUS/vt/status", self._on_status, 10)
        self._state_sub = node.create_subscription(VTSessionState, "ISOBUS/vt/session/state", self._on_state, 10)
        self._diagnostics_sub = node.create_subscription(DiagnosticArray, "ISOBUS/vt/diagnostics", self._on_diagnostics, 10)
        self._pointing_sub = node.create_subscription(VTPointingEvent, "ISOBUS/vt/event/pointing", self._on_pointing, 10)
        self._navigation_sub = node.create_subscription(VTNavigationEvent, "ISOBUS/vt/event/navigation", self._on_navigation, 10)
        self._update_result_sub = node.create_subscription(VTUpdateResult, "ISOBUS/vt/update_result", self._on_updated, 10)
        self._aux_status_sub = node.create_subscription(VTAuxStatus, "ISOBUS/vt/aux/status", self._on_aux_status, 10)

    def trigger_active_mask(self, mask_name: str) -> None:
        self._get_empty_publisher(f"ISOBUS/vt/active_mask/{mask_name}/set").publish(Empty())

    def trigger_softkey_mask(self, mask_name: str) -> None:
        self._get_empty_publisher(f"ISOBUS/vt/softkey_mask/{mask_name}/set").publish(Empty())

    def send_pool_update_xml(self, xml: str) -> None:
        self._get_update_publisher().publish(String(data=xml))

    def status(self) -> VTStatus:
        with self._lock:
            return self._status

    def state(self) -> VTSessionState:
        with self._lock:
            return self._state

    def diagnostics(self) -> DiagnosticArray:
        with self._lock:
            return self._diagnostics

    def pointing(self) -> VTPointingEvent:
        with self._lock:
            return self._pointing

    def navigation(self) -> VTNavigationEvent:
        with self._lock:
            return self._navigation

    def update_result(self) -> VTUpdateResult:
        with self._lock:
            return self._update_result

    def aux_status(self) -> VTAuxStatus:
        with self._lock:
            return self._aux_status

    def on_status(self, callback: Callable[[VTStatus], None]) -> None:
        with self._lock:
            self._status_callbacks.append(callback)

    def on_status_timeout(self, callback: Callable[[VTStatus], None]) -> None:
        with self._lock:
            self._status_timeout_callbacks.append(callback)

    def on_state(self, callback: Callable[[VTSessionState], None]) -> None:
        with self._lock:
            self._state_callbacks.append(callback)

    def on_diagnostics(self, callback: Callable[[DiagnosticArray], None]) -> None:
        with self._lock:
            self._diagnostics_callbacks.append(callback)

    def on_pointing(self, callback: Callable[[VTPointingEvent], None]) -> None:
        with self._lock:
            self._pointing_callbacks.append(callback)

    def on_navigation(self, callback: Callable[[VTNavigationEvent], None]) -> None:
        with self._lock:
            self._navigation_callbacks.append(callback)

    def on_updated(self, callback: Callable[[VTUpdateResult], None]) -> None:
        with self._lock:
            self._updated_callbacks.append(callback)

    def on_aux_status(self, callback: Callable[[VTAuxStatus], None]) -> None:
        with self._lock:
            self._aux_status_callbacks.append(callback)

    def _get_empty_publisher(self, topic: str):
        pub = self._named_empty_set_pubs.get(topic)
        if pub is None:
            pub = self._node.create_publisher(Empty, topic, 10)
            self._named_empty_set_pubs[topic] = pub
        return pub

    def _get_update_publisher(self):
        if self._update_pub is None:
            self._update_pub = self._node.create_publisher(String, "ISOBUS/vt/update", 10)
        return self._update_pub

    def _on_status(self, msg: VTStatus) -> None:
        with self._lock:
            self._status = msg
            callbacks = list(self._status_callbacks)
            timeout_callbacks = list(self._status_timeout_callbacks) if msg.status_timeout else []
        for cb in callbacks:
            cb(msg)
        for cb in timeout_callbacks:
            cb(msg)

    def _on_state(self, msg: VTSessionState) -> None:
        with self._lock:
            self._state = msg
            callbacks = list(self._state_callbacks)
        for cb in callbacks:
            cb(msg)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        with self._lock:
            self._diagnostics = msg
            callbacks = list(self._diagnostics_callbacks)
        for cb in callbacks:
            cb(msg)

    def _on_pointing(self, msg: VTPointingEvent) -> None:
        with self._lock:
            self._pointing = msg
            callbacks = list(self._pointing_callbacks)
        for cb in callbacks:
            cb(msg)

    def _on_navigation(self, msg: VTNavigationEvent) -> None:
        with self._lock:
            self._navigation = msg
            callbacks = list(self._navigation_callbacks)
        for cb in callbacks:
            cb(msg)

    def _on_updated(self, msg: VTUpdateResult) -> None:
        with self._lock:
            self._update_result = msg
            callbacks = list(self._updated_callbacks)
        for cb in callbacks:
            cb(msg)

    def _on_aux_status(self, msg: VTAuxStatus) -> None:
        with self._lock:
            self._aux_status = msg
            callbacks = list(self._aux_status_callbacks)
        for cb in callbacks:
            cb(msg)


__all__ = [
    "VTContainer",
    "VTNumber",
    "VTInputNumber",
    "VTString",
    "VTInputString",
    "VTInputBoolean",
    "VTList",
    "VTButton",
    "VTSoftkey",
    "VTAuxFunction",
    "VTWorkingSet",
]
