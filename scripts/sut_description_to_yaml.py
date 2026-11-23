#!/usr/bin/env python3
"""Transfer an AEF SUT Description XML device configuration to ROS 2 YAML."""

from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


class ConversionError(RuntimeError):
    """Invalid or unsupported SUT description input."""


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def child(element: ET.Element, name: str, required: bool = True) -> Optional[ET.Element]:
    for item in element:
        if local_name(item.tag) == name:
            return item
    if required:
        raise ConversionError(f"missing XML element {name} below {local_name(element.tag)}")
    return None


def children(element: ET.Element, name: str) -> Iterable[ET.Element]:
    return (item for item in element if local_name(item.tag) == name)


def text(element: ET.Element, name: str, default: Optional[str] = None) -> str:
    item = child(element, name, required=default is None)
    if item is None:
        return default or ""
    value = (item.text or "").strip()
    if not value and default is not None:
        return default
    return value


def integer(element: ET.Element, name: str, default: Optional[int] = None) -> int:
    raw = text(element, name, None if default is None else str(default))
    try:
        return int(raw, 0)
    except ValueError as exc:
        raise ConversionError(f"invalid integer in {name}: {raw!r}") from exc


def yaml_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, list):
        return json.dumps(value, ensure_ascii=False, separators=(", ", ": "))
    return str(value)


def name_to_yaml(name: str) -> str:
    clean = re.sub(r"^0x", "", name.strip(), flags=re.IGNORECASE)
    if not re.fullmatch(r"[0-9A-Fa-f]{16}", clean):
        raise ConversionError(f"NAME must contain exactly 8 bytes, got {name!r}")
    return bytes.fromhex(clean)[::-1].hex().upper()


def validate_name_fields(name: str, iso_data: ET.Element) -> None:
    clean = re.sub(r"^0x", "", name.strip(), flags=re.IGNORECASE)
    value = int(clean, 16)
    expected = {
        "IdentityNumber": value & 0x1FFFFF,
        "ManufacturerCode": (value >> 21) & 0x7FF,
        "ECU_Instance": (value >> 32) & 0x7,
        "FunctionInstance": (value >> 35) & 0x1F,
        "FunctionNumber": (value >> 40) & 0xFF,
        "ReservedBit": (value >> 48) & 0x1,
        "DeviceClassNumber": (value >> 49) & 0x7F,
        "DeviceClassInstance": (value >> 56) & 0xF,
        "IndustryGroup": (value >> 60) & 0x7,
        "SelfConfigurable": (value >> 63) & 0x1,
    }
    mismatches = []
    for field, decoded in expected.items():
        configured = integer(iso_data, field)
        if configured != decoded:
            mismatches.append(f"{field}: NAME={decoded}, XML={configured}")
    if mismatches:
        raise ConversionError("NAME does not match ISO11783-5_Data: " + "; ".join(mismatches))


def option_bytes(raw: str) -> str:
    clean = raw.strip()
    if clean in ("", "0", "0x", "0X"):
        return ""
    clean = re.sub(r"^0x", "", clean, flags=re.IGNORECASE)
    if len(clean) % 2 or not re.fullmatch(r"[0-9A-Fa-f]+", clean):
        raise ConversionError(f"invalid FunctionalityOptions value: {raw!r}")
    return ",".join(f"0x{clean[index:index + 2].upper()}" for index in range(0, len(clean), 2))


def select_device(devices: List[ET.Element], name: Optional[str], index: int) -> ET.Element:
    if name is not None:
        matches = [device for device in devices if text(device, "DeviceName") == name]
        if len(matches) != 1:
            raise ConversionError(f"device name {name!r} matched {len(matches)} devices")
        return matches[0]
    if index < 0 or index >= len(devices):
        raise ConversionError(f"device index {index} is outside 0..{len(devices) - 1}")
    return devices[index]


def parse_sut(xml_path: Path, device_name: Optional[str], device_index: int) -> Dict[str, Dict[str, Any]]:
    try:
        root = ET.parse(xml_path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise ConversionError(f"cannot parse {xml_path}: {exc}") from exc

    manufacturer = child(root, "ManufacturerSpecific")
    certification = child(root, "CertificationData")
    devices_parent = child(root, "Devices")
    devices = list(children(devices_parent, "Device"))
    if not devices:
        raise ConversionError("SUT description contains no Device elements")
    device = select_device(devices, device_name, device_index)

    name = text(device, "NAME")
    iso5 = child(device, "ISO11783-5_Data")
    iso12 = child(device, "ISO11783-12_Data")
    validate_name_fields(name, iso5)

    software_parent = child(iso12, "SoftwareIdentification", required=False)
    software = [] if software_parent is None else [
        (item.text or "").strip()
        for item in children(software_parent, "SoftwareIdentifier")
        if (item.text or "").strip()
    ]

    functionality_parent = child(device, "Functionalities", required=False)
    functionality_ids: List[int] = []
    functionality_generations: List[int] = []
    functionality_options: List[str] = []
    if functionality_parent is not None:
        for functionality in children(functionality_parent, "Functionality"):
            if integer(functionality, "Capability", 1) == 0:
                continue
            functionality_ids.append(integer(functionality, "FunctionalityCharacteristics"))
            functionality_generations.append(integer(functionality, "FunctionalityGeneration"))
            functionality_options.append(option_bytes(text(functionality, "FunctionalityOptions", "")))

    diagnostics: Dict[str, Any] = {
        "ecu_part_number": text(iso12, "ECU_PartNumber", ""),
        "ecu_serial_number": text(iso12, "ECU_SerialNumber", ""),
        "ecu_location": text(iso12, "ECU_Location", ""),
        "ecu_type": text(iso12, "ECU_Type", ""),
        "ecu_manufacturer_name": text(iso12, "ECU_MfgName", ""),
        "ecu_hardware_id": text(manufacturer, "HardwareVersion", ""),
        "software_ident_fields": software,
        "diagnostic_protocol_id": integer(iso12, "DiagnosticProtocolNum", 0),
        "compliance_test_lab_id": integer(certification, "LaboratoryId"),
        "compliance_cert_year": integer(certification, "Year"),
        "compliance_cert_revision": integer(certification, "Revision"),
        "compliance_lab_type": integer(certification, "LaboratoryType"),
        "compliance_reference_number": integer(certification, "ReferenceNumber"),
        "product_ident_code": text(manufacturer, "ProductIdentification", ""),
        "product_ident_brand": text(manufacturer, "Brand", ""),
        "product_ident_model": text(manufacturer, "Model", ""),
        "cf_functionality_ids": functionality_ids,
        "cf_functionality_generations": functionality_generations,
        "cf_functionality_options": functionality_options,
    }

    updates: Dict[str, Dict[str, Any]] = {
        "address_manager_node": {"ecu_name_hex": name_to_yaml(name)},
        "diagnostics_node": diagnostics,
    }

    tim = child(device, "TIM_Client_Data", required=False)
    if tim is not None:
        tim_updates: Dict[str, Any] = {
            "minimum_version": integer(tim, "TimMinVersion"),
            "implemented_version": integer(tim, "TimImplVersion"),
            "authlib.minimum_version": integer(tim, "TimMinAuthVersion"),
            "authlib.implemented_version": integer(tim, "TimImplAuthVersion"),
        }
        aux_selection = child(tim, "TimAuxValveTestSelected", required=False)
        if aux_selection is not None:
            selected = integer(tim, "TimAuxValveTestSelected", 0)
            tim_updates["tim.aux_fn_ids"] = [] if selected == 0 else [selected]
            tim_updates["tim.aux_ctrl_modes"] = [] if selected == 0 else [1]
        updates["tim_client_node"] = tim_updates

    return updates


def update_yaml(source: str, updates: Dict[str, Dict[str, Any]]) -> str:
    lines = source.splitlines()
    for node_name, node_updates in updates.items():
        node_start = next((i for i, line in enumerate(lines) if line == f"{node_name}:"), None)
        if node_start is None:
            raise ConversionError(f"YAML node section not found: {node_name}")
        node_end = next(
            (i for i in range(node_start + 1, len(lines)) if lines[i] and not lines[i].startswith((" ", "#"))),
            len(lines),
        )
        params_line = next(
            (i for i in range(node_start + 1, node_end) if lines[i].strip() == "ros__parameters:"),
            None,
        )
        if params_line is None:
            raise ConversionError(f"ros__parameters section not found below {node_name}")

        missing = dict(node_updates)
        key_pattern = re.compile(r"^(\s{4})([A-Za-z0-9_.-]+):(?:\s.*)?$")
        for index in range(params_line + 1, node_end):
            match = key_pattern.match(lines[index])
            if match and match.group(2) in missing:
                key = match.group(2)
                lines[index] = f"    {key}: {yaml_value(missing.pop(key))}"

        if missing:
            insertion = [f"    {key}: {yaml_value(value)}" for key, value in missing.items()]
            lines[node_end:node_end] = insertion

    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Transfer NAME and diagnostics data from an AEF SUT Description XML to ROS 2 YAML")
    parser.add_argument("sut_description", type=Path, help="AEF SUT Description XML")
    parser.add_argument(
        "--params-file", type=Path, default=Path("config/all_nodes_tim_params.yaml"),
        help="ROS 2 parameter YAML to update (default: config/all_nodes_tim_params.yaml)")
    selector = parser.add_mutually_exclusive_group()
    selector.add_argument("--device-name", help="select Device by DeviceName")
    selector.add_argument("--device-index", type=int, default=0, help="select Device by zero-based index")
    parser.add_argument("--output", type=Path, help="write to another YAML instead of updating --params-file")
    parser.add_argument("--dry-run", action="store_true", help="validate and print the generated YAML without writing")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        updates = parse_sut(args.sut_description, args.device_name, args.device_index)
        generated = update_yaml(args.params_file.read_text(encoding="utf-8"), updates)
        if args.dry_run:
            sys.stdout.write(generated)
            return 0
        output = args.output or args.params_file
        output.write_text(generated, encoding="utf-8")
        print(f"Updated {output}")
        print(f"Device NAME (ROS byte order): {updates['address_manager_node']['ecu_name_hex']}")
        return 0
    except (ConversionError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
