#!/usr/bin/env python3
"""TIM Auth compatibility checker for Ros2ISOBUS.

Validation utility for TIM AuthLib certificate/key configuration.

Checks (core):
- required parameter presence
- certificate file presence and parseability
- certificate chain verification (root -> testlab -> manufacturer -> series -> device)
- X25519 private key <-> device certificate public key match

Checks (guideline/interop):
- issuer O/OU token decoding (AEF 040 style)
- ISOBUS NAME consistency against certificate metadata
- conformance field consistency against certificate OU tokens
- best-effort requested function coverage against pseudonym metadata

Exit codes:
- 0: all checks passed
- 1: one or more checks failed
- 2: fatal runtime/usage error (missing params file/tooling etc.)
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


REQUIRED_PARAM_KEYS = [
    "authlib.root_cert_path",
    "authlib.client_testlab_cert_path",
    "authlib.client_manufacturer_cert_path",
    "authlib.client_series_cert_path",
    "authlib.client_device_cert_path",
    "authlib.client_private_key_hex",
]

INFO_PARAM_KEYS = [
    "authlib.strict",
    "authlib.max_slice_iterations",
    "authlib.client_cert_payload_max_len",
    "compliance_test_lab_id",
    "compliance_cert_year",
    "compliance_cert_revision",
    "compliance_lab_type",
    "compliance_reference_number",
    "ecu_name_hex",
    "tim.enable_speed",
    "tim.enable_curvature",
    "tim.enable_rear_pto",
    "tim.enable_rear_hitch",
    "tim.aux_fn_ids",
]


class FatalCheckError(RuntimeError):
    """Fatal runtime error for checker execution."""


@dataclass
class CheckResult:
    ok: bool
    name: str
    details: str


class Reporter:
    def __init__(self) -> None:
        self.results: List[CheckResult] = []

    def add(self, ok: bool, name: str, details: str) -> None:
        self.results.append(CheckResult(ok=ok, name=name, details=details))

    def failed(self) -> int:
        return sum(1 for item in self.results if not item.ok)

    def passed(self) -> int:
        return len(self.results) - self.failed()

    def render_text(self, include_notes: bool = True) -> None:
        print("TIM Auth Compatibility Report")
        print("=" * 31)
        print()
        for item in self.results:
            tag = "PASS" if item.ok else "FAIL"
            print(f"[{tag}] {item.name}")
            print(f"       {item.details}")
            print()
        print(f"Summary: {self.passed()} passed, {self.failed()} failed.")
        if include_notes:
            print()
            print("Notes:")
            print("- Chain verify failures often mean trust-store mismatch between client and server.")
            print("- Private key mismatch means signed-challenge will fail even if cert chain parses.")
            print("- NAME/conformance checks follow AEF 040 O/OU token interpretation.")

    def render_json(self) -> None:
        payload = {
            "summary": {
                "passed": self.passed(),
                "failed": self.failed(),
                "total": len(self.results),
            },
            "checks": [asdict(item) for item in self.results],
        }
        print(json.dumps(payload, indent=2, sort_keys=True))


def run_cmd(args: List[str], input_bytes: Optional[bytes] = None) -> Tuple[int, str, str]:
    proc = subprocess.run(
        args,
        input=input_bytes,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    return (
        proc.returncode,
        proc.stdout.decode("utf-8", errors="replace"),
        proc.stderr.decode("utf-8", errors="replace"),
    )


def _flatten_ros_params_dict(data: Dict[str, Any], node_name: str) -> Dict[str, str]:
    """Flatten ROS2 params YAML to key/value map.

    Supports:
    - node section style: <node_name>.ros__parameters.{...}
    - flat key style: key: value
    """
    out: Dict[str, str] = {}

    def to_str(v: Any) -> str:
        if isinstance(v, bool):
            return "true" if v else "false"
        if isinstance(v, (int, float)):
            return str(v)
        if isinstance(v, list):
            return json.dumps(v)
        return str(v)

    node_section = data.get(node_name)
    if isinstance(node_section, dict):
        ros_params = node_section.get("ros__parameters")
        if isinstance(ros_params, dict):
            for k, v in ros_params.items():
                out[str(k)] = to_str(v)

    if not out:
        for k, v in data.items():
            if isinstance(v, (dict, list)):
                continue
            out[str(k)] = to_str(v)

    return out


def parse_simple_yaml(params_file: Path, node_name: str) -> Dict[str, str]:
    """Fallback parser for simple key: value lines if PyYAML is unavailable."""
    values: Dict[str, str] = {}

    node_context = False
    ros_params_context = False
    node_indent = 0
    ros_indent = 0

    for line in params_file.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.strip() or line.strip().startswith("#"):
            continue

        indent = len(line) - len(line.lstrip(" "))
        stripped = line.strip()

        if stripped == f"{node_name}:":
            node_context = True
            ros_params_context = False
            node_indent = indent
            continue

        if node_context and indent <= node_indent and stripped.endswith(":"):
            node_context = False
            ros_params_context = False

        if node_context and stripped == "ros__parameters:":
            ros_params_context = True
            ros_indent = indent
            continue

        if ros_params_context and indent <= ros_indent and stripped.endswith(":"):
            ros_params_context = False

        m = re.match(r"^([A-Za-z0-9_.-]+):\s*(.+?)\s*$", stripped)
        if not m:
            continue

        key, raw_value = m.group(1), m.group(2)
        if raw_value.startswith("#"):
            continue
        if " #" in raw_value:
            raw_value = raw_value.split(" #", 1)[0].rstrip()
        if (raw_value.startswith('"') and raw_value.endswith('"')) or (
            raw_value.startswith("'") and raw_value.endswith("'")
        ):
            raw_value = raw_value[1:-1]

        if ros_params_context:
            values[key] = raw_value
        elif not node_context and not ros_params_context:
            # Flat file style fallback.
            values[key] = raw_value

    return values


def load_params(params_file: Path, node_name: str) -> Dict[str, str]:
    try:
        import yaml  # type: ignore

        raw = yaml.safe_load(params_file.read_text(encoding="utf-8", errors="replace"))
        if isinstance(raw, dict):
            return _flatten_ros_params_dict(raw, node_name)
    except Exception:
        pass

    return parse_simple_yaml(params_file, node_name)


def resolve_path(path_str: str, repo_root: Path) -> Optional[Path]:
    p = Path(path_str).expanduser()
    if p.is_absolute() and p.exists():
        return p

    candidates = [
        repo_root / path_str,
        repo_root / "install/ros2_isobus/share/ros2_isobus" / path_str,
        repo_root / "src/Ros2ISOBUS/install/ros2_isobus/share/ros2_isobus" / path_str,
    ]
    for c in candidates:
        if c.exists():
            return c
    return None


def der_to_pem(der_path: Path, out_pem: Path) -> Tuple[bool, str]:
    rc, _, err = run_cmd(["openssl", "x509", "-inform", "DER", "-in", str(der_path), "-out", str(out_pem)])
    return rc == 0, err.strip()


def crl_der_to_pem(der_path: Path, out_pem: Path) -> Tuple[bool, str]:
    rc, _, err = run_cmd(["openssl", "crl", "-inform", "DER", "-in", str(der_path), "-out", str(out_pem)])
    return rc == 0, err.strip()


def get_cert_subject_issuer(der_path: Path) -> Tuple[bool, str]:
    rc, out, err = run_cmd(
        [
            "openssl",
            "x509",
            "-inform",
            "DER",
            "-in",
            str(der_path),
            "-noout",
            "-subject",
            "-issuer",
            "-serial",
            "-nameopt",
            "RFC2253",
        ]
    )
    return (rc == 0), out if rc == 0 else err


def chain_verify(root_pem: Path, chain_pems: List[Path], leaf_pem: Path, crl_pem: Optional[Path]) -> Tuple[bool, str]:
    cmd = ["openssl", "verify", "-CAfile", str(root_pem)]

    combined_path: Optional[Path] = None
    if chain_pems:
        combined = tempfile.NamedTemporaryFile(mode="w", suffix=".pem", delete=False)
        try:
            combined_path = Path(combined.name)
            for p in chain_pems:
                text = p.read_text(encoding="utf-8", errors="replace")
                combined.write(text)
                if not text.endswith("\n"):
                    combined.write("\n")
            combined.flush()
        finally:
            combined.close()
        cmd.extend(["-untrusted", str(combined_path)])

    if crl_pem is not None:
        cmd.extend(["-crl_check", "-CRLfile", str(crl_pem)])

    cmd.append(str(leaf_pem))
    rc, out, err = run_cmd(cmd)

    if combined_path and combined_path.exists():
        combined_path.unlink()

    return (rc == 0), (out.strip() or err.strip())


def extract_pubkey_raw_from_cert(cert_der: Path) -> Tuple[bool, str]:
    rc1, pem_pub, err1 = run_cmd(["openssl", "x509", "-inform", "DER", "-in", str(cert_der), "-pubkey", "-noout"])
    if rc1 != 0:
        return False, f"failed extracting cert pubkey: {err1.strip()}"

    proc = subprocess.run(
        ["openssl", "pkey", "-pubin", "-outform", "DER"],
        input=pem_pub.encode("utf-8"),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        return False, f"failed converting cert pubkey to DER: {proc.stderr.decode('utf-8', errors='replace').strip()}"

    der_pub = proc.stdout
    prefix = bytes.fromhex("302a300506032b656e032100")  # X25519 SPKI prefix
    if not der_pub.startswith(prefix) or len(der_pub) != len(prefix) + 32:
        return False, f"unexpected cert pubkey DER format (len={len(der_pub)})"
    return True, der_pub[len(prefix):].hex()


def extract_pubkey_raw_from_hex_private_key(hex_key_path: Path) -> Tuple[bool, str]:
    raw_hex = re.sub(r"\s+", "", hex_key_path.read_text(encoding="utf-8", errors="replace"))
    if not re.fullmatch(r"[0-9A-Fa-f]+", raw_hex):
        return False, "private key hex contains non-hex characters"
    if len(raw_hex) != 64:
        return False, f"unexpected private key hex length={len(raw_hex)} (expected 64)"

    raw = bytes.fromhex(raw_hex)
    pkcs8 = bytes.fromhex("302e020100300506032b656e04220420") + raw  # X25519 private key PKCS#8 wrapper

    with tempfile.NamedTemporaryFile(suffix=".der", delete=False) as f:
        tmp = Path(f.name)
        f.write(pkcs8)

    try:
        proc = subprocess.run(
            ["openssl", "pkey", "-inform", "DER", "-in", str(tmp), "-pubout", "-outform", "DER"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if proc.returncode != 0:
            return False, "openssl could not derive pubkey from private key: " + proc.stderr.decode("utf-8", errors="replace").strip()

        out_der = proc.stdout
        prefix = bytes.fromhex("302a300506032b656e032100")
        if not out_der.startswith(prefix) or len(out_der) != len(prefix) + 32:
            return False, f"unexpected derived pubkey DER format (len={len(out_der)})"

        return True, out_der[len(prefix):].hex()
    finally:
        tmp.unlink(missing_ok=True)


def parse_hash_tokens(field_value: str) -> List[str]:
    return [tok for tok in field_value.split("#") if tok]


def parse_int_list(value: str) -> List[int]:
    v = value.strip()
    if not (v.startswith("[") and v.endswith("]")):
        return []
    body = v[1:-1].strip()
    if not body:
        return []

    out: List[int] = []
    for part in body.split(","):
        p = part.strip()
        if not p:
            continue
        try:
            out.append(int(p, 0))
        except ValueError:
            pass
    return out


def parse_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def parse_int(value: str) -> Optional[int]:
    try:
        return int(value, 0)
    except (TypeError, ValueError):
        return None


def parse_certified_function_ids_from_pseudonym(pseudonym: str) -> List[int]:
    raw = re.sub(r"[^0-9A-Fa-f]", "", pseudonym)
    if len(raw) < 4 or len(raw) % 2 != 0:
        return []

    data = bytes.fromhex(raw)
    if len(data) < 2:
        return []

    count = data[0]
    body = data[1:]

    parts: List[bytes] = []
    cur = bytearray()
    for b in body:
        if b == 0x5F:
            if cur:
                parts.append(bytes(cur))
                cur.clear()
        else:
            cur.append(b)
    if cur:
        parts.append(bytes(cur))

    fn_ids: List[int] = []
    for p in parts:
        if p:
            fn = p[0]
            if fn not in fn_ids:
                fn_ids.append(fn)

    if not fn_ids:
        for i in range(0, len(body), 2):
            fn = body[i]
            if fn not in fn_ids:
                fn_ids.append(fn)

    if 0 < count <= len(fn_ids):
        fn_ids = fn_ids[:count]

    return fn_ids


def decode_isobus_name(name_hex: str) -> Optional[Dict[str, int]]:
    clean = re.sub(r"[^0-9A-Fa-f]", "", name_hex)
    if len(clean) != 16:
        return None
    value = int.from_bytes(bytes.fromhex(clean), "little")
    return {
        "identity_number": value & ((1 << 21) - 1),
        "manufacturer_code": (value >> 21) & ((1 << 11) - 1),
        "ecu_instance": (value >> 32) & 0x7,
        "function_instance": (value >> 35) & 0x1F,
        "function": (value >> 40) & 0xFF,
        "reserved": (value >> 48) & 0x1,
        "vehicle_system": (value >> 49) & 0x7F,
        "vehicle_system_instance": (value >> 56) & 0xF,
        "industry_group": (value >> 60) & 0x7,
        "arbitrary_address_capable": (value >> 63) & 0x1,
    }


def extract_metadata_from_subject_issuer(text: str) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for line in text.splitlines():
        if line.startswith("subject="):
            out["subject"] = line[len("subject="):].strip()
        elif line.startswith("issuer="):
            out["issuer"] = line[len("issuer="):].strip()
        elif line.startswith("serial="):
            out["serial"] = line[len("serial="):].strip()

    src = f"{out.get('subject', '')},{out.get('issuer', '')}"
    for key in ("O=", "OU=", "pseudonym=", "CN="):
        m = re.search(rf"{re.escape(key)}([^,]+)", src)
        if m:
            out[key[:-1]] = m.group(1)
    return out


def run_checks(rep: Reporter, values: Dict[str, str], repo_root: Path, strict_crl: bool) -> None:
    missing_keys = [k for k in REQUIRED_PARAM_KEYS if k not in values]
    if missing_keys:
        raise FatalCheckError("missing required keys: " + ", ".join(missing_keys))
    rep.add(True, "required authlib params", "all required keys present")

    for k in INFO_PARAM_KEYS:
        if k in values:
            rep.add(True, f"param {k}", values[k])

    resolved: Dict[str, Path] = {}
    for k in REQUIRED_PARAM_KEYS:
        resolved_path = resolve_path(values[k], repo_root)
        if resolved_path is None:
            rep.add(False, f"path {k}", f"not found: {values[k]}")
        else:
            resolved[k] = resolved_path
            rep.add(True, f"path {k}", str(resolved_path))

    if not all(k in resolved for k in REQUIRED_PARAM_KEYS):
        return

    cert_keys = REQUIRED_PARAM_KEYS[:-1]
    cert_paths = [resolved[k] for k in cert_keys]
    for cp in cert_paths:
        ok, details = get_cert_subject_issuer(cp)
        rep.add(ok, f"parse certificate {cp.name}", details.splitlines()[0] if ok else details)

    with tempfile.TemporaryDirectory(prefix="tim_auth_check_") as tmpdir:
        tmp = Path(tmpdir)
        pem_paths: Dict[str, Path] = {}
        cert_map = {
            "root": resolved["authlib.root_cert_path"],
            "testlab": resolved["authlib.client_testlab_cert_path"],
            "manufacturer": resolved["authlib.client_manufacturer_cert_path"],
            "series": resolved["authlib.client_series_cert_path"],
            "device": resolved["authlib.client_device_cert_path"],
        }

        for label, der in cert_map.items():
            out_pem = tmp / f"{label}.pem"
            ok, err = der_to_pem(der, out_pem)
            rep.add(ok, f"DER->PEM {label}", "ok" if ok else err)
            if ok:
                pem_paths[label] = out_pem

        cert_dir = resolved["authlib.client_device_cert_path"].parent
        crl_files = [cert_dir / "crl.der", cert_dir / "crl_sub_ca.der"]
        parsed_crls: List[Path] = []
        for c in crl_files:
            if c.exists():
                out_pem = tmp / f"{c.stem}.pem"
                ok, err = crl_der_to_pem(c, out_pem)
                rep.add(ok, f"parse CRL {c.name}", "ok" if ok else err)
                if ok:
                    parsed_crls.append(out_pem)
            else:
                ok = not strict_crl
                rep.add(ok, f"CRL file {c.name}", "missing" if strict_crl else "missing (optional)")

        crl_for_verify = parsed_crls[0] if parsed_crls else None

        if len(pem_paths) == 5:
            ok, out = chain_verify(pem_paths["root"], [], pem_paths["testlab"], crl_for_verify)
            rep.add(ok, "chain verify testlab <- root", out)

            ok, out = chain_verify(pem_paths["root"], [pem_paths["testlab"]], pem_paths["manufacturer"], crl_for_verify)
            rep.add(ok, "chain verify manufacturer <- testlab <- root", out)

            ok, out = chain_verify(
                pem_paths["root"], [pem_paths["testlab"], pem_paths["manufacturer"]], pem_paths["series"], crl_for_verify
            )
            rep.add(ok, "chain verify series <- manufacturer <- testlab <- root", out)

            ok, out = chain_verify(
                pem_paths["root"], [pem_paths["testlab"], pem_paths["manufacturer"], pem_paths["series"]], pem_paths["device"], crl_for_verify
            )
            rep.add(ok, "chain verify device <- series <- manufacturer <- testlab <- root", out)

    ok_key, derived_pub = extract_pubkey_raw_from_hex_private_key(resolved["authlib.client_private_key_hex"])
    rep.add(ok_key, "private key parse/derive pubkey", derived_pub)

    ok_cert_pub, cert_pub = extract_pubkey_raw_from_cert(resolved["authlib.client_device_cert_path"])
    rep.add(ok_cert_pub, "device cert public key extraction", cert_pub)

    if ok_key and ok_cert_pub:
        same = derived_pub.lower() == cert_pub.lower()
        rep.add(same, "private key matches device cert", "public keys are identical" if same else "mismatch")

    ok_meta, meta_text = get_cert_subject_issuer(resolved["authlib.client_device_cert_path"])
    if ok_meta:
        meta = extract_metadata_from_subject_issuer(meta_text)
        rep.add(True, "device cert issuer", meta.get("issuer", "<not found>"))
        for key in ("O", "OU", "pseudonym"):
            if key in meta:
                rep.add(True, f"issuer {key} tokens", f"{meta[key]} -> {parse_hash_tokens(meta[key])}")
            else:
                rep.add(False, f"issuer {key} tokens", "not present")

        tok_o = parse_hash_tokens(meta.get("O", ""))
        tok_ou = parse_hash_tokens(meta.get("OU", ""))

        o_industry = o_device_class = o_tech_type = o_cert_type = -1
        if len(tok_o) >= 7:
            try:
                o_industry = int(tok_o[0], 16)
                o_device_class = int(tok_o[1], 16)
                o_tech_type = int(tok_o[2], 16)
                o_cert_type = int(tok_o[4], 16)
                rep.add(True, "certificate O token decode", f"industry={o_industry}, device_class={o_device_class}, tech={o_tech_type}, cert_type={o_cert_type}")
                rep.add(o_tech_type == 0x00, "O.technology_type (TIM)", f"value=0x{o_tech_type:02X} expected=0x00")
                rep.add(o_cert_type == 0x01, "O.certificate_type (client)", f"value=0x{o_cert_type:02X} expected=0x01")
            except ValueError:
                rep.add(False, "certificate O token decode", f"cannot parse as hex: {tok_o}")
        else:
            rep.add(False, "certificate O token decode", f"need >=7 tokens, got {len(tok_o)}")

        ou_manufacturer = ou_year = ou_revision = ou_test_lab = ou_reference = -1
        if len(tok_ou) >= 5:
            try:
                ou_manufacturer = int(tok_ou[0], 16)
                ou_year = int(tok_ou[1], 16)
                ou_revision = int(tok_ou[2], 16)
                ou_test_lab = int(tok_ou[3], 16)
                ou_reference = int(tok_ou[4], 16)
                rep.add(True, "certificate OU token decode", f"manufacturer=0x{ou_manufacturer:04X}, year=0x{ou_year:02X}, rev=0x{ou_revision:02X}, test_lab=0x{ou_test_lab:04X}, ref=0x{ou_reference:04X}")
            except ValueError:
                rep.add(False, "certificate OU token decode", f"cannot parse as hex: {tok_ou}")
        else:
            rep.add(False, "certificate OU token decode", f"need >=5 tokens, got {len(tok_ou)}")

        name_cfg = values.get("ecu_name_hex", "").strip().upper()
        name_fields = decode_isobus_name(name_cfg) if name_cfg else None
        if name_fields is None:
            rep.add(False, "ISOBUS NAME decode", f"invalid ecu_name_hex '{name_cfg}'")
        else:
            rep.add(True, "ISOBUS NAME decode", f"industry_group={name_fields['industry_group']}, vehicle_system={name_fields['vehicle_system']}, manufacturer_code=0x{name_fields['manufacturer_code']:04X}")
            rep.add(o_industry == name_fields["industry_group"], "NAME industry_group vs cert O[0]", f"name={name_fields['industry_group']} cert=0x{o_industry:02X}")
            rep.add(o_device_class == name_fields["vehicle_system"], "NAME device_class vs cert O[1]", f"name(vehicle_system)={name_fields['vehicle_system']} cert=0x{o_device_class:02X}")
            rep.add(ou_manufacturer == name_fields["manufacturer_code"], "NAME manufacturer_code vs cert OU[0]", f"name=0x{name_fields['manufacturer_code']:04X} cert=0x{ou_manufacturer:04X}")

        cert_year = parse_int(values.get("compliance_cert_year", ""))
        cert_revision = parse_int(values.get("compliance_cert_revision", ""))
        test_lab_id = parse_int(values.get("compliance_test_lab_id", ""))
        reference = parse_int(values.get("compliance_reference_number", ""))

        if cert_year is not None:
            expected_year_byte = (cert_year - 2000) & 0xFF if cert_year >= 2000 else cert_year & 0xFF
            rep.add(ou_year == expected_year_byte, "conformance year vs cert OU[1]", f"param={cert_year} -> encoded=0x{expected_year_byte:02X} cert=0x{ou_year:02X}")
        if cert_revision is not None:
            rep.add(ou_revision == (cert_revision & 0xFF), "conformance revision vs cert OU[2]", f"param={cert_revision} cert=0x{ou_revision:02X}")
        if test_lab_id is not None:
            rep.add(ou_test_lab == (test_lab_id & 0xFFFF), "conformance test lab ID vs cert OU[3]", f"param={test_lab_id} cert=0x{ou_test_lab:04X}")
        if reference is not None:
            rep.add(ou_reference == (reference & 0xFFFF), "conformance reference number vs cert OU[4]", f"param={reference} cert=0x{ou_reference:04X}")

    requested_fn_ids: List[int] = []
    for a in parse_int_list(values.get("tim.aux_fn_ids", "[]")):
        if 1 <= a <= 0x20 and a not in requested_fn_ids:
            requested_fn_ids.append(a)
    if parse_bool(values.get("tim.enable_speed", "false")):
        requested_fn_ids.append(0x44)
    if parse_bool(values.get("tim.enable_curvature", "false")):
        requested_fn_ids.append(0x46)
    if parse_bool(values.get("tim.enable_rear_pto", "false")):
        requested_fn_ids.append(0x41)
    if parse_bool(values.get("tim.enable_rear_hitch", "false")):
        requested_fn_ids.append(0x43)

    requested_fn_ids = sorted(set(requested_fn_ids))
    rep.add(True, "requested TIM function IDs (from params)", ", ".join(f"0x{x:02X}" for x in requested_fn_ids) if requested_fn_ids else "<none>")

    if "authlib.client_series_cert_path" in resolved:
        ok_series_meta, series_meta_text = get_cert_subject_issuer(resolved["authlib.client_series_cert_path"])
        if ok_series_meta:
            series_meta = extract_metadata_from_subject_issuer(series_meta_text)
            pseudonym = series_meta.get("pseudonym", "")
            if pseudonym:
                certified_fn_ids = parse_certified_function_ids_from_pseudonym(pseudonym)
                rep.add(True, "certified TIM function IDs (heuristic from pseudonym)", f"pseudonym={pseudonym}; parsed=" + (", ".join(f"0x{x:02X}" for x in certified_fn_ids) if certified_fn_ids else "<none>"))
                if certified_fn_ids and requested_fn_ids:
                    missing = [x for x in requested_fn_ids if x not in certified_fn_ids]
                    rep.add(len(missing) == 0, "requested functions covered by certificate", "all requested functions found" if not missing else "missing: " + ", ".join(f"0x{x:02X}" for x in missing))
            else:
                rep.add(False, "series certificate pseudonym", "not found; cannot infer certified function IDs")
        else:
            rep.add(False, "parse series certificate metadata", series_meta_text)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check TIM AuthLib certificate compatibility")
    parser.add_argument("--params-file", default="config/all_nodes_tim_params.yaml", help="ROS2 params YAML path")
    parser.add_argument("--repo-root", default=str(Path(__file__).resolve().parents[1]), help="Ros2ISOBUS repository root")
    parser.add_argument("--node-name", default="tim_client_node", help="ROS2 node section name in YAML")
    parser.add_argument("--strict-crl", action="store_true", help="fail if expected CRL files are missing")
    parser.add_argument("--output", choices=("text", "json"), default="text", help="output format")
    parser.add_argument("--quiet", action="store_true", help="text output: suppress notes")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rep = Reporter()

    openssl_path = shutil.which("openssl")
    if openssl_path is None:
        if args.output == "json":
            print(json.dumps({"fatal": "openssl not found in PATH"}, indent=2))
        else:
            print("FATAL: openssl not found in PATH", file=sys.stderr)
        return 2
    rep.add(True, "openssl availability", f"using {openssl_path}")

    repo_root = Path(args.repo_root).resolve()
    params_file = Path(args.params_file)
    if not params_file.is_absolute():
        params_file = (repo_root / params_file).resolve()

    if not params_file.exists():
        if args.output == "json":
            print(json.dumps({"fatal": f"params file not found: {params_file}"}, indent=2))
        else:
            print(f"FATAL: params file not found: {params_file}", file=sys.stderr)
        return 2

    rep.add(True, "params file", f"loaded: {params_file}")

    values = load_params(params_file, args.node_name)
    if not values:
        if args.output == "json":
            print(json.dumps({"fatal": f"could not parse parameters for node '{args.node_name}'"}, indent=2))
        else:
            print(f"FATAL: could not parse parameters for node '{args.node_name}'", file=sys.stderr)
        return 2

    try:
        run_checks(rep, values, repo_root, strict_crl=args.strict_crl)
    except FatalCheckError as exc:
        if args.output == "json":
            print(json.dumps({"fatal": str(exc)}, indent=2))
        else:
            print(f"FATAL: {exc}", file=sys.stderr)
        return 2

    if args.output == "json":
        rep.render_json()
    else:
        rep.render_text(include_notes=not args.quiet)

    return 1 if rep.failed() else 0


if __name__ == "__main__":
    sys.exit(main())
