# TIM Auth Compatibility Checker

`tim_auth_compat_check.py` validates TIMClient AuthLib certificate/key configuration against common interoperability and guideline expectations.

## What It Checks
- required `authlib.*` parameter presence
- certificate file resolution and parseability
- chain verification:
  - root -> testlab
  - root -> testlab -> manufacturer
  - root -> testlab -> manufacturer -> series
  - root -> testlab -> manufacturer -> series -> device
- X25519 private key matches device certificate public key
- issuer O/OU token decode + AEF 040 style checks
- ISOBUS NAME and conformance fields vs certificate metadata
- requested TIM function IDs vs series-certificate pseudonym (best-effort)

## Usage
From repository root:

```bash
./scripts/tim_auth_compat_check.py
```

With explicit files:

```bash
./scripts/tim_auth_compat_check.py \
  --repo-root /path/to/Ros2ISOBUS \
  --params-file config/all_nodes_tim_params.yaml \
  --node-name tim_client_node
```

CI-friendly JSON output:

```bash
./scripts/tim_auth_compat_check.py --output json --quiet
```

Strict CRL mode:

```bash
./scripts/tim_auth_compat_check.py --strict-crl
```

## Exit Codes
- `0`: all checks passed
- `1`: one or more checks failed
- `2`: fatal runtime/configuration error (missing openssl, params file not found, parse failure)

## Notes
- Function-ID extraction from certificate pseudonym is heuristic (best-effort).
- This script does not replace full AEF conformance testing; it is a pre-check tool.
