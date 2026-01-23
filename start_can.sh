#!/usr/bin/env bash
set -euo pipefail

# Bring up a SocketCAN interface (default: can0) at 250 kbit/s.
IFACE=${1:-can0}
BITRATE=250000

if ! command -v ip >/dev/null 2>&1; then
  echo "ip command not found (install iproute2)" >&2
  exit 1
fi

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root or with sudo: sudo $0 [iface]" >&2
  exit 1
fi

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "Interface '$IFACE' not found" >&2
  exit 1
fi

# Reset and configure bitrate, then bring interface up.
ip link set "$IFACE" down 2>/dev/null || true
ip link set "$IFACE" type can bitrate "$BITRATE"
ip link set "$IFACE" up

echo "${IFACE} up @ ${BITRATE} bps"
ip -details link show "$IFACE" | grep -E "state|bitrate"
