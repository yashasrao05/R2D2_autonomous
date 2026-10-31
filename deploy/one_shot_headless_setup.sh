#!/usr/bin/env bash
set -Eeuo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
expected_repo="/home/r2d2/R2D2_autonomous"
pixhawk_ip="192.168.168.2"
jetson_ip_cidr="192.168.168.10/24"
service_name="r2d2-autonomous.service"
ethernet_interface="${R2D2_ETH_IFACE:-}"
run_winch=false
run_cameras=true
start_service=true

usage() {
  cat <<'EOF'
Usage: bash deploy/one_shot_headless_setup.sh [options]
  --ethernet-interface IFACE  Pixhawk-only wired interface (auto-detected when unique)
  --with-winch-test           Run the 65-second Servo 7 PWM 2000 bench test
  --skip-cameras              Skip the 10-second headless camera/stream test
  --no-start                  Install and enable the service without starting it

Run only while the aircraft is DISARMED, propellers are removed, TX15 SC is
DOWN, and RC12 is held at its actual mechanical neutral. The script never arms.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ethernet-interface) ethernet_interface="${2:?missing interface}"; shift 2 ;;
    --with-winch-test) run_winch=true; shift ;;
    --skip-cameras) run_cameras=false; shift ;;
    --no-start) start_service=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

fail() { echo "SETUP FAILED: $*" >&2; exit 2; }
step() { printf '\n==== %s ====\n' "$*"; }

[[ "${repo_dir}" == "${expected_repo}" ]] || fail "repository must be ${expected_repo}; found ${repo_dir}"
[[ "$(id -un)" == "r2d2" ]] || fail "run this as user r2d2, not $(id -un)"
[[ -f "${repo_dir}/runner.py" ]] || fail "runner.py is missing"
[[ -f "${repo_dir}/config/mission_mapping.yaml" ]] || fail "mission mapping is missing"

step "Acquire sudo and stop the previous boot service"
sudo -v
sudo systemctl stop "${service_name}" 2>/dev/null || true
if systemctl list-unit-files aerothon-mission.service --no-legend 2>/dev/null | grep -q '^aerothon-mission\.service'; then
  echo "Disabling obsolete aerothon-mission.service to prevent two controllers at boot."
  sudo systemctl disable --now aerothon-mission.service
fi

step "Select and persist the Pixhawk Ethernet interface"
if [[ -z "${ethernet_interface}" ]]; then
  mapfile -t candidates < <(
    for path in /sys/class/net/*; do
      name="$(basename "${path}")"
      [[ "${name}" == "lo" ]] && continue
      [[ -d "${path}/wireless" ]] && continue
      [[ -e "${path}/device" ]] || continue
      printf '%s\n' "${name}"
    done
  )
  for name in "${candidates[@]}"; do
    if ip -4 -o address show dev "${name}" | grep -q '192\.168\.168\.10/24'; then
      ethernet_interface="${name}"
      break
    fi
  done
  if [[ -z "${ethernet_interface}" && ${#candidates[@]} -eq 1 ]]; then
    ethernet_interface="${candidates[0]}"
  fi
  [[ -n "${ethernet_interface}" ]] || fail "cannot uniquely choose the wired Pixhawk interface; rerun with --ethernet-interface IFACE. Candidates: ${candidates[*]:-none}"
fi
ip link show dev "${ethernet_interface}" >/dev/null 2>&1 || fail "interface ${ethernet_interface} does not exist"

if ! ip -4 -o address show dev "${ethernet_interface}" | grep -q '192\.168\.168\.10/24'; then
  if [[ -n "${SSH_CONNECTION:-}" ]]; then
    ssh_client="${SSH_CONNECTION%% *}"
    ssh_device="$(ip route get "${ssh_client}" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' | head -n1)"
    [[ "${ssh_device}" != "${ethernet_interface}" ]] || fail "refusing to readdress ${ethernet_interface}; this PuTTY session uses it"
  fi
  command -v nmcli >/dev/null 2>&1 || fail "NetworkManager/nmcli is required for persistent headless Ethernet configuration"
  connection="$(nmcli -g GENERAL.CONNECTION device show "${ethernet_interface}" | head -n1)"
  if [[ -z "${connection}" || "${connection}" == "--" ]]; then
    connection="r2d2-pixhawk"
    sudo nmcli connection add type ethernet ifname "${ethernet_interface}" con-name "${connection}"
  fi
  sudo nmcli connection modify "${connection}" \
    ipv4.method manual ipv4.addresses "${jetson_ip_cidr}" \
    ipv4.gateway "" ipv4.dns "" ipv4.never-default yes ipv6.method disabled
  sudo nmcli connection up "${connection}"
fi
ip -4 -br address show dev "${ethernet_interface}"
ip route get "${pixhawk_ip}" | grep -q "dev ${ethernet_interface}" || fail "route to ${pixhawk_ip} does not use ${ethernet_interface}"
ping -c 2 -W 1 "${pixhawk_ip}" >/dev/null 2>&1 || echo "WARNING: Pixhawk did not answer ICMP; MAVLink heartbeat check remains authoritative."

step "Create/update the isolated Python environment"
cd "${repo_dir}"
if [[ ! -x .venv/bin/python3 ]]; then
  python3 -m venv .venv
fi
.venv/bin/python3 -m pip install --upgrade pip setuptools wheel
.venv/bin/python3 -m pip install -r requirements.txt
.venv/bin/python3 -B scripts/environment_check.py

step "Run unit and mock-flight gates"
.venv/bin/python3 -B -m unittest discover -s tests -v
.venv/bin/python3 -B scripts/mock_mission2_sil.py --profile all

step "Install the headless boot service"
bash deploy/install_boot_service.sh

step "Run live non-arming Ethernet/FC readiness"
.venv/bin/python3 -B runner.py --mode configure-servo7 --transport ethernet
.venv/bin/python3 -B runner.py --mode competition-check --transport ethernet --lifecycle-seconds 5

step "Run finite TX15 monitor (no mode change, arm, or takeoff)"
.venv/bin/python3 -B runner.py --mode test-tx15 --transport ethernet \
  --tx15-timeout 10 --tx15-monitor-seconds 5

if [[ "${run_cameras}" == true ]]; then
  step "Run both cameras headlessly for 10 seconds"
  .venv/bin/python3 -B scripts/test_cameras.py --camera both --duration 10
fi

if [[ "${run_winch}" == true ]]; then
  step "Run requested Servo 7 winch bench test: PWM 2000 for 65 seconds"
  .venv/bin/python3 -B runner.py --mode test-servo7 --transport ethernet \
    --servo7-pwm 2000 --servo7-seconds 65
else
  echo "Winch actuation skipped. Rerun with --with-winch-test only with propellers removed and the winch secured."
fi

if [[ "${start_service}" == true ]]; then
  step "Start and verify the boot service"
  sudo systemctl restart "${service_name}"
  sleep 3
  sudo systemctl is-active --quiet "${service_name}" || {
    sudo systemctl status "${service_name}" --no-pager || true
    sudo journalctl -u "${service_name}" -n 80 --no-pager || true
    fail "${service_name} did not become active"
  }
  sudo systemctl status "${service_name}" --no-pager --lines=12
fi

step "ONE-SHOT HEADLESS SETUP PASSED"
echo "Ethernet=${ethernet_interface}; service enabled=$(systemctl is-enabled "${service_name}" 2>/dev/null || true)"
echo "Logs: sudo journalctl -fu ${service_name}"
echo "Profile selection from this PuTTY session: bash deploy/select_mission_profile.sh full|segment1|segment2"
