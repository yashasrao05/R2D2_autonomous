#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
expected_repo="/home/r2d2/R2D2_autonomous"
unit_name="r2d2-autonomous.service"

if [[ "${repo_dir}" != "${expected_repo}" ]]; then
  echo "Repository is ${repo_dir}; service expects ${expected_repo}." >&2
  echo "Move the repository or edit deploy/${unit_name} before installation." >&2
  exit 2
fi
if [[ ! -x "${repo_dir}/.venv/bin/python3" ]]; then
  echo "Missing executable ${repo_dir}/.venv/bin/python3" >&2
  exit 2
fi

"${repo_dir}/.venv/bin/python3" -B "${repo_dir}/scripts/environment_check.py"

sudo install -o root -g root -m 0644 "${repo_dir}/deploy/${unit_name}" "/etc/systemd/system/${unit_name}"
if [[ ! -e /etc/default/r2d2-autonomous ]]; then
  control_token="$("${repo_dir}/.venv/bin/python3" -c 'import secrets; print(secrets.token_urlsafe(24))')"
  printf 'R2D2_CONTROL_TOKEN=%s\n' "${control_token}" | sudo tee /etc/default/r2d2-autonomous >/dev/null
  sudo chmod 0600 /etc/default/r2d2-autonomous
  echo "Field console token (store on the operator laptop): ${control_token}"
else
  echo "Preserving existing /etc/default/r2d2-autonomous control token."
fi
sudo systemctl daemon-reload
sudo systemctl enable "${unit_name}"

echo "Installed and enabled ${unit_name}."
echo "Run the non-arming checks first:"
echo "  cd ${repo_dir}"
echo "  .venv/bin/python3 runner.py --mode competition-check --transport ethernet"
echo "Then start with: sudo systemctl start ${unit_name}"
echo "Open http://10.233.44.10:5000/ and select the official run with SC DOWN."
