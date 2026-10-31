#!/usr/bin/env bash
set -euo pipefail

profile="${1:-}"
case "${profile}" in
  segment1|segment2) ;;
  *)
    echo "Usage: bash deploy/select_mission_profile.sh {segment1|segment2}" >&2
    exit 2
    ;;
esac

token="$(sudo sed -n 's/^R2D2_CONTROL_TOKEN=//p' /etc/default/r2d2-autonomous | head -n 1)"
if [[ -z "${token}" ]]; then
  echo "R2D2_CONTROL_TOKEN is missing from /etc/default/r2d2-autonomous" >&2
  exit 2
fi

curl --fail-with-body --silent --show-error \
  -H 'Content-Type: application/json' \
  -H "X-R2D2-Token: ${token}" \
  --data "{\"profile\":\"${profile}\"}" \
  http://127.0.0.1:5000/api/profile
printf '\n'
