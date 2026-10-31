#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
sudo install -Dm644 "$ROOT/systemd/aerothon-mission.service" /etc/systemd/system/aerothon-mission.service
sudo systemctl daemon-reload
sudo systemctl enable aerothon-mission.service
echo "Installed. Start manually after editing vehicle files: sudo systemctl start aerothon-mission"
