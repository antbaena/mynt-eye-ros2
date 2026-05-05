#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo cp "${ROOT_DIR}/88-mynteye.rules" /etc/udev/rules.d/88-mynteye.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG video "$USER" || true

echo "[OK] Reglas udev instaladas. Puede hacer falta cerrar sesión y volver a entrar."