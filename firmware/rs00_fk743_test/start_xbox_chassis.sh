#!/usr/bin/env bash
set -Eeuo pipefail

readonly PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$PROJECT_DIR/host/run_antbot_xbox.sh" "$@"
