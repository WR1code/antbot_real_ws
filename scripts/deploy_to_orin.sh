#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"
target="${1:-}"
destination="${2:-~/antbot_real_ws}"

if [[ -z "$target" ]]; then
  echo "用法：$0 <orin用户@主机或IP> [远端目录]" >&2
  echo "示例：$0 orin@192.168.1.50 /home/orin/antbot_real_ws" >&2
  exit 2
fi

rsync -az --info=progress2 \
  --exclude '/build/' \
  --exclude '/install/' \
  --exclude '/log/' \
  --exclude '/dist/' \
  --exclude '__pycache__/' \
  --exclude '.pytest_cache/' \
  "${workspace}/" "${target}:${destination%/}/"

echo
echo "复制完成。登录 Orin 后运行："
echo "  cd ${destination}"
echo "  ./scripts/install_dependencies.sh core"
echo "  ./scripts/build.sh"
echo "  source ./scripts/setup_env.sh"
echo "  ./scripts/check_hardware.sh"
