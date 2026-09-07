#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cubemx_bin="/usr/local/STMicroelectronics/STM32Cube/STM32CubeMX/STM32CubeMX"
script_file="$(mktemp)"
trap 'rm -f -- "$script_file"' EXIT

if [[ ! -x "$cubemx_bin" ]]; then
    echo "未找到 STM32CubeMX: $cubemx_bin" >&2
    exit 1
fi

printf 'config load %s\nproject generate %s\nexit\n' \
    "$project_dir/RS00_FK743.ioc" "$project_dir" >"$script_file"
"$cubemx_bin" -q "$script_file"
