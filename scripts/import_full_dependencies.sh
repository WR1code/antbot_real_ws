#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"
manifest="${workspace}/repos/antbot_dependencies.repos"

if ! command -v vcs >/dev/null 2>&1; then
  echo "错误：请先安装 python3-vcstool。" >&2
  exit 1
fi

vcs import "${workspace}/src" --skip-existing < "$manifest"
