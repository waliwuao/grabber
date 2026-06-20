#!/usr/bin/env bash
set -euo pipefail

libreoffice_root="/home/charlie/bigspace/apps/LibreOffice/root/opt/libreoffice25.8"
profile_dir="/home/charlie/bigspace/apps/LibreOffice/user-profile"

if [[ ! -x "$libreoffice_root/program/soffice" ]]; then
  echo "错误：没有找到 bigspace 中的 LibreOffice。"
  exit 1
fi

mkdir -p "$profile_dir"

exec "$libreoffice_root/program/soffice" \
  "-env:UserInstallation=file://$profile_dir" \
  --calc \
  "$@"
