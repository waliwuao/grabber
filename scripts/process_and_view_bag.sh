#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
input_path="${1:-}"
expected_count="${2:-6}"

if [[ -z "$input_path" ]]; then
  metadata_file="$(
    find "$workspace/data/bags" \
      -mindepth 3 -maxdepth 3 -name metadata.yaml \
      -printf '%T@ %p\n' \
      | sort -n \
      | tail -n 1 \
      | cut -d' ' -f2-
  )"
  if [[ -z "$metadata_file" ]]; then
    echo "错误：data/bags 中没有可用的 rosbag。"
    exit 1
  fi
  bag_dir="$(dirname "$metadata_file")"
else
  input_path="$(realpath "$input_path")"
  if [[ -f "$input_path/metadata.yaml" ]]; then
    bag_dir="$input_path"
  elif [[ -f "$input_path/bag/metadata.yaml" ]]; then
    bag_dir="$input_path/bag"
  else
    echo "错误：路径中没有找到 metadata.yaml：$input_path"
    exit 2
  fi
fi

if [[ ! "$expected_count" =~ ^[1-9][0-9]*$ ]]; then
  echo "错误：目标数量必须是正整数。"
  exit 2
fi

run_dir="$(dirname "$bag_dir")"
result_file="$run_dir/recognition_result.json"

set +u
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
set -u

echo "============================================================"
echo "处理 rosbag：$bag_dir"
echo "固定目标数：$expected_count"
echo "结果文件：$result_file"
echo "============================================================"

ros2 run spear_locator analyze_bag \
  "$bag_dir" \
  --expected-count "$expected_count" \
  | tee "$result_file"

echo
echo "离线处理完成，正在启动循环回放、在线识别和 RViz。"
echo "按 Ctrl+C 可一次性停止全部进程。"

exec ros2 launch spear_locator bag_recognition_viewer.launch.py \
  bag:="$bag_dir" \
  expected_count:="$expected_count"
