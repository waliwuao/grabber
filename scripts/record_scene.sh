#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 5 ]]; then
  echo "用法: $0 <场景名> <真值距离_mm> [时长_s=30] [重复编号=1] [矛头数量]"
  echo "示例: $0 six_spears 200 30 1 6"
  exit 2
fi

scene_name="$1"
distance_mm="$2"
duration_s="${3:-30}"
repeat_index="${4:-1}"
target_count="${5:-待填写}"

if [[ ! "$scene_name" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "错误：场景名只能包含字母、数字、下划线和连字符。"
  exit 2
fi
if [[ ! "$distance_mm" =~ ^[0-9]+$ ]] || (( distance_mm > 400 )); then
  echo "错误：真值距离必须是 0～400 的整数毫米。"
  exit 2
fi
if [[ ! "$duration_s" =~ ^[0-9]+$ ]] || (( duration_s < 5 )); then
  echo "错误：录制时长必须是不小于 5 秒的整数。"
  exit 2
fi
if [[ ! "$repeat_index" =~ ^[0-9]+$ ]] || (( repeat_index < 1 )); then
  echo "错误：重复编号必须是正整数。"
  exit 2
fi
if [[ "$target_count" != "待填写" ]] && (
  [[ ! "$target_count" =~ ^[0-9]+$ ]] || (( target_count > 6 ))
); then
  echo "错误：矛头数量必须是 0～6 的整数。"
  exit 2
fi

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS 2 setup scripts may read optional variables before defining them, which
# is incompatible with nounset (`set -u`). Temporarily disable nounset while
# sourcing the environments, then restore strict mode for this script.
set +u
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
set -u

if ! timeout 5s ros2 topic echo /scan --once >/dev/null 2>&1; then
  echo "错误：5 秒内没有收到 /scan。请先启动 STL-27L 驱动。"
  exit 1
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
run_name="${scene_name}_${distance_mm}mm_r${repeat_index}_${timestamp}"
run_dir="$workspace/data/bags/$run_name"
bag_dir="$run_dir/bag"
mkdir -p "$run_dir"

{
  echo "# 场景元数据"
  echo
  echo "- 场景名称：$scene_name"
  echo "- 真值距离：$distance_mm mm"
  echo "- 录制时长：$duration_s s"
  echo "- 重复编号：$repeat_index"
  echo "- 录制时间：$(date --iso-8601=seconds)"
  echo "- 雷达型号：STL-27L"
  echo "- 雷达话题：/scan"
  echo "- 最大要求距离：400 mm"
  echo "- 目标误差要求：≤10 mm"
  echo "- 矛头数量：$target_count"
  echo "- 标准间距：待填写 mm"
  echo "- 矛头宽度：待填写 mm"
  echo "- 矛头材质：待填写"
  echo "- 扫描平面位置：待填写"
  echo "- 遮挡/干扰说明：待填写"
  echo "- 真值测量工具及精度：待填写"
} > "$run_dir/scene.md"

echo "开始录制：$run_name"
echo "数据目录：$run_dir"
echo "将在 $duration_s 秒后自动停止。"

record_pid=""

stop_recorder() {
  if [[ -n "$record_pid" ]] && kill -0 "$record_pid" 2>/dev/null; then
    echo "正在停止录制并写入数据，请稍候……"
    # rosbag2 on Jazzy may not finish flushing its cache within the old
    # timeout/INT grace period. TERM is handled gracefully by the recorder.
    kill -TERM -- "-$record_pid" 2>/dev/null || true
    wait "$record_pid" 2>/dev/null || true
  fi
}

on_interrupt() {
  echo
  echo "收到停止请求。"
  stop_recorder
  if [[ -f "$bag_dir/metadata.yaml" ]]; then
    echo "已保存提前结束的数据：$bag_dir"
  else
    echo "本次录制未生成有效 rosbag。"
  fi
  exit 130
}

trap on_interrupt INT TERM

setsid ros2 bag record \
  --output "$bag_dir" \
  --storage mcap \
  --custom-data "scene=$scene_name" \
  "distance_mm=$distance_mm" \
  "repeat_index=$repeat_index" \
  --topics /scan /tf /tf_static &
record_pid=$!

sleep "$duration_s"
stop_recorder
trap - INT TERM

if [[ ! -f "$bag_dir/metadata.yaml" ]]; then
  echo "错误：录制结束但没有生成 metadata.yaml，本次数据无效。"
  exit 1
fi

echo
ros2 bag info "$bag_dir"
echo
echo "录制完成。请编辑：$run_dir/scene.md"
