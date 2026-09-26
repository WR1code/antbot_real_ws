#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 --stage A|B|C|D|E [--duration 60] [--output-root benchmark]"
  echo "Drivers must already be running for stages B-E. C/D/E start the fusion node."
}

stage="E"
duration=60
output_root="benchmark"
front_topic="/antbot/lidar/front_left/points_raw_native"
rear_topic="/antbot/lidar/rear_right/points_raw_native"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --stage) stage="${2^^}"; shift 2 ;;
    --duration) duration="$2"; shift 2 ;;
    --output-root) output_root="$2"; shift 2 ;;
    --front-topic) front_topic="$2"; shift 2 ;;
    --rear-topic) rear_topic="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$stage" =~ ^[ABCDE]$ ]] || { echo "stage must be A, B, C, D, or E" >&2; exit 2; }
[[ "$duration" =~ ^[0-9]+$ ]] && (( duration > 0 )) || {
  echo "duration must be a positive integer" >&2; exit 2;
}

timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${output_root%/}/${timestamp}_${stage}"
mkdir -p "$output_dir"
system_csv="$output_dir/system.csv"
process_csv="$output_dir/process.csv"
topics_csv="$output_dir/topics.csv"
network_csv="$output_dir/network.csv"
latency_csv="$output_dir/latency.csv"
tegrastats_log="$output_dir/tegrastats.log"

echo "timestamp,total_cpu_pct,per_core_cpu_pct,ram_used_kb,ram_total_kb" > "$system_csv"
echo "timestamp,kind,pid,cpu_pct,rss_kb" > "$process_csv"
echo "timestamp,topic,metric,value,unit" > "$topics_csv"
echo "timestamp,interface,rx_bytes_per_sec,tx_bytes_per_sec" > "$network_csv"

background_pids=()
fusion_pid=""
cleanup() {
  for pid in "${background_pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  if [[ -n "$fusion_pid" ]]; then
    kill -INT "$fusion_pid" 2>/dev/null || true
    wait "$fusion_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$stage" != "A" ]]; then
  for topic in "$front_topic" "$rear_topic"; do
    if ! timeout 5 ros2 topic type "$topic" 2>/dev/null | grep -qx 'sensor_msgs/msg/PointCloud2'; then
      echo "Required PointCloud2 topic is not live: $topic" >&2
      exit 1
    fi
  done
fi

case "$stage" in
  C) processing_stage="crop" ;;
  D) processing_stage="transform" ;;
  E) processing_stage="merge" ;;
  *) processing_stage="" ;;
esac

if [[ -n "$processing_stage" ]]; then
  ros2 launch antbot_mid360_fusion dual_mid360_fusion.launch.py \
    stage:="$processing_stage" front_topic:="$front_topic" rear_topic:="$rear_topic" \
    metrics_csv:="$latency_csv" use_rviz:=false > "$output_dir/fusion.log" 2>&1 &
  fusion_pid=$!
  sleep 2
  kill -0 "$fusion_pid" 2>/dev/null || {
    echo "Fusion node failed to start; inspect $output_dir/fusion.log" >&2
    exit 1
  }
else
  echo "stamp_ns,stage,front_crop_ms,rear_crop_ms,front_tf_ms,rear_tf_ms,merge_ms,total_ms,front_input_points,rear_input_points,front_kept_points,rear_kept_points,sync_success,sync_miss,dropped_clouds" > "$latency_csv"
fi

sample_system() {
  declare -A previous_total previous_idle
  while true; do
    now="$(date +%s.%N)"
    per_core=""
    total_cpu="0"
    while read -r label user nice system idle iowait irq softirq steal _; do
      current_total=$((user + nice + system + idle + iowait + irq + softirq + steal))
      current_idle=$((idle + iowait))
      if [[ -n "${previous_total[$label]:-}" ]]; then
        delta_total=$((current_total - previous_total[$label]))
        delta_idle=$((current_idle - previous_idle[$label]))
        value="$(awk -v t="$delta_total" -v i="$delta_idle" 'BEGIN {printf "%.2f", t ? 100*(t-i)/t : 0}')"
        if [[ "$label" == "cpu" ]]; then
          total_cpu="$value"
        else
          per_core+="${label#cpu}:$value;"
        fi
      fi
      previous_total[$label]=$current_total
      previous_idle[$label]=$current_idle
    done < <(awk '/^cpu/ {print}' /proc/stat)
    read -r ram_total ram_available < <(awk '
      /^MemTotal:/ {total=$2} /^MemAvailable:/ {available=$2}
      END {print total, available}' /proc/meminfo)
    echo "$now,$total_cpu,$per_core,$((ram_total - ram_available)),$ram_total" >> "$system_csv"
    sleep 1
  done
}

sample_processes() {
  while true; do
    now="$(date +%s.%N)"
    while read -r pid; do
      [[ -n "$pid" ]] || continue
      read -r cpu rss < <(ps -p "$pid" -o %cpu=,rss= 2>/dev/null || echo "0 0")
      echo "$now,fusion,$pid,${cpu:-0},${rss:-0}" >> "$process_csv"
    done < <(pgrep -f 'dual_mid360_fusion_node' || true)
    while read -r pid; do
      [[ -n "$pid" ]] || continue
      read -r cpu rss < <(ps -p "$pid" -o %cpu=,rss= 2>/dev/null || echo "0 0")
      echo "$now,livox_driver,$pid,${cpu:-0},${rss:-0}" >> "$process_csv"
    done < <(pgrep -f 'livox_ros_driver2_node' || true)
    sleep 1
  done
}

sample_network() {
  declare -A previous_rx previous_tx
  while true; do
    now="$(date +%s.%N)"
    while read -r interface rx tx; do
      if [[ -n "${previous_rx[$interface]:-}" ]]; then
        echo "$now,$interface,$((rx - previous_rx[$interface])),$((tx - previous_tx[$interface]))" >> "$network_csv"
      fi
      previous_rx[$interface]=$rx
      previous_tx[$interface]=$tx
    done < <(awk -F'[: ]+' 'NR>2 {print $2,$3,$11}' /proc/net/dev)
    sleep 1
  done
}

monitor_hz() {
  local topic="$1"
  timeout "$duration" ros2 topic hz "$topic" --window 20 2>/dev/null | while read -r line; do
    if [[ "$line" =~ average[[:space:]]rate:[[:space:]]([0-9.]+) ]]; then
      echo "$(date +%s.%N),$topic,hz,${BASH_REMATCH[1]},Hz" >> "$topics_csv"
    fi
  done
}

monitor_bw() {
  local topic="$1"
  timeout "$duration" ros2 topic bw "$topic" --window 20 2>/dev/null | while read -r line; do
    if [[ "$line" =~ average:[[:space:]]([0-9.]+)([KMGT]?B)/s ]]; then
      value="${BASH_REMATCH[1]}"
      unit="${BASH_REMATCH[2]}"
      value_mb="$(awk -v v="$value" -v u="$unit" 'BEGIN {
        if (u=="B") v=v/1000000; else if (u=="KB") v=v/1000;
        else if (u=="GB") v=v*1000; else if (u=="TB") v=v*1000000;
        printf "%.6f",v}')"
      echo "$(date +%s.%N),$topic,bandwidth,$value_mb,MB/s" >> "$topics_csv"
    fi
  done
}

sample_system & background_pids+=("$!")
sample_processes & background_pids+=("$!")
sample_network & background_pids+=("$!")

if command -v tegrastats >/dev/null 2>&1; then
  tegrastats --interval 1000 > "$tegrastats_log" 2>&1 &
  background_pids+=("$!")
else
  echo "tegrastats not found" > "$tegrastats_log"
fi

if [[ "$stage" != "A" ]]; then
  topics=("$front_topic" "$rear_topic")
  [[ "$stage" =~ ^[CDE]$ ]] && topics+=("/mid360/front/filtered" "/mid360/rear/filtered")
  [[ "$stage" == "E" ]] && topics+=("/mid360/merged")
  for topic in "${topics[@]}"; do
    monitor_hz "$topic" & background_pids+=("$!")
    monitor_bw "$topic" & background_pids+=("$!")
  done
fi

echo "Benchmark stage $stage running for ${duration}s; output: $output_dir"
sleep "$duration"
cleanup
trap - EXIT INT TERM

average_column() {
  local file="$1" column="$2" condition="${3:-1}"
  awk -F, -v c="$column" -v condition="$condition" '
    NR>1 && (condition=="1" || $2==condition) && $c!="" {sum+=$c; n++}
    END {printf "%.2f", n ? sum/n : 0}' "$file"
}

maximum_column() {
  local file="$1" column="$2" condition="${3:-1}"
  awk -F, -v c="$column" -v condition="$condition" '
    NR>1 && (condition=="1" || $2==condition) && $c>max {max=$c}
    END {printf "%.2f", max}' "$file"
}

latency_stat() {
  local percentile="$1"
  awk -F, 'NR>1 && $8!="" {print $8}' "$latency_csv" | sort -n | awk -v p="$percentile" '
    {value[NR]=$1} END {if (!NR) {printf "0.00"} else {i=int(p*NR+0.999999); printf "%.2f", value[i]}}'
}

if [[ -r /proc/device-tree/model ]]; then
  platform="$(tr -d '\0' < /proc/device-tree/model)"
else
  platform="$(uname -m)"
fi
ram_total_kb="$(awk -F, 'NR==2 {print $5}' "$system_csv")"
ram_peak_kb="$(maximum_column "$system_csv" 4)"
fusion_cpu="$(average_column "$process_csv" 4 fusion)"
fusion_rss_kb="$(maximum_column "$process_csv" 5 fusion)"
total_average="$(awk -F, 'NR>1 && $8!="" {s+=$8;n++} END {printf "%.2f",n?s/n:0}' "$latency_csv")"
total_max="$(maximum_column "$latency_csv" 8)"
last_counts="$(awk -F, 'NR>1 {v=$13","$14","$15} END {print v}' "$latency_csv")"
IFS=, read -r sync_success sync_miss dropped_clouds <<< "${last_counts:-0,0,0}"

topic_average() {
  local topic="$1" metric="$2"
  awk -F, -v t="$topic" -v m="$metric" '
    $2==t && $3==m {sum+=$4;n++} END {printf "%.2f",n?sum/n:0}' "$topics_csv"
}

front_input_pps="$(awk -F, -v d="$duration" 'NR>1 {s+=$9} END {printf "%.0f",s/d}' "$latency_csv")"
rear_input_pps="$(awk -F, -v d="$duration" 'NR>1 {s+=$10} END {printf "%.0f",s/d}' "$latency_csv")"
front_kept_pps="$(awk -F, -v d="$duration" 'NR>1 {s+=$11} END {printf "%.0f",s/d}' "$latency_csv")"
rear_kept_pps="$(awk -F, -v d="$duration" 'NR>1 {s+=$12} END {printf "%.0f",s/d}' "$latency_csv")"
front_keep_ratio="$(awk -F, 'NR>1 {i+=$9;k+=$11} END {printf "%.2f",i?100*k/i:0}' "$latency_csv")"
rear_keep_ratio="$(awk -F, 'NR>1 {i+=$10;k+=$12} END {printf "%.2f",i?100*k/i:0}' "$latency_csv")"
network_mb="$(awk -F, 'NR>1 {s+=$3+$4;n++} END {printf "%.2f",n?s/n/1000000:0}' "$network_csv")"

cat > "$output_dir/summary.txt" <<EOF
========= RESULT =========

Stage: $stage
Duration: ${duration} s
Platform: $platform

Raw dual MID360:
Front: $(topic_average "$front_topic" hz) Hz, $(topic_average "$front_topic" bandwidth) MB/s, $front_input_pps processed pts/s
Rear: $(topic_average "$rear_topic" hz) Hz, $(topic_average "$rear_topic" bandwidth) MB/s, $rear_input_pps processed pts/s

After crop:
Front: $front_kept_pps pts/s, keep ratio $front_keep_ratio %
Rear: $rear_kept_pps pts/s, keep ratio $rear_keep_ratio %

Merged:
$(topic_average "/mid360/merged" hz) Hz, $(topic_average "/mid360/merged" bandwidth) MB/s, $((front_kept_pps + rear_kept_pps)) pts/s

Resource:
Total CPU average: $(average_column "$system_csv" 2) %
Total CPU peak: $(maximum_column "$system_csv" 2) %
Fusion node: $fusion_cpu % CPU, $(awk -v k="$fusion_rss_kb" 'BEGIN {printf "%.1f",k/1024}') MB peak RSS
RAM peak: $(awk -v k="$ram_peak_kb" 'BEGIN {printf "%.2f",k/1024/1024}') / $(awk -v k="$ram_total_kb" 'BEGIN {printf "%.2f",k/1024/1024}') GiB
Network RX+TX average across sampled interfaces: $network_mb MB/s

Latency:
average $total_average ms
P50 $(latency_stat 0.50) ms
P95 $(latency_stat 0.95) ms
P99 $(latency_stat 0.99) ms
max $total_max ms

Sync successes: ${sync_success:-0}
Sync misses: ${sync_miss:-0}
Dropped clouds: ${dropped_clouds:-0}

Topic rates/bandwidth are recorded in topics.csv; detailed point rates and keep ratios are also in fusion.log.
Conclusion: inspect measured CSV/log values; this script does not estimate unmeasured capacity.
EOF

echo "Completed: $output_dir"
