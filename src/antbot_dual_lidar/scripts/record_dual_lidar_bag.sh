#!/usr/bin/env bash
set -euo pipefail

bag_root="${1:-datasets/bags}"
case_name="${2:-manual}"
case "${case_name}" in
  01_static|02_left_rotation|03_right_rotation|04_wall_motion|05_column_motion) ;;
  01_static|02_forward|03_backward|04_strafe_left|05_strafe_right|06_yaw_left|07_yaw_right|08_diagonal|09_forward_yaw|10_strafe_yaw|11_stop_go|12_reset_recovery|manual) ;;
  *)
    echo "Unknown dataset case: ${case_name}" >&2
    exit 2
    ;;
esac

mkdir -p "${bag_root}"
bag_path="${bag_root%/}/${case_name}_$(date +%Y%m%d_%H%M%S)"
metadata_path="${bag_path}.capture.yaml"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"

topics=(
  /clock
  /antbot/lidar/front_left/points_raw_native
  /antbot/lidar/rear_right/points_raw_native
  /antbot/lidar/front_left/points_lio
  /antbot/lidar/rear_right/points_lio
  /antbot/lidar/front_left/points_deskew_truth
  /antbot/lidar/rear_right/points_deskew_truth
  /antbot/lidar/front_left/points_world_reference
  /antbot/lidar/rear_right/points_world_reference
  /antbot/imu/data_raw
  /antbot/imu/data
  /antbot/ground_truth/odom
  /odom
  /tf
  /tf_static
  /joint_states
  /cmd_vel
)

if [[ "${ANTBOT_RECORD_AUX_IMU:-0}" == "1" ]]; then
  topics+=(
    /antbot/lidar/front_left/imu
    /antbot/lidar/rear_right/imu
  )
fi

clock_sample() {
  timeout 3 ros2 topic echo --once /clock rosgraph_msgs/msg/Clock --field clock 2>/dev/null \
    | tr '\n' ' ' | sed 's/[[:space:]]\\+/ /g' || true
}

start_sim_clock="$(clock_sample)"
git_revision="$(git -C "${project_root}" rev-parse HEAD 2>/dev/null || echo unavailable)"
git_dirty="$(git -C "${project_root}" status --porcelain -- antbot 2>/dev/null | wc -l)"
usd_path="${ANTBOT_DATASET_USD:-${project_root}/antbot/isaac/usd/antbot_dual_lidar_navigation.usd}"
imu_config="${project_root}/antbot/isaac/config/imu_profiles.yaml"
lio_config="${project_root}/antbot/ros2_ws/src/antbot_dual_lidar/config/lio_input_contract.yaml"

{
  echo "schema_version: 1"
  echo "case: ${case_name}"
  echo "bag_path: ${bag_path}"
  echo "storage: mcap"
  echo "lidar_profile: navigation"
  echo "imu_profile: ${ANTBOT_IMU_PROFILE:-ideal}"
  echo "pre_roll_sec: ${ANTBOT_PRE_ROLL_SEC:-1.2}"
  echo "post_roll_sec: ${ANTBOT_POST_ROLL_SEC:-1.2}"
  echo "git_revision: ${git_revision}"
  echo "git_dirty_path_count: ${git_dirty}"
  echo "start_sim_clock: \"${start_sim_clock}\""
  echo "hashes:"
  for path in "${usd_path}" "${imu_config}" "${lio_config}"; do
    if [[ -f "${path}" ]]; then
      echo "  $(basename "${path}"): $(sha256sum "${path}" | awk '{print $1}')"
    else
      echo "  $(basename "${path}"): unavailable"
    fi
  done
} > "${metadata_path}"

echo "Recording Phase 2A MCAP: ${bag_path}"
set +e
if [[ "${case_name}" =~ ^0[1-5]_ ]]; then
  pre_roll="${ANTBOT_PRE_ROLL_SEC:-1.2}"
  post_roll="${ANTBOT_POST_ROLL_SEC:-1.2}"
  action_sec="${ANTBOT_ACTION_SEC:-4.0}"
  linear_x=0.0
  angular_z=0.0
  case "${case_name}" in
    02_left_rotation) angular_z="${ANTBOT_YAW_RATE:-0.8}" ;;
    03_right_rotation) angular_z="-${ANTBOT_YAW_RATE:-0.8}" ;;
    04_wall_motion) linear_x="${ANTBOT_LINEAR_SPEED:-0.25}"; angular_z="${ANTBOT_YAW_RATE:-0.8}" ;;
    05_column_motion) angular_z="${ANTBOT_YAW_RATE:-0.8}" ;;
  esac
  ros2 bag record --storage mcap --output "${bag_path}" "${topics[@]}" &
  recorder_pid=$!
  sleep "${pre_roll}"
  timeout --signal=TERM --kill-after=2 "${action_sec}" ros2 topic pub -r 20 /cmd_vel \
    geometry_msgs/msg/Twist \
    "{linear: {x: ${linear_x}}, angular: {z: ${angular_z}}}" >/dev/null 2>&1
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
  sleep "${post_roll}"
  kill -TERM "${recorder_pid}" 2>/dev/null
  wait "${recorder_pid}"
  record_status=$?
else
  ros2 bag record --storage mcap --output "${bag_path}" "${topics[@]}"
  record_status=$?
fi
set -e

end_sim_clock="$(clock_sample)"
echo "end_sim_clock: \"${end_sim_clock}\"" >> "${metadata_path}"

if [[ -d "${bag_path}" ]]; then
  ros2 bag info "${bag_path}" | tee "${bag_path}.info.txt"
  if ros2 run antbot_dual_lidar validate_lio_dataset \
      "${bag_path}" --capture-metadata "${metadata_path}"; then
    echo "Dataset validation completed."
  else
    echo "Dataset validation failed; bag has been retained for diagnosis." >&2
    [[ "${record_status}" -ne 0 ]] || record_status=3
  fi
else
  echo "No bag directory was created." >&2
fi
exit "${record_status}"
