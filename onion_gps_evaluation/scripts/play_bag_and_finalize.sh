#!/usr/bin/env bash

# Play one rosbag to completion, wait for subscriber callback queues to drain,
# and then call exactly one Onion GPS workflow's finalize service.
#
# Workflow-to-service mapping:
#   registration -> /onion_gps_trajectory_registration/finalize
#   evaluation   -> /onion_localization_accuracy_evaluation/finalize
#   segmented    -> /onion_segmented_registration_evaluation/finalize
#
# The matching evaluation launch must already be running before this script.

set -u

readonly DEFAULT_DRAIN_SEC="2"
readonly DEFAULT_SERVICE_WAIT_SEC="30"

bag_path=""
workflow=""
drain_sec="${DEFAULT_DRAIN_SEC}"
service_wait_sec="${DEFAULT_SERVICE_WAIT_SEC}"
rosbag_arguments=()
interrupted=0

print_usage() {
  cat <<'USAGE'
Usage:
  rosrun onion_gps_evaluation play_bag_and_finalize.sh \
    --bag BAG_PATH \
    --workflow registration|evaluation|segmented \
    [--drain-sec SECONDS] \
    [--service-wait-sec SECONDS] \
    [-- ROSBAG_PLAY_ARGUMENTS...]

Workflow mapping:
  registration
    /onion_gps_trajectory_registration/finalize
  evaluation
    /onion_localization_accuracy_evaluation/finalize
  segmented
    /onion_segmented_registration_evaluation/finalize

Examples:
  rosrun onion_gps_evaluation play_bag_and_finalize.sh \
    --bag /data/registration.bag \
    --workflow registration

  rosrun onion_gps_evaluation play_bag_and_finalize.sh \
    --bag /data/localization_evaluation.bag \
    --workflow evaluation \
    -- --clock -r 0.5

Behavior:
  1. Verify that the selected finalize service is available.
  2. Run rosbag play in the foreground with stdin detached.
  3. Do not finalize if playback is interrupted or exits with an error.
  4. After normal playback, wait two wall-clock seconds by default.
  5. Call the selected finalize service and check success=true.
USAGE
}

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

require_option_value() {
  local option_name="$1"
  local remaining_count="$2"
  if [[ "${remaining_count}" -lt 2 ]]; then
    fail "${option_name} requires a value"
  fi
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --bag)
      require_option_value "$1" "$#"
      bag_path="$2"
      shift 2
      ;;
    --workflow)
      require_option_value "$1" "$#"
      workflow="$2"
      shift 2
      ;;
    --drain-sec)
      require_option_value "$1" "$#"
      drain_sec="$2"
      shift 2
      ;;
    --service-wait-sec)
      require_option_value "$1" "$#"
      service_wait_sec="$2"
      shift 2
      ;;
    --)
      shift
      rosbag_arguments=("$@")
      break
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      fail "unknown argument: $1 (use --help)"
      ;;
  esac
done

[[ -n "${bag_path}" ]] || fail "--bag is required"
[[ -r "${bag_path}" ]] || fail "bag is not readable: ${bag_path}"
[[ -n "${workflow}" ]] || fail "--workflow is required"
[[ "${drain_sec}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
  fail "--drain-sec must be a non-negative number"
[[ "${service_wait_sec}" =~ ^[0-9]+$ ]] ||
  fail "--service-wait-sec must be a non-negative integer"

case "${workflow}" in
  registration)
    finalize_service="/onion_gps_trajectory_registration/finalize"
    ;;
  evaluation)
    finalize_service="/onion_localization_accuracy_evaluation/finalize"
    ;;
  segmented)
    finalize_service="/onion_segmented_registration_evaluation/finalize"
    ;;
  *)
    fail "unsupported workflow: ${workflow}; expected registration, evaluation, or segmented"
    ;;
esac

command -v rosbag >/dev/null 2>&1 ||
  fail "rosbag command is unavailable; source ROS Noetic first"
command -v rosservice >/dev/null 2>&1 ||
  fail "rosservice command is unavailable; source ROS Noetic first"

echo "Onion GPS rosbag playback"
echo "  bag:              ${bag_path}"
echo "  workflow:         ${workflow}"
echo "  finalize service: ${finalize_service}"
echo "  callback drain:   ${drain_sec} s"

service_ready=0
wait_start_sec="$(date +%s)"
while true; do
  if rosservice info "${finalize_service}" >/dev/null 2>&1; then
    service_ready=1
    break
  fi
  current_sec="$(date +%s)"
  if ((current_sec - wait_start_sec >= service_wait_sec)); then
    break
  fi
  sleep 0.2
done
if [[ "${service_ready}" -ne 1 ]]; then
  fail "finalize service did not appear within ${service_wait_sec} s: ${finalize_service}"
fi

echo "Starting rosbag play; keyboard pause input is disabled."
trap 'interrupted=1' INT TERM
rosbag play "${rosbag_arguments[@]}" "${bag_path}" </dev/null
play_status=$?
trap - INT TERM

if [[ "${interrupted}" -ne 0 ]]; then
  fail "rosbag playback was interrupted; finalize was not called"
fi
if [[ "${play_status}" -ne 0 ]]; then
  fail "rosbag play exited with status ${play_status}; finalize was not called"
fi

echo "Rosbag playback completed normally."
echo "Waiting ${drain_sec} s for ROS subscriber callbacks to drain."
sleep "${drain_sec}"

echo "Calling ${finalize_service}"
finalize_output="$(rosservice call "${finalize_service}" "{}" 2>&1)"
finalize_status=$?
printf '%s\n' "${finalize_output}"
if [[ "${finalize_status}" -ne 0 ]]; then
  fail "rosservice call failed with status ${finalize_status}"
fi
if ! grep -Eq '^success: (True|true)$' <<<"${finalize_output}"; then
  fail "finalize service returned success=false"
fi

echo "Onion GPS evaluation report finalized successfully."
