#!/usr/bin/env bash
set -e -o pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$workspace/install/wla_manual_control/share/wla_manual_control/config/cyclonedds-local.xml}"

if [[ "${1:-}" != follow ]]; then
  exec /usr/bin/python3 "$workspace/person_follow_trial.py" "$@"
fi

log_directory="$workspace/.runtime/follow-trials"
mkdir -p "$log_directory"
log_file="$log_directory/follow-$(date +%Y%m%d-%H%M%S)-$$.log"
: > "$log_file"
printf '本次跟随终端日志：%s\n' "$log_file"

# Pipe stdout/stderr only: stdin remains the operator's terminal for ID selection.
# Ignore SIGINT in tee so the client's Ctrl+C stop acknowledgement is also saved.
set +e
{
  printf 'Started: %s\nCommand:' "$(date --iso-8601=seconds)"
  printf ' %q' "$0" "$@"
  printf '\n'
  /usr/bin/python3 -u "$workspace/person_follow_trial.py" "$@"
  trial_status=$?
  printf '\nFinished: %s; client_exit=%s\n' "$(date --iso-8601=seconds)" "$trial_status"
  exit "$trial_status"
} 2>&1 | tee -i -a "$log_file"
pipeline_status=("${PIPESTATUS[@]}")
if (( pipeline_status[0] != 0 )); then
  exit "${pipeline_status[0]}"
fi
exit "${pipeline_status[1]}"
