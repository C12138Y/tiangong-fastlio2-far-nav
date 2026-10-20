#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DESTINATION="${1:-/home/nvidia/hric_tools}"
STAMP="$(date +%Y%m%d_%H%M%S)"

mkdir -p "${DESTINATION}"

install_one() {
  local source_path="$1"
  local target_path="${DESTINATION}/$(basename "${source_path}")"
  if [[ -e "${target_path}" ]]; then
    cp -a "${target_path}" "${target_path}.bak_${STAMP}"
    printf 'Backup: %s\n' "${target_path}.bak_${STAMP}"
  fi
  install -m 0755 "${source_path}" "${target_path}"
  printf 'Installed: %s\n' "${target_path}"
}

install_one "${REPO_ROOT}/scripts/far_waypoint_shadow.py"
install_one "${REPO_ROOT}/scripts/far_single_waypoint_trial.py"
install_one "${REPO_ROOT}/scripts/far_waypoint_follower_trial.py"
install_one "${REPO_ROOT}/scripts/collect_robot_sources.sh"

for source_path in "${REPO_ROOT}"/scripts/stack/far_stack_*.sh; do
  install_one "${source_path}"
done

printf '\nInstalled integration tools to %s\n' "${DESTINATION}"
printf 'This installer did not modify FAST-LIO2, FAR, terrain, adapter, or HRIC vendor workspaces.\n'

