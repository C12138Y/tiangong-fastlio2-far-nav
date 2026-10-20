#!/usr/bin/env bash
# Collect the exact robot-side FAST-LIO2/FAR source state without modifying it.
set -Eeuo pipefail

STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT_ROOT="${HOME}/fastlio2_far_source_snapshot_${STAMP}"
ARCHIVE_PATH="${SNAPSHOT_ROOT}.tar.gz"

mkdir -p "${SNAPSHOT_ROOT}/sources" "${SNAPSHOT_ROOT}/metadata"

copy_path() {
  local source_path="$1"
  local target_name="$2"
  if [[ -e "${source_path}" ]]; then
    cp -a "${source_path}" "${SNAPSHOT_ROOT}/sources/${target_name}"
    printf 'COPIED %s\n' "${source_path}"
  else
    printf 'MISSING %s\n' "${source_path}" | tee -a "${SNAPSHOT_ROOT}/metadata/missing.txt"
  fi
}

capture_git() {
  local repo_path="$1"
  local label="$2"
  if [[ -d "${repo_path}/.git" ]]; then
    git -C "${repo_path}" rev-parse HEAD >"${SNAPSHOT_ROOT}/metadata/${label}_commit.txt"
    git -C "${repo_path}" status --short >"${SNAPSHOT_ROOT}/metadata/${label}_status.txt"
    git -C "${repo_path}" --no-pager diff >"${SNAPSHOT_ROOT}/metadata/${label}_working.patch"
    git -C "${repo_path}" --no-pager diff --cached >"${SNAPSHOT_ROOT}/metadata/${label}_staged.patch"
  else
    printf 'NOT_A_GIT_REPO %s\n' "${repo_path}" | tee -a "${SNAPSHOT_ROOT}/metadata/missing.txt"
  fi
}

copy_path "${HOME}/fastlio2_ws/src/FASTLIO2_ROS2_http11/fastlio2" "fastlio2"
copy_path "${HOME}/tiangong_nav_ws/src/fastlio_nav_adapter" "fastlio_nav_adapter"
copy_path "${HOME}/far_planner_eval_ws/cmu_dev_jazzy/src/terrain_analysis" "terrain_analysis"
copy_path "${HOME}/far_planner_eval_ws/cmu_dev_jazzy/src/terrain_analysis_ext" "terrain_analysis_ext"
copy_path "${HOME}/far_planner_eval_ws/far_planner/src" "far_planner_src"

mkdir -p "${SNAPSHOT_ROOT}/sources/hric_tools"
for source_path in "${HOME}"/hric_tools/far_*.py "${HOME}"/hric_tools/far_stack_*.sh; do
  if [[ -e "${source_path}" ]]; then
    cp -a "${source_path}" "${SNAPSHOT_ROOT}/sources/hric_tools/"
  fi
done

capture_git "${HOME}/fastlio2_ws/src/FASTLIO2_ROS2_http11" "fastlio2"
capture_git "${HOME}/tiangong_nav_ws" "tiangong_nav_ws"
capture_git "${HOME}/far_planner_eval_ws/cmu_dev_jazzy" "cmu_dev_jazzy"
capture_git "${HOME}/far_planner_eval_ws/far_planner" "far_planner"

{
  printf 'date='; date --iso-8601=seconds
  printf 'hostname='; hostname
  printf 'uname='; uname -a
  printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-unset}"
  printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-unset}"
} >"${SNAPSHOT_ROOT}/metadata/environment.txt"

find "${SNAPSHOT_ROOT}/sources" -type f -print0 \
  | sort -z \
  | xargs -0 sha256sum >"${SNAPSHOT_ROOT}/metadata/SHA256SUMS"

tar -C "$(dirname "${SNAPSHOT_ROOT}")" -czf "${ARCHIVE_PATH}" "$(basename "${SNAPSHOT_ROOT}")"

printf '\nSnapshot directory: %s\n' "${SNAPSHOT_ROOT}"
printf 'Archive: %s\n' "${ARCHIVE_PATH}"
printf 'Copy the archive into D:\\Codex\\nav\\outputs\\robot_source_snapshots\\ on the Windows workstation.\n'

