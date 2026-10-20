#!/usr/bin/env bash
# Recreate the three source workspaces at the robot-verified commits.
# Existing source directories are never overwritten.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORK_ROOT="${1:-${HOME}}"

FAST_WS="${WORK_ROOT}/fastlio2_ws"
NAV_WS="${WORK_ROOT}/tiangong_nav_ws"
FAR_WS="${WORK_ROOT}/far_planner_eval_ws"

FAST_REPO="${FAST_WS}/src/FASTLIO2_ROS2_http11"
ADAPTER_DST="${NAV_WS}/src/fastlio_nav_adapter"
CMU_REPO="${FAR_WS}/cmu_dev_jazzy"
FAR_REPO="${FAR_WS}/far_planner"

for path in "${FAST_REPO}" "${ADAPTER_DST}" "${CMU_REPO}" "${FAR_REPO}"; do
  if [[ -e "${path}" ]]; then
    printf 'Refusing to overwrite existing path: %s\n' "${path}" >&2
    exit 1
  fi
done

mkdir -p "${FAST_WS}/src" "${NAV_WS}/src" "${FAR_WS}"

git -c http.version=HTTP/1.1 clone \
  https://github.com/liangheming/FASTLIO2_ROS2.git "${FAST_REPO}"
git -C "${FAST_REPO}" checkout 7baa59f7339ba88ce3b646fe35b23f61031c5d3f
git -C "${FAST_REPO}" apply --check "${REPO_ROOT}/patches/fastlio2.patch"
git -C "${FAST_REPO}" apply "${REPO_ROOT}/patches/fastlio2.patch"
install -m 0644 "${REPO_ROOT}/extensions/fastlio2/pc2_to_custom.cpp" \
  "${FAST_REPO}/fastlio2/src/pc2_to_custom.cpp"

cp -a "${REPO_ROOT}/ros2_packages/fastlio_nav_adapter" "${ADAPTER_DST}"

git -c http.version=HTTP/1.1 clone \
  https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git \
  "${CMU_REPO}"
git -C "${CMU_REPO}" checkout 8313dfed10533787582be8a6044483fe3299622f
git -C "${CMU_REPO}" apply --check "${REPO_ROOT}/patches/terrain_analysis.patch"
git -C "${CMU_REPO}" apply "${REPO_ROOT}/patches/terrain_analysis.patch"

git -c http.version=HTTP/1.1 clone \
  https://github.com/MichaelFYang/far_planner.git "${FAR_REPO}"
git -C "${FAR_REPO}" checkout 29fb215886df4b0c53c27f487c51041f808c5eba

printf '\nSource workspaces recreated under: %s\n' "${WORK_ROOT}"
printf 'Sophus installation and colcon build steps are documented in docs/01 and docs/02.\n'

