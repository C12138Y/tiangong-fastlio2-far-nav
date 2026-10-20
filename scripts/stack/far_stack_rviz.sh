#!/usr/bin/env bash
set -Eeo pipefail

# shellcheck disable=SC1091
source /home/nvidia/hric_tools/hric_env.sh
source /home/nvidia/fastlio2_ws/install/setup.bash
source /home/nvidia/tiangong_nav_ws/install/setup.bash
source /home/nvidia/far_planner_eval_ws/far_planner/install/setup.bash
unset ROS_DOMAIN_ID

RVIZ_CONFIG="$(ros2 pkg prefix --share fastlio2)/rviz/fastlio2.rviz"
exec rviz2 -d "$RVIZ_CONFIG"

