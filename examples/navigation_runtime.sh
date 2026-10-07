#!/usr/bin/env bash
# Inside the released Navigation v0.1 container; /community is this checkout.
set -eo pipefail
cd /community
source /opt/ros/humble/setup.bash
source /opt/nav3d_ws/install/setup.bash
ros2 launch /community/examples/navigation_topics.launch.py > /artifacts/navigation-runtime.log 2>&1 &
runtime_pid=$!
# Background launch can inherit ignored SIGINT. Terminate it on exit without
# an unbounded wait; the container lifecycle also reaps all remaining nodes.
trap 'kill -TERM "$runtime_pid" 2>/dev/null || true' EXIT
python3 -m community.navigation_ros2 --output /artifacts/navigation-trace.json "$@"
