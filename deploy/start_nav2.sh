#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=20
CFG=~/ros2_ws/src/fpms_rover1/config/nav2_planner.yaml
setsid nohup ros2 run nav2_planner planner_server --ros-args \
  --params-file "$CFG" -r __node:=planner_server \
  > /tmp/nav2_planner.log 2>&1 &
sleep 6
setsid nohup ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p node_names:="['planner_server']" -p autostart:=true \
  -p bond_timeout:=0.0 -r __node:=lifecycle_manager_planner \
  > /tmp/nav2_lifecycle.log 2>&1 &
sleep 12
echo "--- planner_server ---"; tail -6 /tmp/nav2_planner.log
echo "--- lifecycle ---";      tail -4 /tmp/nav2_lifecycle.log
