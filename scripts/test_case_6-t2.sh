#!/bin/zsh


rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352883600, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 11.5, y: -5.0, theta: 2.604}
goal: {x: -25.00, y: 3.00, theta: -1.999}
mod_type: 1
upstream: true
weight_c: 0.1"
