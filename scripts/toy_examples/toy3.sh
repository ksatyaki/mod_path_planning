#!/bin/zsh
# Position(1,284, 7,616, 0,000), Angle: 0,000
# Position(32,156, 7,302, 0,000), Angle: 0,000


rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 1.284, y: 7.302, theta: 0.0}
goal: {x: 32.156, y: 7.302, theta: 0.0}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 1.284, y: 7.302, theta: 0.0}
goal: {x: 32.156, y: 7.302, theta: 0.0}
mod_type: 0
upstream: false
weight_c: 0.02"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 1.284, y: 7.302, theta: 0.0}
goal: {x: 32.156, y: 7.302, theta: 0.0}
mod_type: 0
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 1.284, y: 7.302, theta: 0.0}
goal: {x: 32.156, y: 7.302, theta: 0.0}
mod_type: 2
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 1.284, y: 7.302, theta: 0.0}
goal: {x: 32.156, y: 7.302, theta: 0.0}
mod_type: 4
upstream: true
weight_c: 0.2"
