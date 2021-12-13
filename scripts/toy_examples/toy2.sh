#!/bin/zsh
# Position(33,869, 2,057, 0,000), Angle: 1,648
# Position(8,871, 21,916, 0,000), Angle: 1,787

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 33.869, y: 2.057, theta: 1.648}
goal: {x: 8.871, y: 21.916, theta: 1.787}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 33.869, y: 2.057, theta: 1.648}
goal: {x: 8.871, y: 21.916, theta: 1.787}
mod_type: 0
upstream: false
weight_c: 0.02"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 33.869, y: 2.057, theta: 1.648}
goal: {x: 8.871, y: 21.916, theta: 1.787}
mod_type: 0
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 33.869, y: 2.057, theta: 1.648}
goal: {x: 8.871, y: 21.916, theta: 1.787}
mod_type: 2
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 33.869, y: 2.057, theta: 1.648}
goal: {x: 8.871, y: 21.916, theta: 1.787}
mod_type: 4
upstream: true
weight_c: 0.2"
