#!/bin/zsh
# Start: Position(2,018, 2,162, 0,000), Orientation(0,000, 0,000, 0,029, 1,000) = Angle: 0,059
# Goal:  Position(26,457, 17,616, 0,000), Orientation(0,000, 0,000, 0,639, 0,769) = Angle: 1,386

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 2.018, y: 2.162, theta: 0.059}
goal: {x: 26.457, y: 17.616, theta: 1.386}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 2.018, y: 2.162, theta: 0.059}
goal: {x: 26.457, y: 17.616, theta: 1.386}
mod_type: 0
upstream: false
weight_c: 0.02"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 2.018, y: 2.162, theta: 0.059}
goal: {x: 26.457, y: 17.616, theta: 1.386}
mod_type: 0
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 2.018, y: 2.162, theta: 0.059}
goal: {x: 26.457, y: 17.616, theta: 1.386}
mod_type: 2
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 2.018, y: 2.162, theta: 0.059}
goal: {x: 26.457, y: 17.616, theta: 1.386}
mod_type: 4
upstream: true
weight_c: 0.2"
