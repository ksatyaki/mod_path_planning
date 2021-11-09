#!/bin/zsh
# START ( 2.233, 1.916, Angle: 1.226)
# GOAL  (22.968, 18.647, Angle: 0.785)

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 2.233, y: 1.9, theta: 1.226}
goal: {x: 22.968, y: 19.0, theta: 0.785}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 2.233, y: 1.9, theta: 1.226}
goal: {x: 22.968, y: 19.0, theta: 0.785}
mod_type: 0
upstream: false
weight_c: 0.02"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 2.233, y: 1.9, theta: 1.226}
goal: {x: 22.968, y: 19.0, theta: 0.785}
mod_type: 0
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 2.233, y: 1.9, theta: 1.226}
goal: {x: 22.968, y: 19.0, theta: 0.785}
mod_type: 2
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 2.233, y: 1.9, theta: 1.226}
goal: {x: 22.968, y: 19.0, theta: 0.785}
mod_type: 4
upstream: true
weight_c: 0.2"
