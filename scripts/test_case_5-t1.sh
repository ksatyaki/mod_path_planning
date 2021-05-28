#!/bin/zsh

# START: Frame:map, Position(-0.133, -8.555, 0.000), Orientation(0.000, 0.000, 0.964, 0.266) = Angle: 2.604
# GOAL: Frame:map, Position(31.093, -20.553, 0.000), Orientation(0.000, 0.000, -0.841, 0.541) = Angle: -1.999


rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: -0.133, y: -8.555, theta: 2.604}
goal: {x: 31.093, y: -20.553, theta: -1.999}
mod_type: 0
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: -0.133, y: -8.555, theta: 2.604}
goal: {x: 31.093, y: -20.553, theta: -1.999}
mod_type: 1
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: -0.133, y: -8.555, theta: 2.604}
goal: {x: 31.093, y: -20.553, theta: -1.999}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: -0.133, y: -8.555, theta: 2.604}
goal: {x: 31.093, y: -20.553, theta: -1.999}
mod_type: 0
upstream: false
weight_c: 0.02"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: -0.133, y: -8.555, theta: 2.604}
goal: {x: 31.093, y: -20.553, theta: -1.999}
mod_type: 2
upstream: true
weight_c: 0.1"