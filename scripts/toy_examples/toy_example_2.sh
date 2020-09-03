#!/bin/zsh

# Setting goal: Frame:map, Position(29,369, 10,420, 0,000), Orientation(0,000, 0,000, 0,729, 0,685) = Angle: 1,633
# Setting goal: Frame:map, Position(29,097, 22,473, 0,000), Orientation(0,000, 0,000, 0,727, 0,687) = Angle: 1,628




rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 29,369, y: 10.420, theta: 1.632}
goal: {x: 29,369, y: 22.589, theta: 1.632}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 5.731, y: 10.420, theta: 1.632}
goal: {x: 4.876, y: 22.589, theta: 1.632}
mod_type: 0
upstream: false
weight_c: 0.02"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 29,369, y: 10.420, theta: 1.632}
goal: {x: 29,369, y: 22.589, theta: 1.632}
mod_type: 0
upstream: true
weight_c: 0.1"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 29,369, y: 10.420, theta: 1.632}
goal: {x: 29,369, y: 22.589, theta: 1.632}
mod_type: 2
upstream: true
weight_c: 0.1"
