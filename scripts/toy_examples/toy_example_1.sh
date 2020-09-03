#!/bin/zsh

# Setting goal: Frame:map, Position(5,731, 4,705, 0,000), Orientation(0,000, 0,000, 0,728, 0,685) = Angle: 1,632
# Setting goal: Frame:map, Position(4,876, 22,589, 0,000), Orientation(0,000, 0,000, 0,730, 0,684) = Angle: 1,635

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 5.731, y: 4.705, theta: 1.632}
goal: {x: 4.876, y: 22.589, theta: 1.632}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 5.731, y: 4.705, theta: 1.632}
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
start: {x: 5.731, y: 4.705, theta: 1.632}
goal: {x: 4.876, y: 22.589, theta: 1.632}
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
start: {x: 5.731, y: 4.705, theta: 1.632}
goal: {x: 4.876, y: 22.589, theta: 1.632}
mod_type: 2
upstream: true
weight_c: 0.1"
