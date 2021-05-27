#!/bin/zsh


#[ INFO] [1586214884.514785968]: Setting goal: Frame:map, Position(-17,000, -8,656, 0,000), Orientation(0,000, 0,000, 0,562, 0,827) = Angle: 1,194

#[ INFO] [1586214896.230579540]: Setting goal: Frame:map, Position(-3,493, 7,846, 0,000), Orientation(0,000, 0,000, 0,393, 0,919) = Angle: 0,809


rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: -17.00, y: -8.656, theta: 1.194}
goal: {x: -3.493, y: 7.846, theta: 0.809}
mod_type: 0
upstream: false
weight_c: 0.000"


rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: -17.00, y: -8.656, theta: 1.194}
goal: {x: -3.493, y: 7.846, theta: 0.809}
mod_type: 0
upstream: false
weight_c: 0.02"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: -17.00, y: -8.656, theta: 1.194}
goal: {x: -3.493, y: 7.846, theta: 0.809}
mod_type: 0
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: -17.00, y: -8.656, theta: 1.194}
goal: {x: -3.493, y: 7.846, theta: 0.809}
mod_type: 1
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: -17.00, y: -8.656, theta: 1.194}
goal: {x: -3.493, y: 7.846, theta: 0.809}
mod_type: 2
upstream: true
weight_c: 0.1"
