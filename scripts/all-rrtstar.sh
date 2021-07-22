#!/bin/zsh

#Setting goal: Frame:map, Position(47.690, -18.848, 0.000), Orientation(0.000, 0.000, 0.924, -0.383) = Angle: -2.356
#Setting goal: Frame:map, Position(-19,575, 12,390, 0,000), Orientation(0,000, 0,000, 0,915, 0,403) = Angle: 2,313


rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 47.690, y: -18.848, theta: -2.356}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 47.690, y: -18.848, theta: -2.356}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 0
upstream: true
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 47.690, y: -18.848, theta: -2.356}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 1
upstream: true
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 47.690, y: -18.848, theta: -2.356}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 2
upstream: true
weight_c: 0.00"

###

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
goal: {x: -3.85, y: -6.82, theta: -0.8203}
start: {x: 21.471, y: -17.647, theta: 0.733}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
goal: {x: -3.85, y: -6.82, theta: -0.8203}
start: {x: 21.471, y: -17.647, theta: 0.733}
mod_type: 0
upstream: true
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
goal: {x: -3.85, y: -6.82, theta: -0.8203}
start: {x: 21.471, y: -17.647, theta: 0.733}
mod_type: 1
upstream: true
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
goal: {x: -3.85, y: -6.82, theta: -0.8203}
start: {x: 21.471, y: -17.647, theta: 0.733}
mod_type: 2
upstream: true
weight_c: 0.00"

###

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
upstream: true
weight_c: 0.00"

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
weight_c: 0.00"

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
weight_c: 0.00"

###

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 11.5, y: -5.0, theta: 2.604}
goal: {x: -25.00, y: 3.00, theta: -1.999}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 11.5, y: -5.0, theta: 2.604}
goal: {x: -25.00, y: 3.00, theta: -1.999}
mod_type: 0
upstream: true
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 11.5, y: -5.0, theta: 2.604}
goal: {x: -25.00, y: 3.00, theta: -1.999}
mod_type: 1
upstream: true
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 11.5, y: -5.0, theta: 2.604}
goal: {x: -25.00, y: 3.00, theta: -1.999}
mod_type: 2
upstream: true
weight_c: 0.00"