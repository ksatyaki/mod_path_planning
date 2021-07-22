#!/bin/zsh

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
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352883600, nsecs: 0}
  frame_id: 'map'
times: 30
planning_time_limit: $1
start: {x: 47.690, y: -18.848, theta: -2.356}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 1
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352890000, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 47.690, y: -18.848, theta: -2.356}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 1
upstream: true
weight_c: 0.1"

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
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352883600, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
goal: {x: -3.85, y: -6.82, theta: -0.8203}
start: {x: 21.471, y: -17.647, theta: 0.733}
mod_type: 1
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352890000, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
goal: {x: -3.85, y: -6.82, theta: -0.8203}
start: {x: 21.471, y: -17.647, theta: 0.733}
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
mod_type: 1
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352883600, nsecs: 0}
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
  stamp: {secs: 1352890000, nsecs: 0}
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
start: {x: 11.5, y: -5.0, theta: 2.604}
goal: {x: -25.00, y: 3.00, theta: -1.999}
mod_type: 1
upstream: true
weight_c: 0.1"

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

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352890000, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 11.5, y: -5.0, theta: 2.604}
goal: {x: -25.00, y: 3.00, theta: -1.999}
mod_type: 1
upstream: true
weight_c: 0.1"
