#!/bin/zsh

# x: 0.08393287658691406
# y: 27.059646606445312
#    z: 0.028810509310999877
#    w: 0.9995848911189288

# x: 30.613258361816406
# y: 0.7859249114990234
#     z: -0.7123635930770941
#     w: 0.7018105950028771

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 0.0, y: 27.0, theta: 0.0}
goal: {x: 30.0, y: 0.0, theta: -1.57079632679}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 0.0, y: 27.0, theta: 0.0}
goal: {x: 30.0, y: 0.0, theta: -1.57079632679}
mod_type: 0
upstream: false
weight_c: 0.02"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 0.0, y: 27.0, theta: 0.0}
goal: {x: 30.0, y: 0.0, theta: -1.57079632679}
mod_type: 0
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 0.0, y: 27.0, theta: 0.0}
goal: {x: 30.0, y: 0.0, theta: -1.57079632679}
mod_type: 2
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 0.0, y: 27.0, theta: 0.0}
goal: {x: 30.0, y: 0.0, theta: -1.57079632679}
mod_type: 4
upstream: true
weight_c: 0.2"
