#!/bin/zsh
# x: 30.243206024169922
# y: -0.4475841522216797
#    z: 0.7291656139070131
#    w: 0.6843372761260406

# x: 2.2425737380981445
# y: 27.614727020263672
#    z: 0.9993821931641956
#    w: 0.03514586727230288

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
goal: {x: 10.0, y: 27.0, theta: 0.0}
start: {x: 22.0, y: 2.0, theta: -3.14}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
goal: {x: 10.0, y: 27.0, theta: 0.0}
start: {x: 22.0, y: 2.0, theta: -3.14}
mod_type: 0
upstream: false
weight_c: 0.02"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
goal: {x: 10.0, y: 27.0, theta: 0.0}
start: {x: 22.0, y: 2.0, theta: -3.14}
mod_type: 0
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
goal: {x: 10.0, y: 27.0, theta: 0.0}
start: {x: 22.0, y: 2.0, theta: -3.14}
mod_type: 2
upstream: true
weight_c: 0.1"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352866100, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
goal: {x: 10.0, y: 27.0, theta: 0.0}
start: {x: 22.0, y: 2.0, theta: -3.14}
mod_type: 4
upstream: true
weight_c: 0.2"
