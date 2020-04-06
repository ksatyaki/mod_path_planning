#!/bin/zsh

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352597736, nsecs: 0}
  frame_id: 'map'
times: 5
planning_time_limit: $1
start: {x: -15.97, y: -11.0, theta: 1.603}
goal: {x: -15.343, y: 11.00, theta: 1.586}
mod_type: 0
upstream: false
weight_c: 0.01"

sleep 1

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352597736, nsecs: 0}
  frame_id: 'map'
times: 5
planning_time_limit: $1
start: {x: -15.97, y: -11.0, theta: 1.603}
goal: {x: -15.343, y: 11.00, theta: 1.586}
mod_type: 0
upstream: false
weight_c: 0.01"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352597736, nsecs: 0}
  frame_id: 'map'
times: 5
planning_time_limit: $1
start: {x: -15.97, y: -11.0, theta: 1.603}
goal: {x: -15.343, y: 11.00, theta: 1.586}
mod_type: 3
upstream: false
weight_c: 0.01"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352597736, nsecs: 0}
  frame_id: 'map'
times: 5
planning_time_limit: $1
start: {x: -15.97, y: -11.0, theta: 1.603}
goal: {x: -15.343, y: 11.00, theta: 1.586}
mod_type: 0
upstream: true
weight_c: 0.01"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352597736, nsecs: 0}
  frame_id: 'map'
times: 5
planning_time_limit: $1
start: {x: -15.97, y: -11.0, theta: 1.603}
goal: {x: -15.343, y: 11.00, theta: 1.586}
mod_type: 1
upstream: true
weight_c: 0.01"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352597736, nsecs: 0}
  frame_id: 'map'
times: 5
planning_time_limit: $1
start: {x: -15.97, y: -11.0, theta: 1.603}
goal: {x: -15.343, y: 11.00, theta: 1.586}
mod_type: 2
upstream: true
weight_c: 0.01"
