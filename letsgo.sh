#!/bin/zsh

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1554117961, nsecs: 0}
  frame_id: 'map'
times: 5
planning_time_limit: 600.0
start: {x: 2.044, y: 1.118, theta: 3.122}
goal: {x: -8.633, y: 11.713, theta: 2.403}
mod_type: 0
upstream: false
weight_c: 0.001"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1554117961, nsecs: 0}
  frame_id: 'map'
times: 5
planning_time_limit: 600.0
start: {x: 2.044, y: 1.118, theta: 3.122}
goal: {x: -8.633, y: 11.713, theta: 2.403}
mod_type: 3
upstream: false
weight_c: 0.001"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1554117961, nsecs: 0}
  frame_id: 'map'
times: 5
planning_time_limit: 600.0
start: {x: 2.044, y: 1.118, theta: 3.122}
goal: {x: -8.633, y: 11.713, theta: 2.403}
mod_type: 0
upstream: true
weight_c: 0.01"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1554117961, nsecs: 0}
  frame_id: 'map'
times: 5
planning_time_limit: 600.0
start: {x: 2.044, y: 1.118, theta: 3.122}
goal: {x: -8.633, y: 11.713, theta: 2.403}
mod_type: 1
upstream: true
weight_c: 0.001"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1554117961, nsecs: 0}
  frame_id: 'map'
times: 5
planning_time_limit: 600.0
start: {x: 2.044, y: 1.118, theta: 3.122}
goal: {x: -8.633, y: 11.713, theta: 2.403}
mod_type: 2
upstream: true
weight_c: 0.1"
