#!/bin/zsh

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352890000, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: -17.00, y: -8.656, theta: 1.194}
goal: {x: -3.493, y: 7.846, theta: 0.809}
mod_type: 2
upstream: true
weight_c: 0.1"
