#!/bin/zsh

#Setting goal: Frame:map, Position(46,287, -22,824, 0,000), Orientation(0,000, 0,000, 0,991, 0,133) = Angle: 2,875

#Setting goal: Frame:map, Position(-19,575, 12,390, 0,000), Orientation(0,000, 0,000, 0,915, 0,403) = Angle: 2,313


rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352894400, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 46.287, y: -22.824, theta: 2.875}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 0
upstream: false
weight_c: 0.00"

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:
  seq: 0
  stamp: {secs: 1352894400, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 46.287, y: -22.824, theta: 2.875}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 0
upstream: false
weight_c: 0.02"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352894400, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 46.287, y: -22.824, theta: 2.875}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 3
upstream: false
weight_c: 0.1"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352894400, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 46.287, y: -22.824, theta: 2.875}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 0
upstream: true
weight_c: 0.1"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352894400, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 46.287, y: -22.824, theta: 2.875}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 1
upstream: true
weight_c: 0.1"

sleep 10

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352894400, nsecs: 0}
  frame_id: 'map'
times: $2
planning_time_limit: $1
start: {x: 46.287, y: -22.824, theta: 2.875}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 2
upstream: true
weight_c: 0.1"
