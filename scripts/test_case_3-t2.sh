#!/bin/zsh

#Setting goal: Frame:map, Position(47.690, -18.848, 0.000), Orientation(0.000, 0.000, 0.924, -0.383) = Angle: -2.356
#Setting goal: Frame:map, Position(-19,575, 12,390, 0,000), Orientation(0,000, 0,000, 0,915, 0,403) = Angle: 2,313

rostopic pub -1 /mod_planning_goal mod_path_planning/MoDPlanningGoal "header:                                                                                              
  seq: 0
  stamp: {secs: 1352883600, nsecs: 0}
  frame_id: 'map'
times: 30
planning_time_limit: $1
start: {x: 47.690, y: -18.848, theta: -2.356}
goal: {x: -19.575, y: 12.390, theta: 2.313}
mod_type: 2
upstream: true
weight_c: 0.1"
