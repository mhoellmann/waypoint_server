#!/bin/bash

# rosservice call /waypoint_server/get_shortest_path "start: {x: 10.8, y: 6.9, z: 0.0}
# target: {x: 13.7, y: 15.8, z: 0.0}
# startzone: ''
# targetzone: ''
# floor_number: 0"

rosservice call /waypoint_server/get_shortest_path "start:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 10.8, y: 6.9, z: 0.0}
    orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
target:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 13.7, y: 15.8, z: 0.0}
    orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
startzone: ''
targetzone: ''
floor_number: 0"