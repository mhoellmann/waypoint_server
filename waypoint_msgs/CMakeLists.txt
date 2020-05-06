cmake_minimum_required(VERSION 2.8.3)
project(waypoint_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  nav_msgs
  geometry_msgs
  genmsg
  uuid_msgs
)

add_message_files(FILES
  WaypointNode.msg
  WaypointEdge.msg
  WaypointGraph.msg
)

add_service_files(FILES
  RemoveEdge.srv
  SaveWaypoints.srv
  LoadWaypoints.srv
  LoadZones.srv
  LoadWaypointsShapefiles.srv
  LoadZonesShapefiles.srv
  GetWaypointGraph.srv
  GetShortestPath.srv
  SetFloorLevel.srv
  LoadDoorData.srv
  GetZone.srv
)

generate_messages(DEPENDENCIES
  nav_msgs
  geometry_msgs
  std_msgs
  uuid_msgs
)

catkin_package(CATKIN_DEPENDS
  message_generation
  message_runtime
  nav_msgs
  geometry_msgs
  std_msgs
  genmsg
  uuid_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
