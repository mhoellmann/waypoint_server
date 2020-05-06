#!/usr/bin/env python2

SCALE_EDGE = 5 #0.45
SCALE_NODE = 10 #1

MAP_FRAME = "utm"

import rospy
import tf

# interactice markers
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

# gui / rviz
from text_box_input import InputApp
from waypoint_msgs.msg import WaypointEdge, WaypointNode, WaypointGraph

#import services
from waypoint_msgs.srv import RemoveEdge, RemoveEdgeResponse
from waypoint_msgs.srv import SaveWaypoints, SaveWaypointsResponse
from waypoint_msgs.srv import LoadWaypoints, LoadWaypointsResponse
from waypoint_msgs.srv import LoadZones, LoadZonesResponse
from waypoint_msgs.srv import LoadWaypointsShapefiles, LoadWaypointsShapefilesResponse
from waypoint_msgs.srv import LoadZonesShapefiles, LoadZonesShapefilesResponse
from waypoint_msgs.srv import GetWaypointGraph, GetWaypointGraphResponse
from waypoint_msgs.srv import GetShortestPath, GetShortestPathResponse
from waypoint_msgs.srv import SetFloorLevel, SetFloorLevelResponse
from waypoint_msgs.srv import LoadDoorData, LoadDoorDataResponse
from waypoint_msgs.srv import GetZone, GetZoneResponse


# geometry and visualization
from visualization_msgs.msg import *
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Point32, Quaternion, Pose, PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
import shapely.geometry
import networkx as nx


#general stuff
import unique_id
import math
import yaml
from random import randint
import os

#shapefile operations
import shapefile

#mbf imports
import actionlib
from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import GetPathGoal
from mbf_msgs.msg import GetPathResult

#from shortest_path import ShortestPath

# constants
from waypoint_constants import *





def euclidean_distance(point1, point2):
    """Return the euclidean distance between two points."""
    return math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2 + (point2.z - point1.z)**2)

class WaypointServer:
    def __init__(self):
        #init member variables
        self.server = InteractiveMarkerServer("waypoint_server")
        self.waypoint_graph = nx.Graph()
        self.next_waypoint_id = 0
        self.next_edge_id = 0
        self.floor_level = rospy.get_param("~floor_level", 0)
        self.state = STATE_REGULAR
        self.active_waypoints = None
        self.connect_from_marker = ""
        self.active_marker = None
        self.edge_type = EDGE_REGULAR
        self.waypoint_type = WAYPOINT_REGULAR
        self.edge_line_publisher = rospy.Publisher("~edges", MarkerArray, queue_size=10, latch=True)
        self.marker_frame = rospy.get_param("~marker_frame", MAP_FRAME)
        self.uuid_name_map = {}
        self.door_data = {}
        self.waypoint_file = ""
        self.zones_file= ""
        self.zones = {}
        self.zones_controller = {}

        self.tf_listener = tf.TransformListener()

        #self.path_manager = ShortestPath()  # for checking closest waypoints by path

        #services, subscriber, publisher
        rospy.Service('~remove_edge', RemoveEdge, self.remove_edge_service_call)
        rospy.Service('~load_waypoints', LoadWaypoints, self.load_waypoints_service)
        rospy.Service('~load_waypoints_shapefiles', LoadWaypointsShapefiles, self.load_waypoints_shapefiles_service)
        rospy.Service('~load_zones', LoadZones, self.load_zones_service)
        rospy.Service('~load_zones_shapefiles', LoadZonesShapefiles, self.load_zones_shapefiles_service)
        rospy.Service('~save_waypoints', SaveWaypoints, self.save_waypoints_service)
        rospy.Service('~get_shortest_path', GetShortestPath, self.get_shortest_path_service)
        rospy.Service('~get_waypoint_graph', GetWaypointGraph, self.get_waypoint_graph_service_call)
        rospy.Service('~set_floor_level', SetFloorLevel, self.set_floor_level)
        rospy.Service('~get_zone', GetZone, self.get_zone_service)
        rospy.Subscriber("/clicked_point", PointStamped, self.insert_marker_callback)
        rospy.Subscriber("/clicked_pose", PoseStamped, self.insert_terminal_marker_callback)
        self.zones_pub = rospy.Publisher("~zones", PolygonArray, queue_size=10, latch=True)
        rospy.on_shutdown(self.clear_all_markers)
        # rospy.logwarn("The waypoint server is waiting for RViz to run and to be subscribed to {0}.".format(rospy.resolve_name("~edges")))
        # while self.edge_line_publisher.get_num_connections() == 0:
        #     rospy.sleep(1.0)


        #reset marker
        self.clear_all_markers()

        #load zones directory path from param server
        self.zones_shp_dir = rospy.get_param("~zones_shp_dir", "") # directory with shapefiles of all zones

        if len(self.zones_shp_dir) != 0:
            rospy.loginfo("Waypoint_Server is loading initial zone shapefiles from directory {0}.".format(self.zones_shp_dir))
            error_message = self.load_zones_from_shp(self.zones_shp_dir)
            if error_message:
                rospy.logerr(error_message)

        #load waypoint directory path from param server
        self.waypoint_shp_dir = rospy.get_param("~waypoint_shp_dir", "") # directory with shapefiles of all zones

        if len(self.waypoint_shp_dir) != 0:
            rospy.loginfo("Waypoint_Server is loading initial waypoint shapefiles from directory {0}.".format(self.waypoint_shp_dir))
            error_message = self.load_waypoints_from_shp(self.waypoint_shp_dir)
            if error_message:
                rospy.logerr(error_message)

        


        #load waypoint file path from param server
        self.waypoint_file = rospy.get_param("~waypoint_file", "") # yaml file with waypoints to load

        #load waypoints
        if len(self.waypoint_file) != 0:
            if len(self.zones_shp_dir) != 0:
                rospy.logwarn("Waypoints already loaded from shapefiles! Discarding waypoint file!")
            else:
                rospy.loginfo("Waypoint_Server is loading initial waypoint file {0}.".format(self.waypoint_file))
                error_message = self.load_waypoints_from_file(self.waypoint_file)
                if error_message:
                    rospy.logerr(error_message)

        #load zones file path from param server
        self.zones_file = rospy.get_param("~zones_file", "") # yaml file with zones to load

        # load zones
        if len(self.zones_file) != 0:
            if len(self.zones_shp_dir) != 0:
                rospy.logwarn("Zones already loaded from shapefiles! Discarding zones file!")
            else:
                rospy.loginfo("Waypoint_Server is loading initial zones file {0}.".format(self.zones_file))
                error_message = self.load_zones_from_file(self.zones_file)
                if error_message:
                    rospy.logerr(error_message)

    #callback insert normale marker
    def insert_marker_callback(self, pos):
        rospy.logdebug("Inserting new waypoint at position ({0},{1},{2}).".format(pos.point.x, pos.point.y, pos.point.z))
        self.waypoint_type = WAYPOINT_REGULAR
        self.insert_marker(pos.point, floor_level=self.floor_level)

    #callback insert terminal marker
    def insert_terminal_marker_callback(self, pos):
        rospy.logdebug("Inserting new terminal waypoint at position ({0},{1},{2}).".format(pos.pose.position.x, pos.pose.position.y, pos.pose.position.z))
        self.waypoint_type = WAYPOINT_TERMINATING
        self.insert_marker(pos.pose, waypoint_type=WAYPOINT_TERMINATING, floor_level=self.floor_level)

    # clear all markers from marker server
    def clear_all_markers(self, clear_graph=True):
        edges = MarkerArray()
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.marker_frame
        marker.ns = "waypoint_edges"
        marker.id = 0
        marker.action = Marker.DELETEALL  # NOTE: DELETALL only supported from kinetic onwards
        edges.markers.append(marker)
        self.edge_line_publisher.publish(edges)
        #also clear networkx graph
        if clear_graph:
            self.waypoint_graph.clear()

    #returns a marker
    def _make_marker(self, int_marker):
        marker = Marker()
        if self.waypoint_type == WAYPOINT_REGULAR:
            marker.type = Marker.SPHERE
            marker.scale.x = int_marker.scale * 0.45
            marker.scale.y = int_marker.scale * 0.45
            marker.scale.z = int_marker.scale * 0.45
            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0
            marker.color.a = 1.0
        elif self.waypoint_type == WAYPOINT_TERMINATING:
            marker.type = Marker.ARROW
            marker.pose.orientation = Quaternion(1, 0, 0, 0)
            marker.scale.x = int_marker.scale * 0.5
            marker.scale.y = int_marker.scale * 0.25
            marker.scale.z = int_marker.scale * 0.45
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 1.0

        return marker

    # returns an edge
    def _make_edge(self, scale, begin, end, edge_type=EDGE_REGULAR):
        edge = Marker()
        edge.header.frame_id = self.marker_frame
        edge.header.stamp = rospy.Time.now()
        edge.ns = "waypoint_edges"
        edge.id = self.next_edge_id
        self.next_edge_id += 1 # increase global id counter
        edge.type = Marker.LINE_LIST
        edge.action = Marker.ADD
        edge.scale.x = scale * SCALE_EDGE
        edge.scale.y = scale * SCALE_EDGE
        edge.scale.z = scale * SCALE_EDGE
        self._set_edge_color(edge, edge_type)
        edge.points.append(begin)
        edge.points.append(end)
        return edge

    # returns the edge color corresponding to edge type
    def _set_edge_color(self, edge, edge_type):
        if edge_type == EDGE_REGULAR:
            edge.color = EDGE_REGULAR_COLOR
        elif edge_type == EDGE_DOOR:
            edge.color = EDGE_DOOR_COLOR
        elif edge_type == EDGE_ELEVATOR:
            edge.color = EDGE_ELEVATOR_COLOR
        edge.color.a = 1

    #renames a marker (gui callback)
    def _rename_marker(self, name):
        old_name = self.uuid_name_map[name]
        new_name = InputApp(old_name, "Rename Marker").create_app()
        if new_name == old_name or new_name == None:
            return

        interactiveMarker = self.server.get(name)
        interactiveMarker.description = new_name
        self.uuid_name_map.update({name: new_name})
        self.server.insert(interactiveMarker)
        self.server.applyChanges()
        rospy.loginfo("changed waypoint name from '{0}' to '{1}'".format(old_name, new_name))
    
    #changes the zone of a marker (gui callback)
    def _change_marker_zone(self, name, handle):
        if handle == MENU_CHANGE_ZONE1:
            headline = "Change Zone 1"
            zoneAttr='zone1'
        elif handle == MENU_CHANGE_ZONE2:
            headline = "Change Zone 2"
            zoneAttr='zone2'

        zone_old = self.waypoint_graph.nodes[name][zoneAttr]

        zone_new = InputApp(zone_old, headline).create_app()
        if zone_new == zone_old or zone_new == None:
            return

        self.waypoint_graph.nodes[name][zoneAttr] = zone_new
        rospy.loginfo("changed waypoint {0} from '{1}' to '{2}'".format(zoneAttr, zone_old, zone_new))
        self.update_edges_zone()

    #remove a marker
    def _remove_marker(self, name):
        self._clear_marker_edges(name)
        self.waypoint_graph.remove_node(name)
        del self.uuid_name_map[name]
        self.server.erase(name)
        self.server.applyChanges()


    def _clear_marker_edges(self, name):
        # remove all edges to a waypoint
        edges = MarkerArray()
        to_remove = []
        for u, v, marker in self.waypoint_graph.edges(name, data='marker'):
            marker.action = Marker.DELETE
            to_remove.append((u, v, marker))

        # update knowledge database to remove all edges
        for u, v, marker in to_remove:
            edges.markers.append(marker)
            self.waypoint_graph.remove_edge(u, v)
        self.edge_line_publisher.publish(edges)    # publish deletion

    def _disconnect_markers(self, u, v):
        if self.waypoint_graph.has_edge(u, v):
            # remove edge
            edges = MarkerArray()
            marker = self.waypoint_graph.get_edge_data(u, v)["marker"]
            marker.action = Marker.DELETE
            edges.markers.append(marker)
            self.waypoint_graph.remove_edge(u, v)
            self.edge_line_publisher.publish(edges)  # publish deletion

    def _connect_markers(self, u, v, edge_type=EDGE_REGULAR, floor_level=0):
        if u == v:
            rospy.logwarn("Cannot connect a marker to itself")
            return
        
        u_pose = self.server.get(u).pose
        v_pose = self.server.get(v).pose
        u_pos = u_pose.position
        v_pos = v_pose.position

        #get the path between the nodes
        #the costs of the path are used for the edge costs
        path = self.get_path(u_pose, v_pose)
        
        # insert edge
        edge = self._make_edge(0.2, u_pos, v_pos, edge_type)
        edge.text = str(path.cost)

        # insert edge into graph
        self.waypoint_graph.add_edge(u, v, u=u, v=v, cost=path.cost, path=path.path, edge_type=edge_type, marker=edge, floor_level=floor_level)
        self.update_edges_zone()
        self.update_edges()
        rospy.logwarn("added edge")

    def set_marker_highlight(self, highlight=True, marker=None):
        if marker == None:
            marker = self.active_marker
        m = self.server.get(self.active_marker)
        m.controls[0].markers[0].color.b = 1 if highlight == True else 0
        self.server.insert(m)
        self.server.applyChanges()
        pass

    def update_edges(self):
        edges = MarkerArray()
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.marker_frame
        marker.ns = "waypoint_edges"
        marker.id = 0
        marker.action = Marker.DELETEALL
        edges.markers.append(marker)
        for _, _, marker in self.waypoint_graph.edges(data=True):
            if marker["floor_level"] == self.floor_level:
                edges.markers.append(marker["marker"])
        self.edge_line_publisher.publish(edges)

    def update_edges_zone(self):
        graph = self.waypoint_graph

        for u, v, data in self.waypoint_graph.edges(data=True):
            #if the zones of the incident nodes are the same add the controller of the zone to the edge
            if graph.nodes[u]["zone1"] == graph.nodes[v]["zone1"] or graph.nodes[u]["zone1"] == graph.nodes[v]["zone2"]:
                data["controller"] = self.zones_controller.get(graph.nodes[u]["zone1"], None)
            elif graph.nodes[u]["zone2"] == graph.nodes[v]["zone1"] or graph.nodes[u]["zone2"] == graph.nodes[v]["zone2"]:
                data["controller"] = self.zones_controller.get(graph.nodes[u]["zone2"], None)
            
            

    def process_feedback(self, feedback):
        #right-click menu of the interactive markers
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            handle = feedback.menu_entry_id
            #start the creation of a normal edge
            if handle == MENU_CONNECT_EDGE:
                self.state = STATE_CONNECT
                self.edge_type = EDGE_REGULAR
                self.active_marker = feedback.marker_name
                self.connect_from_marker = feedback.marker_name
                self.set_marker_highlight(marker=feedback.marker_name)
            #start the creation of a door edge
            elif handle == MENU_CONNECT_EDGE_DOOR:
                self.state = STATE_CONNECT
                self.edge_type = EDGE_DOOR
                self.active_marker = feedback.marker_name
                self.connect_from_marker = feedback.marker_name
                self.set_marker_highlight(marker=feedback.marker_name)
            #start the creation of a elevator edge
            elif handle == MENU_CONNECT_EDGE_ELEVATOR:
                self.state = STATE_CONNECT
                self.edge_type = EDGE_ELEVATOR
                self.active_marker = feedback.marker_name
                self.connect_from_marker = feedback.marker_name
                self.set_marker_highlight(marker=feedback.marker_name)
            #start the deletion of an edge
            elif handle == MENU_DISCONNECT_EDGE:
                self.state = STATE_DISCONNECT
                self.active_marker = feedback.marker_name
                self.connect_from_marker = feedback.marker_name
                self.set_marker_highlight(marker=feedback.marker_name)
            #clear all edges of the marker
            elif handle == MENU_CLEAR:
                self._clear_marker_edges(feedback.marker_name)
            #rename the marker via pop-up text input box
            elif handle == MENU_RENAME:
                self._rename_marker(feedback.marker_name)
            #remove the marker
            elif handle == MENU_REMOVE:
                self._remove_marker(feedback.marker_name)
            #change a zone of the marker via pop-up text input box
            elif handle == MENU_CHANGE_ZONE1 or handle == MENU_CHANGE_ZONE2:
                self._change_marker_zone(feedback.marker_name,handle)

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            #finish connection of two nodes
            if self.state == STATE_CONNECT:
                self.state = STATE_NONE
                self._connect_markers(self.connect_from_marker, feedback.marker_name, edge_type=self.edge_type, floor_level=self.floor_level)
                self.set_marker_highlight(highlight=False)
            #finish deletion of an edge
            elif self.state == STATE_DISCONNECT:
                self.state = STATE_NONE
                self._disconnect_markers(self.connect_from_marker, feedback.marker_name)
                self.set_marker_highlight(highlight=False)
            elif self.state == STATE_NONE:
                pass    # ignore
            else:
                #updating the pose of an interactive marker
                pos = feedback.pose.position
                rospy.logdebug("Updating pose of marker {3} to ({0},{1},{2})".format(pos.x, pos.y, pos.z, feedback.marker_name))
                # update database
                # push to scene database
                pstamped = PoseStamped()
                pstamped.header.frame_id = self.marker_frame
                pstamped.pose = feedback.pose

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            if self.state == STATE_NONE:
                self.state = STATE_REGULAR

        self.server.applyChanges()

    #update marker and all edges (path, costs)
    def move_feedback(self, feedback):
        pose = feedback.pose

        self.server.setPose(feedback.marker_name, pose)

        # correct all edges
        for u, v, data in self.waypoint_graph.edges([feedback.marker_name], data=True):
            if feedback.marker_name == data["u"]:
                data["marker"].points[0] = pose.position
            else:
                data["marker"].points[1] = pose.position
            u_pose = self.server.get(u).pose
            v_pose = self.server.get(v).pose
            if self.waypoint_graph.node[u]["waypoint_type"] == WAYPOINT_REGULAR:
                self.waypoint_graph.node[u]["position"] = u_pose.position
            elif self.waypoint_graph.node[u]["waypoint_type"] == WAYPOINT_TERMINATING:
                self.waypoint_graph.node[u]["position"] = u_pose
            if self.waypoint_graph.node[v]["waypoint_type"] == WAYPOINT_REGULAR:
                self.waypoint_graph.node[v]["position"] = v_pose.position
            elif self.waypoint_graph.node[v]["waypoint_type"] == WAYPOINT_TERMINATING:
                self.waypoint_graph.node[v]["position"] = v_pose
            
            path = self.get_path(u_pose,v_pose)
            data["cost"] = path.cost
            data["path"] = path.path
        self.update_edges()
        self.server.applyChanges()

    #insert a marker
    def insert_marker(self, position, waypoint_type=WAYPOINT_REGULAR, name=None, uuid=None, frame_id=None, floor_level=0, zone1="", zone2=""):
        #standard frame id
        if frame_id is None:
            frame_id = self.marker_frame

        #new uuid if not exists
        if uuid is None:
            uuid = unique_id.fromRandom()

        #create new standard name if not exists
        if name is None:
            name = "wp{:03}".format(self.next_waypoint_id)
            self.next_waypoint_id += 1

        self.uuid_name_map[str(uuid)] = name
        self.waypoint_type = waypoint_type
        # insert waypoint
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        if waypoint_type == WAYPOINT_REGULAR:
            int_marker.pose.position = position
            int_marker.pose.orientation.x = 1
        elif waypoint_type == WAYPOINT_TERMINATING:
            int_marker.pose.position = position.position
            int_marker.pose.orientation = position.orientation
        int_marker.scale = SCALE_NODE
        int_marker.name = str(uuid)
        int_marker.description = name

        # make markers moveable in the plane
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        if waypoint_type == WAYPOINT_REGULAR:
            control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        elif waypoint_type == WAYPOINT_TERMINATING:
            control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE

        # interactive menu for each marker
        menu_handler = MenuHandler()
        menu_handler.insert("Connect normal edge...", callback=self.process_feedback)
        menu_handler.insert("Connect door edge...", callback=self.process_feedback)
        menu_handler.insert("Connect elevator edge...", callback=self.process_feedback)
        menu_handler.insert("Disconnect edge...", callback=self.process_feedback)
        menu_handler.insert("Clear connected edges", callback=self.process_feedback)
        menu_handler.insert("Rename marker...", callback=self.process_feedback)
        menu_handler.insert("Remove marker", callback=self.process_feedback)
        menu_handler.insert("Change zone 1...", callback=self.process_feedback)
        menu_handler.insert("Change zone 2...", callback=self.process_feedback)

        # make a box which also moves in the plane
        control.markers.append(self._make_marker(int_marker))
        control.always_visible = True
        int_marker.controls.append(control)

        # we want to use our special callback function
        self.server.insert(int_marker, self.process_feedback)
        menu_handler.apply(self.server, int_marker.name)

        # set different callback for POSE_UPDATE feedback
        pose_update = InteractiveMarkerFeedback.POSE_UPDATE
        self.server.setCallback(int_marker.name, self.move_feedback, pose_update)
        self.server.applyChanges()

        # insert into graph
        self.waypoint_graph.add_node(str(uuid), waypoint_type=waypoint_type, floor_level=floor_level, position=position, zone1=zone1, zone2=zone2)

    #save the waypoints to yaml file
    def save_waypoints_service(self, request):
        #use filename of request
        if request.file_name:
            filename = request.file_name
        #use filename from loading waypoint file if no request filename is given
        else:
            filename = self.waypoint_file

        response = SaveWaypointsResponse()
        error_message = self.save_waypoints_to_file(filename)

        if error_message == None:
            response.success = True
            response.message = "Saved waypoints to " + filename
            rospy.loginfo("Saved waypoints to {0}".format(filename))
        else:
            response.success = False
            response.message = str(error_message)
            rospy.loginfo("Failed to save waypoints: {0}".format(error_message))
        return response

    #service callback load waypoints from file
    def load_waypoints_service(self, request):
        self.waypoint_file = request.file_name
        response = LoadWaypointsResponse()
        error_message = self.load_waypoints_from_file(self.waypoint_file)

        if error_message == None:
            response.success = True
            response.message = "Loaded waypoints from " + str(self.waypoint_file)
            response.graph = self.get_waypoint_graph_msg(self.waypoint_graph)
            rospy.loginfo("Loaded waypoints from {0}".format(self.waypoint_file))
        else:
            response.success = False
            response.message = str(error_message)
            rospy.loginfo("Failed to load waypoints: {0}".format(error_message))
        return response

    #service callback load waypoints from shapefiles
    def load_waypoints_shapefiles_service(self, request):
        self.waypoint_shp_dir = request.file_folder
        response = LoadWaypointsShapefilesResponse()
        error_message = self.load_waypoints_from_shp(self.waypoint_file)

        if error_message == None:
            response.success = True
            response.message = "Loaded waypoints from " + str(self.waypoint_shp_dir)
            response.graph = self.get_waypoint_graph_msg(self.waypoint_graph)
            rospy.loginfo("Loaded waypoints from {0}".format(self.waypoint_shp_dir))
        else:
            response.success = False
            response.message = str(error_message)
            rospy.loginfo("Failed to load waypoints: {0}".format(error_message))
        return response

    #service callback load waypoints from file
    def load_zones_service(self, request):
        self.zones_file = request.file_name
        response = LoadZonesResponse()
        error_message = self.load_zones_from_file(self.zones_file)

        if error_message == None:
            response.success = True
            response.message = "Loaded zones from " + str(self.zones_file)
            # response.polygons = 
            rospy.loginfo("Loaded zones from {0}".format(self.zones_file))
        else:
            response.success = False
            response.message = str(error_message)
            rospy.loginfo("Failed to load zones: {0}".format(error_message))
        return response
    
    #service callback load waypoints from shapefiles
    def load_zones_shapefiles_service(self, request):
        self.zones_shp_dir = request.folder
        response = LoadZonesShapefilesResponse()
        error_message = self.load_zones_from_shp(self.zones_shp_dir)

        if error_message == None:
            response.success = True
            response.message = "Loaded zones from " + str(self.zones_shp_dir)
            # response.polygons = 
            rospy.loginfo("Loaded zones from {0}".format(self.zones_shp_dir))
        else:
            response.success = False
            response.message = str(error_message)
            rospy.loginfo("Failed to load zones: {0}".format(error_message))
        return response

    #save the waypoints to file
    def save_waypoints_to_file(self, filename):
        error_message = None  # assume file location and contents are correct
        try:
            #data struct which will be dumped to file
            data = {"waypoints": {}, "edges": []}

            if self.waypoint_file and not filename:
                filename = self.waypoint_file

            #write all waypoints to data
            for uuid, waypoint_data in self.waypoint_graph.nodes(data=True):
                name = self.uuid_name_map[uuid]
                waypoint_type = waypoint_data['waypoint_type']
                floor_level = waypoint_data['floor_level']
                zone1 = waypoint_data['zone1']
                zone2 = waypoint_data['zone2']
                if waypoint_type == WAYPOINT_REGULAR:
                    pos = waypoint_data["position"]
                    data["waypoints"].update({uuid: {"name": name, "x": pos.x, "y": pos.y, "z": pos.z, "floor_level":floor_level,
                    "waypoint_type":waypoint_type, "zone1":zone1, "zone2":zone2}})
                elif waypoint_type == WAYPOINT_TERMINATING:
                    pos = waypoint_data["position"].position
                    orientation = waypoint_data["position"].orientation
                    data["waypoints"].update({uuid: {"name": name, "x": pos.x, "y": pos.y, "z": pos.z, "floor_level":floor_level,
                    "waypoint_type":waypoint_type, "zone1":zone1, "zone2":zone2, "orientation":{"x":orientation.x, "y":orientation.y, "w":orientation.w, "z":orientation.z}}})
            
            #write all edges to data
            for u, v, edge_data in self.waypoint_graph.edges(data=True):
                cost = edge_data['cost']
                edge_type = edge_data['edge_type']
                floor_level = edge_data['floor_level']
                data["edges"].append({'u': u, 'v': v, 'cost': cost, 'edge_type': edge_type, "floor_level":floor_level})

            #write data to file
            with open(filename, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
        except Exception as e:
            error_message = e
        return error_message

    #save the zones to file
    def save_zones_to_file(self, filename):
        #NOT IMPLEMENTED
        #zones cannot be edited --> writing not needed
       return False

    #load waypoints from file
    def load_waypoints_from_file(self, filename):
        error_message = None  # assume file location and contents are correct
        try:
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)

            #delete existing data (marker, networkx)
            self.clear_all_markers()
            self.server.clear()
            self.server.applyChanges()

            #from loaded data to needed data struct
            #waypoints
            for uuid, wp in data["waypoints"].items():
                wp_type = wp["waypoint_type"]
                if wp_type == WAYPOINT_REGULAR:
                    point = Point(wp["x"], wp["y"], wp["z"])
                elif wp_type == WAYPOINT_TERMINATING:
                    point = Pose()
                    point.position = Point(wp["x"], wp["y"], wp["z"])
                    point.orientation = Quaternion(wp["orientation"]["x"], wp["orientation"]["y"],
                        wp["orientation"]["z"], wp["orientation"]["w"])
                self.insert_marker(position=point, uuid=uuid, name=wp['name'], waypoint_type=wp_type, floor_level=wp["floor_level"], zone1=wp["zone1"], zone2=wp["zone2"])
            
            #from loaded data to needed data struct
            #edges
            for edge in data["edges"]:
                self._connect_markers(edge['u'], edge['v'], edge['edge_type'], floor_level=edge["floor_level"])

            self.display_waypoints_from_graph()

        except Exception as e:
            error_message = e
        return error_message


    def load_waypoints_from_shp(self, dir):
        error_message = None  # assume file location and contents are correct
        try:

            #delete existing data (marker, networkx)
            self.clear_all_markers()
            self.server.clear()
            self.server.applyChanges()

            #check all files in directory
            for file in os.listdir(dir):
                #load if it is a shapefile
                if file.endswith(".shp"):
                    print(os.path.join(dir, file))
                    with shapefile.Reader(os.path.join(dir, file)) as shp:
                        #iterate over all shapes
                        for shapeRecord in shp.iterShapeRecords():
                            for coords in shapeRecord.shape.points:
                                wp_type = WAYPOINT_REGULAR
                                point = Point(coords[0], coords[1], 5)
                                
                                self.insert_marker(position=point, uuid=unique_id.fromRandom(), name=shapeRecord.record.name, waypoint_type=wp_type, floor_level=0, zone1=shapeRecord.record.zone1, zone2=shapeRecord.record.zone2)

            #connect all nodes of a zone with each other
            #edges
            for i, (uuid_a, waypoint_data_a) in enumerate(self.waypoint_graph.nodes.items()[:-1]):
                for uuid_b, waypoint_data_b in self.waypoint_graph.nodes.items()[i+1:]:
                    if waypoint_data_a["zone1"] == waypoint_data_b["zone1"] or \
                       waypoint_data_a["zone1"] == waypoint_data_b["zone2"] or \
                       waypoint_data_a["zone2"] == waypoint_data_b["zone1"] or \
                       waypoint_data_a["zone2"] == waypoint_data_b["zone2"]:

                        self._connect_markers(uuid_a, uuid_b, EDGE_REGULAR, floor_level=0)

            self.display_waypoints_from_graph()

        except Exception as e:
            error_message = e
        return error_message


    #load zones from file
    def load_zones_from_file(self, filename):
        error_message = None  # assume file location and contents are correct
        try:
            #load from file
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)

            #clear existing data
            self.zones.clear()

            
            #transform yaml data to needed data struct
            for zonename, points in data.items():
                poly = []
                for coords in points["nodes"]:                 
                    poly.append([coords["x"], coords["y"]])
                rospy.loginfo("loaded zone " + zonename)
                rospy.loginfo(poly)
                self.zones.update({zonename: shapely.geometry.Polygon(poly)})
                self.zones_controller.update({zonename: points["controller"]})

            rospy.logwarn("loaded all zones")
            self.update_edges_zone()

            #publish zones 
            self.publish_zones()

        except Exception as e:
            error_message = e
        return error_message

    #load zones from shapefile
    def load_zones_from_shp(self, dir):
        error_message = None  # assume file location and contents are correct
        try:

            #clear existing data
            self.zones.clear()

            #check all files in directory
            for file in os.listdir(dir):
                #load if it is a shapefile
                if file.endswith(".shp"):
                    print(os.path.join(dir, file))
                    with shapefile.Reader(os.path.join(dir, file)) as shp:
                        #iterate over all shapes
                        for shapeRecord in shp.iterShapeRecords():
                    

                            poly = []
                            #copy coordinates
                            #do not copy last point, because shapely does not repeat first point at the end
                            for coords in shapeRecord.shape.points[:-1]:
                                poly.append(coords)
                            #get zone name
                            zonename = shapeRecord.record.name
                            # rospy.loginfo("loaded zone " + zonename)
                            # rospy.loginfo(poly)
                            #save to member variables
                            self.zones.update({zonename: shapely.geometry.Polygon(poly)})
                            self.zones_controller.update({zonename: shapeRecord.record.controller})

            rospy.logwarn("loaded all zones")
            self.update_edges_zone()

            #publish zones 
            self.publish_zones()

        except Exception as e:
            error_message = e
        return error_message

    #publishes the zones
    def publish_zones(self):
        #use polygonarray for publishing
        poly_array = PolygonArray()
        poly_array.header.frame_id = self.marker_frame

        #copy all zones to polygonarray
        for _, zone_poly in self.zones.items():
            poly_msg = PolygonStamped()
            poly_msg.header.frame_id = self.marker_frame
            #iterate over points of polygon (do not use last point, because repitition of first in geo_msgs not needed)
            for x, y in zip(zone_poly.exterior.coords.xy[0], zone_poly.exterior.coords.xy[1]):
                poly_msg.polygon.points.append(Point32(x, y, 0))

            poly_array.polygons.append(poly_msg)
            #create ramdom color
            i = randint(0, 255)
            #save as label and likelihood
            poly_array.labels.append(i)
            poly_array.likelihood.append(i)
            
            
        self.zones_pub.publish(poly_array)

    #from networkx graph display markers
    def display_waypoints_from_graph(self):
        #clear markers
        self.clear_all_markers(clear_graph=False)
        self.server.clear()
        self.server.applyChanges()

        #nodes
        for uuid, waypoint_data in self.waypoint_graph.nodes(data=True):
            name = self.uuid_name_map[uuid]
            position = waypoint_data["position"]
            wp_type = waypoint_data['waypoint_type']
            zone1 = waypoint_data["zone1"]
            zone2 = waypoint_data["zone2"]
            #just the nodes of current floor
            if waypoint_data["floor_level"] == self.floor_level:
                self.insert_marker(position=position, uuid=uuid, name=name, waypoint_type=wp_type, floor_level=self.floor_level, zone1=zone1, zone2=zone2)
        #edges
        for u, v, edge_data in self.waypoint_graph.edges(data=True):
            #just the edges of current floor
            if edge_data["floor_level"] == self.floor_level:
                self._connect_markers(u, v, edge_data['edge_type'], floor_level=self.floor_level)

    #service callback: remove an edge
    def remove_edge_service_call(self, request):
        ret = RemoveEdgeResponse()
        #remove edge if existing
        if self.waypoint_graph.has_edge(request.edge.source, request.target):
            self._disconnect_markers(request.edge.source, request.edge.target)
            ret.success = True
        return RemoveEdgeResponse()

    #service callback: get waypoint graph
    def get_waypoint_graph_service_call(self, request):
        if request.completeGraph:
            #complete graph
            graph = self.waypoint_graph
        else:
            #partial graph of waypoints of request
            for r in request.waypoints:
                if not self.waypoint_graph.has_node(str(r)):
                    raise rospy.ServiceException("invalid waypoint {:}".format(r))
            graph = self.waypoint_graph.subgraph(request.waypoints)

        ret = GetWaypointGraphResponse()
        ret.graph = self.get_waypoint_graph_msg(graph)
        return ret

    #create a graph_msgs based on networkx graph
    def get_waypoint_graph_msg(self, graph):
        ret = WaypointGraph()
        ret.name = "graph"

        #nodes
        for uuid, node_data in graph.nodes(data=True):
            n = WaypointNode()
            n.uuid = unique_id.UniqueID(uuid)
            n.name = self.uuid_name_map[str(uuid)]
            n.positions = node_data["position"]
            n.zone1 = node_data["zone1"]
            n.zone2 = node_data["zone2"]
            ret.nodes.append(n)

        #edges
        for u, v, edge_data in graph.edges(data=True):
            e = WaypointEdge()
            e.source = unique_id.UniqueID(u)
            e.target = unique_id.UniqueID(v)
            e.cost = edge_data["cost"]
            #e.path = edge_data["path"]
            ret.edges.append(e)

        return ret

    #service callback: get the shortest path between two start and target pose via network
    def get_shortest_path_service(self, request):
        response = GetShortestPathResponse()
        graph = self.waypoint_graph


        #transform start pose into used frame
        try:
            start_transformed = self.tf_listener.transformPose(self.marker_frame, request.start)
        except Exception as e:
            print(e)
            response.success = False
            response.message = "could not transform startpose into " + self.marker_frame + " frame"
            rospy.logerr("could not transform startpose into " + self.marker_frame + " frame")
            return response
        #transform target pose into used frame
        try:
            target_transformed = self.tf_listener.transformPose(self.marker_frame, request.target)
        except Exception as e:
            print(e)
            response.success = False
            response.message = "could not transform targetpose into " + self.marker_frame + " frame"
            rospy.logerr("could not transform targetpose into " + self.marker_frame + " frame")
            return response

        #use info from msg if not empty
        if request.startzone != "":
            startzone = request.startzone
        else:
            #get the zone geometrybased
            startzone = self.get_zone(start_transformed.pose.position)
            if startzone is None:
                response.success = False
                response.message = "startpoint is in no zone"
                rospy.logerr("startpoint is in no zone")
                return response

        #use info from msg if not empty
        if request.targetzone != "":
            targetzone = request.targetzone
        else:
            #get the zone geometrybased
            targetzone = self.get_zone(target_transformed.pose.position)
            if targetzone is None:
                response.success = False
                response.message = "targetpoint is in no zone"
                rospy.logerr("targetpoint is in no zone")
                return response

        #remove start from network if exists
        if graph.has_node("start"):
            self._remove_marker("start")
        #add start node to network
        self.insert_marker(position=start_transformed.pose, waypoint_type=WAYPOINT_TERMINATING, uuid="start", name="start", zone1=startzone)

        #remove target from network if exists
        if graph.has_node("target"):
            self._remove_marker("target")
        #add target node to network
        self.insert_marker(position=target_transformed.pose, waypoint_type=WAYPOINT_TERMINATING, uuid="target", name="target", zone1=targetzone)

        #connect start / target node if all nodes of the zone
        #all other nodes are on the border between two zones -> they are in TWO zones
        for uuid in graph.nodes():
            if (graph.nodes[uuid]["zone1"] == startzone or graph.nodes[uuid]["zone2"] == startzone) and uuid != "start":
                self._connect_markers(uuid,"start")
            if (graph.nodes[uuid]["zone1"] == targetzone or graph.nodes[uuid]["zone2"] == targetzone) and uuid != "target":
                self._connect_markers(uuid,"target")
        self.update_edges()

        waypoints = nx.shortest_path(self.waypoint_graph, "start", "target", weight='cost')

        rospy.logerr(waypoints)
        response.elevator_required = False
        response.door_service_required = False

        response.waypoints = []
        #add the waypoints of shortest path to response
        for waypoint in waypoints:
            wn = WaypointNode()
            wn.uuid = unique_id.UniqueID(waypoint)
            wn.terminal_waypoint = False
            wn.name = self.uuid_name_map[waypoint]
            if self.waypoint_graph.nodes[waypoint]["waypoint_type"] == WAYPOINT_TERMINATING:
                wn.terminal_waypoint = True
                wp_pose = geometry_msgs.msg.PoseStamped()
                wp_pose.header.frame_id = self.marker_frame
                wp_pose.pose.position = self.waypoint_graph.nodes[waypoint]["position"].position
                wp_pose.pose.orientation = self.waypoint_graph.nodes[waypoint]["position"].orientation

                try:
                    wp_pose_transformed = self.tf_listener.transformPose(request.target.header.frame_id, wp_pose)
                except Exception as e:
                    print(e)
                    response.success = False
                    response.message = "could not transform waypoint into " + request.target.header.frame_id + " frame"
                    rospy.logerr("could not transform waypoint into " + request.target.header.frame_id + " frame")
                    return response

                wn.positions = wp_pose_transformed.pose.position
                wn.orientation =  wp_pose_transformed.pose.orientation

            else:
                wp_point = geometry_msgs.msg.PointStamped()
                wp_point.header.frame_id = self.marker_frame
                wp_point.point = self.waypoint_graph.nodes[waypoint]["position"]

                try:
                    wp_point_transformed = self.tf_listener.transformPoint(request.target.header.frame_id, wp_point)
                except Exception as e:
                    print(e)
                    response.success = False
                    response.message = "could not transform waypoint into " + request.target.header.frame_id + " frame"
                    rospy.logerr("could not transform waypoint into " + request.target.header.frame_id + " frame")
                    return response

                wn.positions = wp_point_transformed.point
                wn.orientation.x = 0.5
                wn.orientation.y = 0.5
                wn.orientation.z = -0.5
                wn.orientation.w = 0.5
            response.waypoints.append(wn)

        #add the global path and the controler of each subpath to the answer
        i = 0
        while i < (len(waypoints)-1):
            edge_type = self.waypoint_graph.get_edge_data(waypoints[i], waypoints[i+1])['edge_type']
            if edge_type == EDGE_ELEVATOR:
                response.waypoints[i].elevator_required = response.waypoints[i].name
                response.elevator_required = True
            elif edge_type == EDGE_DOOR:
                response.waypoints[i].door_required = response.waypoints[i].name
                response.door_service_required = True
            response.path.append(self.waypoint_graph.get_edge_data(waypoints[i], waypoints[i+1])['path'])
            response.controller.append(self.waypoint_graph.get_edge_data(waypoints[i], waypoints[i+1]).get('controller',None))
            i += 1

        self.show_active_path(waypoints)
        response.success = True
        rospy.loginfo("Waypoints:")
        rospy.loginfo(waypoints)
        rospy.loginfo("controller:")
        rospy.loginfo(response.controller)

        return response
    
    #service callback: get the zone  of a point and controller of the zone
    def get_zone_service(self, request):
        response = GetZoneResponse()

        try:
            point_transformed = self.tf_listener.transformPoint(self.marker_frame, request.point)
            zone = self.get_zone(point_transformed)
        except Exception as e:
            print(e)
            zone = None

        if zone is None:
            response.success = False
            response.message = "point is in no zone"
        else:
            controller = self.zones_controller.get(zone, None)
            response.success = True
            response.zone = zone
            response.controller = controller

        return response

    #get the zone geometrybased
    def get_zone(self, point):
        #point in polygon test from shapely
        for zone, poly in self.zones.items():
            if poly.contains(shapely.geometry.Point(point.x, point.y)):
                return zone

        return None #never reached except no zone found

    #show the active path in rviz via different color
    def show_active_path(self, waypoints):
        self.clear_active_path()  # clear previous
        self.active_waypoints = waypoints
        edges = MarkerArray()
        i = 0
        while i < (len(waypoints)-1):
            u = waypoints[i]
            v = waypoints[i+1]
            if self.waypoint_graph.has_edge(u, v):
                marker = self.waypoint_graph.get_edge_data(u, v)["marker"]
                marker.color.r = min(1, marker.color.r+0.4)
                marker.color.g = min(1, marker.color.g+0.4)
                marker.color.b = min(1, marker.color.b+0.4)
                marker.scale.x = 0.2
                edges.markers.append(marker)
            i += 1
        self.edge_line_publisher.publish(edges)

    #clear the active path, all edges to normal
    def clear_active_path(self):
        if self.active_waypoints is None:
            return
        waypoints = self.active_waypoints
        edges = MarkerArray()
        i = 0
        while i < (len(waypoints)-1):
            u = waypoints[i]
            v = waypoints[i+1]
            if self.waypoint_graph.has_edge(u, v):
                marker = self.waypoint_graph.get_edge_data(u, v)["marker"]
                edge_type = self.waypoint_graph.get_edge_data(u, v)["edge_type"]
                self._set_edge_color(marker, edge_type)
                marker.scale.x = 0.09
                edges.markers.append(marker)
            i += 1
        self.edge_line_publisher.publish(edges)

    #set the floor level
    def set_floor_level(self, request):
        response = SetFloorLevelResponse()
        prev_floor_level = self.floor_level

        self.floor_level = request.floor_level
        error_message = self.display_waypoints_from_graph()
        if error_message == None:
            response.success = True
            response.message = "Loaded waypoints from floor_level " + str(self.floor_level)
            rospy.loginfo("Loaded waypoints from floor_level {0}".format(self.floor_level))
        else:
            response.success = False
            self.floor_level = prev_floor_level
            response.message = str(error_message)
            rospy.logwarn("Failed to load waypoints from floor_level: {0}".format(request.floor_level))
        return response

    # def get_waypoints_within_distance(self, x, y, dist_limit=4):
    #     """Return all waypoints within 'dist' metres of x and y point"""
    #     waypoints_distance_map = {}
    #     for waypoint in self.waypoint_graph.nodes:
    #         wp = self.waypoint_graph.node[waypoint]
    #         if wp["floor_level"] == self.floor_level and wp["waypoint_type"] == WAYPOINT_REGULAR:  # could remove waypoint type check
    #             start = Point()
    #             start.x = x
    #             start.y = y
    #             goal = Point()
    #             goal.x = wp["position"].x
    #             goal.y = wp["position"].y
    #             dist = euclidean_distance(start, goal)
    #             if dist < dist_limit:
    #                 waypoints_distance_map[waypoint] = euclidean_distance(start, goal) #self.path_manager.efficient_path(start, goal)
    #     return waypoints_distance_map

    #get the path via mbf
    def get_path(self, point1, point2):
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient('move_base_flex/get_path', GetPathAction)
        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        startpose = PoseStamped()
        startpose.header.frame_id = self.marker_frame
        startpose.pose = point1

        targetpose = PoseStamped()
        targetpose.header.frame_id = self.marker_frame
        targetpose.pose = point2

        
        try:
            self.tf_listener.waitForTransform(target_frame="map",source_frame=self.marker_frame,time=rospy.Time(0),timeout=rospy.Duration(10.0))
            startpose_map = self.tf_listener.transformPose("map", startpose)
            targetpose_map = self.tf_listener.transformPose("map", targetpose)
        except Exception as e:
            print(e)
            return e


        # Creates a goal to send to the action server.
        goal = GetPathGoal(use_start_pose=True,start_pose=startpose_map,target_pose=targetpose_map,tolerance=0.1,planner="planner")

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        return client.get_result()


if __name__ == "__main__":
    rospy.init_node("waypoint_server")
    rospy.loginfo("(Waypoint_Server) initializing...")
    server = WaypointServer()
    rospy.loginfo("[Waypoint_Server] ready.")
    rospy.spin()
