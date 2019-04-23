#!/usr/bin/python2
import operator  # sorting dictionaries
import rospy
from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from text_box_input import InputWindow
from waypoint_msgs.msg import WaypointEdge, WaypointNode, WaypointGraph

import numpy
import sys
from PyQt5 import QtCore, QtWidgets

from waypoint_msgs.srv import RemoveEdge, RemoveEdgeResponse
from waypoint_msgs.srv import SaveWaypoints, SaveWaypointsResponse
from waypoint_msgs.srv import LoadWaypoints, LoadWaypointsResponse
from waypoint_msgs.srv import GetWaypointGraph, GetWaypointGraphResponse
from waypoint_msgs.srv import GetShortestPath, GetShortestPathResponse
from waypoint_msgs.srv import SetFloorLevel, SetFloorLevelResponse
from waypoint_msgs.srv import LoadDoorData, LoadDoorDataResponse
from uuid_msgs.msg import UniqueID
import unique_id


from visualization_msgs.msg import *
from geometry_msgs.msg import PointStamped, PoseStamped, PoseStamped, Point, Quaternion, Pose
import networkx as nx
import math
import yaml

from shortest_path import ShortestPath

from constants import *

def euclidean_distance(point1, point2):
    return math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2 + (point2.z - point1.z)**2)

class WaypointServer:
    def __init__(self):
        self.server = InteractiveMarkerServer("waypoint_server")
        self.waypoint_graph = nx.Graph()
        self.next_waypoint_id = 0
        self.next_edge_id = 0
        self.floor_level = rospy.get_param("~floor_level", 0)
        self.state = STATE_REGULAR
        self.active_waypoints = None
        self.connect_from_marker = ""
        self.edge_type = EDGE_REGULAR
        self.waypoint_type = WAYPOINT_REGULAR
        self.edge_line_publisher = rospy.Publisher("~edges", MarkerArray, queue_size=10)
        self.marker_frame = rospy.get_param("~marker_frame", "map")
        self.uuid_name_map = {}
        self.door_data = {}
        self.filename = ""

        self.path_manager = ShortestPath()  # for checking closest waypoints by path

        self.removeService = rospy.Service('~remove_edge', RemoveEdge, self.remove_edge_service_call)
        self.loadService = rospy.Service('~load_waypoints', LoadWaypoints, self.load_waypoints_service)
        self.saveService = rospy.Service('~save_waypoints', SaveWaypoints, self.save_waypoints_service)
        self.getShortestPathService = rospy.Service('~get_shortest_path', GetShortestPath, self.get_shortest_path_service)
        self.getWaypointGraphService = rospy.Service('~get_waypoint_graph', GetWaypointGraph, self.get_waypoint_graph_service_call)
        self.setFloorLevelService = rospy.Service('~set_floor_level', SetFloorLevel, self.set_floor_level)
        self.loadDoorDataService = rospy.Service('~load_door_data', LoadDoorData, self.load_door_data_service)
        rospy.Subscriber("/clicked_point", PointStamped, self.insert_marker_callback)
        rospy.Subscriber("/clicked_pose", PoseStamped, self.insert_terminating_marker_callback)
        rospy.on_shutdown(self.clear_all_markers)
        rospy.logwarn("The waypoint server is waiting for RViz to run and to be subscribed to {0}.".format(rospy.resolve_name("~edges")))
        while self.edge_line_publisher.get_num_connections() == 0:
            rospy.sleep(1.0)

        self.clear_all_markers()

        load_file = rospy.get_param("~waypoint_file", "") # yaml file with waypoints to load
        door_file = rospy.get_param("~door_file", "")

        if len(load_file) != 0:
            rospy.loginfo("Waypoint_Server is loading initial waypoint file {0}.".format(load_file))
            error_message = self.load_waypoints_from_file(load_file)
            if error_message:
                rospy.logerr(error_message)
        if len(door_file) != 0:
            rospy.loginfo("Door descriptions loaded from file {0}".format(door_file))
            error_message = self.load_door_data_from_file(door_file)
            if error_message:
                rospy.logerr(error_message)

    def load_door_data_from_file(self, filename):
        error_message = None
        try:
            with open(filename, 'r') as f:
                data = yaml.load(f)

            self.door_data = data

        except Exception as e:
            self.door_data = {}
            error_message = e
        return error_message

    def load_door_data_service(self, req):
        response = LoadDoorDataResponse()
        response.message = None
        error_message = self.load_door_data_from_file(req.file_name)

        if error_message:
            response.success = False
            response.message = str(error_message)
        else:
            response.success = True
            response.message = "loaded door points from {0}".format(req.file_name)
        return response

    def find_door_edge(self, u, v):
        edge_door = None

        dp1 = Point()
        dp2 = Point()

        for door, door_data in self.door_data.iteritems():
            dp1.x = door_data['start']['x']
            dp1.y = door_data['start']['y']

            dp2.x = door_data['end']['x']
            dp2.y = door_data['end']['y']

            if self.is_intersection(u, v, dp1, dp2):
                edge_door = door
                break

        return edge_door

    def is_intersection(self, u, v, p1, p2):
        o1 = self.line_orientation(u,v,p1)
        o2 = self.line_orientation(u,v,p2)
        o3 = self.line_orientation(p1,p2,u)
        o4 = self.line_orientation(p1,p2,v)

        if o1 != o2 and o3 != o4:
            return True
        else:
            return False

    def line_orientation(self, a, b, c):
        Area = (b.y - a.y)*(c.x - b.x) - (b.x - a.x)*(c.y - b.y)

        if (Area < 0):
            return CLOCKWISE
        elif (Area > 0):
            return COUNTERCLOCKWISE
        else:
            return COLINEAR

    def insert_marker_callback(self, pos):
        rospy.logdebug("Inserting new waypoint at position ({0},{1},{2}).".format(pos.point.x, pos.point.y, pos.point.z))
        self.waypoint_type = WAYPOINT_REGULAR
        self.insert_marker(pos.point, floor_level=self.floor_level)

    def insert_terminating_marker_callback(self, pos):
        rospy.logdebug("Inserting new terminating waypoint at position ({0},{1},{2}).".format(pos.point.x, pos.point.y, pos.point.z))
        self.waypoint_type = WAYPOINT_TERMINATING
        self.insert_marker(pos.pose, waypoint_type=WAYPOINT_TERMINATING, floor_level=self.floor_level)

    def clear_all_markers(self, clear_graph=True):
        edges = MarkerArray()
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.marker_frame
        marker.ns = "waypoint_edges"
        marker.id = 0
        marker.action = Marker.DELETEALL
        edges.markers.append(marker)
        self.edge_line_publisher.publish(edges)
        if clear_graph:
            self.waypoint_graph.clear()

    def _make_marker(self, msg):
        marker = Marker()
        if self.waypoint_type == WAYPOINT_REGULAR:
            marker.type = Marker.SPHERE
            marker.scale.x = msg.scale * 0.45
            marker.scale.y = msg.scale * 0.45
            marker.scale.z = msg.scale * 0.45
            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0
            marker.color.a = 1.0
        elif self.waypoint_type == WAYPOINT_TERMINATING:
            marker.type = Marker.ARROW
            marker.pose.orientation = Quaternion(1, 0, 0, 0)
            marker.scale.x = msg.scale * 0.5
            marker.scale.y = msg.scale * 0.25
            marker.scale.z = msg.scale * 0.45
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 1.0

        return marker

    def _make_edge(self, scale, begin, end, edge_type=EDGE_REGULAR):
        edge = Marker()
        edge.header.frame_id = self.marker_frame
        edge.header.stamp = rospy.Time.now()
        edge.ns = "waypoint_edges"
        edge.id = self.next_edge_id
        self.next_edge_id += 1
        edge.type = Marker.LINE_LIST
        edge.action = Marker.ADD
        edge.scale.x = scale * 0.45
        edge.scale.y = scale * 0.45
        edge.scale.z = scale * 0.45
        self._set_edge_color(edge, edge_type)
        edge.points.append(begin)
        edge.points.append(end)
        return edge

    def _set_edge_color(self, edge, edge_type):
        if edge_type == EDGE_REGULAR:
            edge.color.r = EDGE_REGULAR_COLOR.r
            edge.color.g = EDGE_REGULAR_COLOR.g
            edge.color.b = EDGE_REGULAR_COLOR.b
        elif edge_type == EDGE_DOOR:
            edge.color.r = EDGE_DOOR_COLOR.r
            edge.color.g = EDGE_DOOR_COLOR.g
            edge.color.b = EDGE_DOOR_COLOR.b
        elif edge_type == EDGE_ELEVATOR:
            edge.color.r = EDGE_ELEVATOR_COLOR.r
            edge.color.g = EDGE_ELEVATOR_COLOR.g
            edge.color.b = EDGE_ELEVATOR_COLOR.b
        edge.color.a = 1

    def _rename_marker(self, name):
        old_name = self.uuid_name_map[name]
        app = QtWidgets.QApplication.instance()
        if app is None:
            app = QtWidgets.QApplication(sys.argv)
            rename_popup_window = InputWindow(old_name)
            rename_popup_window.show()
            app.exec_()
            new_name = rename_popup_window.getNewName()
            app.exit()
            del(app)

        InteractiveMarker = self.server.get(name)
        InteractiveMarker.description = new_name
        self.uuid_name_map.update({name: new_name})
        self.server.insert(InteractiveMarker)
        self.server.applyChanges()
        if new_name != old_name:
            rospy.loginfo("changed waypoint name from '{0}' to '{1}'".format(old_name, new_name))

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

    def _connect_markers(self, u, v, cost=1.0, edge_type=EDGE_REGULAR, floor_level=0):
            if u == v:
                rospy.logwarn("Cannot connect a marker to itself")
                return
            u_pos = self.server.get(u).pose.position
            v_pos = self.server.get(v).pose.position
            cost = euclidean_distance(u_pos,v_pos)
            # insert edge
            edge = self._make_edge(0.2, u_pos, v_pos, edge_type)
            edge.text = str(cost)
            # insert edge into graph
            self.waypoint_graph.add_edge(u, v, u=u, v=v, cost=cost, edge_type=edge_type, marker=edge, floor_level=floor_level)
            self.update_edges()

    def set_marker_highlight(self, highlight=True, marker=None):
        if marker == None:
            marker = self.active_marker
        m = self.server.get(self.active_marker)
        m.controls[0].markers[0].color.b = 1 if highlight==True else 0
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
        for u, v, marker in self.waypoint_graph.edges(data=True):
            if marker["floor_level"] == self.floor_level:
                edges.markers.append(marker["marker"])
        self.edge_line_publisher.publish(edges)

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            handle = feedback.menu_entry_id
            if handle == MENU_CONNECT_EDGE:
                self.state = STATE_CONNECT
                self.edge_type = EDGE_REGULAR
                self.active_marker = feedback.marker_name
                self.connect_from_marker = feedback.marker_name
                self.set_marker_highlight(marker=feedback.marker_name)
            elif handle == MENU_CONNECT_EDGE_DOOR:
                self.state = STATE_CONNECT
                self.edge_type = EDGE_DOOR
                self.active_marker = feedback.marker_name
                self.connect_from_marker = feedback.marker_name
                self.set_marker_highlight(marker=feedback.marker_name)
            elif handle == MENU_CONNECT_EDGE_ELEVATOR:
                self.state = STATE_CONNECT
                self.edge_type = EDGE_ELEVATOR
                self.active_marker = feedback.marker_name
                self.connect_from_marker = feedback.marker_name
                self.set_marker_highlight(marker=feedback.marker_name)
            elif handle == MENU_DISCONNECT_EDGE:
                self.state = STATE_DISCONNECT
                self.active_marker = feedback.marker_name
                self.connect_from_marker = feedback.marker_name
                self.set_marker_highlight(marker=feedback.marker_name)
            elif handle == MENU_CLEAR:
                self._clear_marker_edges(feedback.marker_name)
            elif handle == MENU_RENAME:
                self._rename_marker(feedback.marker_name)
            elif handle == MENU_REMOVE:
                self._remove_marker(feedback.marker_name)

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if self.state == STATE_CONNECT:
                self.state = STATE_NONE
                self._connect_markers(self.connect_from_marker, feedback.marker_name, edge_type=self.edge_type, floor_level=self.floor_level)
                self.set_marker_highlight(highlight=False)
            elif self.state == STATE_DISCONNECT:
                self.state = STATE_NONE
                self._disconnect_markers(self.connect_from_marker, feedback.marker_name)
                self.set_marker_highlight(highlight=False)
            elif self.state == STATE_NONE:
                pass    # ignore
            else:
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

    def move_feedback(self, feedback):
        pose = feedback.pose

        self.server.setPose(feedback.marker_name, pose)

        # correct all edges
        for u, v, data in self.waypoint_graph.edges([feedback.marker_name], data=True):
            if feedback.marker_name == data["u"]:
                data["marker"].points[0] = pose.position
            else:
                data["marker"].points[1] = pose.position
            u_pos = self.server.get(u).pose
            v_pos = self.server.get(v).pose
            if self.waypoint_graph.node[u]["waypoint_type"] == WAYPOINT_REGULAR:
                self.waypoint_graph.node[u]["position"] = u_pos.position
            elif self.waypoint_graph.node[u]["waypoint_type"] == WAYPOINT_TERMINATING:
                self.waypoint_graph.node[u]["position"] = u_pos
            if self.waypoint_graph.node[v]["waypoint_type"] == WAYPOINT_REGULAR:
                self.waypoint_graph.node[v]["position"] = v_pos.position
            elif self.waypoint_graph.node[v]["waypoint_type"] == WAYPOINT_TERMINATING:
                self.waypoint_graph.node[v]["position"] = v_pos
            data["cost"] = euclidean_distance(u_pos.position,v_pos.position)
        self.update_edges()
        self.server.applyChanges()

    def insert_marker(self, position, waypoint_type=WAYPOINT_REGULAR, name=None, uuid=None, frame_id=None, floor_level=0):
        if frame_id is None:
            frame_id = self.marker_frame

        if uuid is None:
            uuid = unique_id.fromRandom()

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
            int_marker.pose.orientation.w = 1
        elif waypoint_type == WAYPOINT_TERMINATING:
            int_marker.pose.position = position.position
            int_marker.pose.orientation = position.orientation
        int_marker.scale = 0.5
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
        self.waypoint_graph.add_node(str(uuid), waypoint_type=waypoint_type, floor_level=floor_level, position=position)

    def save_waypoints_service(self, request):
        if request.file_name:
            filename = request.file_name
        else:
            filename = self.filename

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

    def load_waypoints_service(self, request):
        self.filename = request.file_name
        response = LoadWaypointsResponse()
        error_message = self.load_waypoints_from_file(self.filename)

        if error_message == None:
            response.success = True
            response.message = "Loaded waypoints from " + str(self.filename)
            rospy.loginfo("Loaded waypoints from {0}".format(self.filename))
        else:
            response.success = False
            response.message = str(error_message)
            rospy.loginfo("Failed to load waypoints: {0}".format(error_message))
        return response

    def save_waypoints_to_file(self, filename):
        error_message = None  # assume file location and contents are correct
        try:
            data = {"waypoints": {}, "edges": []}

            if self.filename and not filename:
                filename = self.filename

            for uuid, waypoint_data in self.waypoint_graph.nodes(data=True):
                name = self.uuid_name_map[uuid]
                waypoint_type = waypoint_data['waypoint_type']
                floor_level = waypoint_data['floor_level']
                if waypoint_type == WAYPOINT_REGULAR:
                    pos = waypoint_data["position"]
                    data["waypoints"].update({uuid: {"name": name, "x": pos.x, "y": pos.y, "z": pos.z, "floor_level":floor_level,
                    "waypoint_type":waypoint_type}})
                elif waypoint_type == WAYPOINT_TERMINATING:
                    pos = waypoint_data["position"].position
                    orientation = waypoint_data["position"].orientation
                    data["waypoints"].update({uuid: {"name": name, "x": pos.x, "y": pos.y, "z": pos.z, "floor_level":floor_level,
                    "waypoint_type":waypoint_type, "orientation":{"x":orientation.x, "y":orientation.y, "w":orientation.w, "z":orientation.z}}})
            for u, v, edge_data in self.waypoint_graph.edges(data=True):
                cost = edge_data['cost']
                edge_type = edge_data['edge_type']
                floor_level = edge_data['floor_level']
                data["edges"].append({'u': u, 'v': v, 'cost': cost, 'edge_type': edge_type, "floor_level":floor_level})

            with open(filename, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
        except Exception as e:
            error_message = e
        return error_message

    def load_waypoints_from_file(self, filename):
        error_message = None  # assume file location and contents are correct
        try:
            with open(filename, 'r') as f:
                data = yaml.load(f)

            self.clear_all_markers()
            self.server.clear()
            self.server.applyChanges()

            for uuid, wp in data["waypoints"].items():
                wp_type = wp["waypoint_type"]
                if wp_type == WAYPOINT_REGULAR:
                    point = Point(wp["x"], wp["y"], wp["z"])
                elif wp_type == WAYPOINT_TERMINATING:
                    point = Pose()
                    point.position = Point(wp["x"], wp["y"], wp["z"])
                    point.orientation = Quaternion(wp["orientation"]["x"], wp["orientation"]["y"],
                        wp["orientation"]["z"], wp["orientation"]["w"])
                self.insert_marker(position=point, uuid=uuid, name=wp['name'], waypoint_type=wp_type, floor_level=wp["floor_level"])

            for edge in data["edges"]:
                self._connect_markers(edge['u'], edge['v'], edge['cost'], edge['edge_type'], floor_level=edge["floor_level"])

            self.display_waypoints_from_graph()

        except Exception as e:
            error_message = e
        return error_message

    def display_waypoints_from_graph(self):
        self.clear_all_markers(clear_graph=False)
        self.server.clear()
        self.server.applyChanges()

        for uuid, waypoint_data in self.waypoint_graph.nodes(data=True):
            name = self.uuid_name_map[uuid]
            position = waypoint_data["position"]
            wp_type = waypoint_data['waypoint_type']
            if waypoint_data["floor_level"] == self.floor_level:
                self.insert_marker(position=position, uuid=uuid, name=name, waypoint_type=wp_type, floor_level=self.floor_level)
        for u, v, edge_data in self.waypoint_graph.edges(data=True):
            if edge_data["floor_level"] == self.floor_level:
                self._connect_markers(u, v, edge_data['cost'], edge_data['edge_type'], floor_level=self.floor_level)

    def remove_edge_service_call(self, request):
        if self.waypoint_graph.has_edge(request.u, request.v):
            self._disconnect_markers(request.u, request.v)

        return RemoveConnectionResponse()

    def get_waypoint_graph_service_call(self, request):
        ret = GetWaypointGraphResponse()
        for r in request.waypoints:
            if not self.waypoint_graph.has_node(str(r)):
                raise rospy.ServiceException("invalid waypoint {:}".format(r))

        if len(request.waypoints) == 0:
            graph = self.waypoint_graph
        else:
            graph = self.waypoint_graph.subgraph(request.waypoints)

        for node in graph.nodes():
            pose = self.server.get(node).pose
            ret.names.append(node)
            ret.positions.append(pose)

        for u, v in graph.edges():
            e = Edge()
            e.source = u
            e.target = v
            ret.edges.append(e)

        return ret

    def get_shortest_path_service(self, request):
        response = GetShortestPathResponse()
        # ensure waypoints robot is in between exist in graph
        if not request.u in self.uuid_name_map or not request.v in self.uuid_name_map:
            response.success = False
            response.message = "The u or v does not match a waypoint"
            rospy.logwarn("get_shortest_path_service: The u or v does not match a waypoint")
            return response

        # ensure target in request exists in graph
        matching_targets = [k for k, v in self.uuid_name_map.items() if request.target in v]
        if not matching_targets:
            response.success = False
            response.message = "The target does not match a waypoint"
            rospy.logwarn("get_shortest_path_service: The target does not match a waypoint")
            return response
        # compute shortest path between each initial waypoint and target
        shortest_path_length = 100000000000000000
        for target in matching_targets:
            u_path = nx.shortest_path_length(self.waypoint_graph, request.u, target, weight='cost')
            v_path = nx.shortest_path_length(self.waypoint_graph, request.v, target, weight='cost')
            shortest_path_length = min(shortest_path_length, u_path, v_path)
            if shortest_path_length == u_path:
                waypoints = nx.shortest_path(self.waypoint_graph, request.u, target, weight='cost')
            elif shortest_path_length == v_path:
                waypoints = nx.shortest_path(self.waypoint_graph, request.v, target, weight='cost')

        response.elevator_required = False
        response.door_service_required = False

        response.waypoints = []
        for waypoint in waypoints:
            wn = WaypointNode()
            wn.uuid = waypoint
            wn.terminating_waypoint = False
            wn.name = self.uuid_name_map[waypoint]
            if self.waypoint_graph.nodes[waypoint]["waypoint_type"] == WAYPOINT_TERMINATING:
                wn.terminating_waypoint = True
                wn.positions = self.waypoint_graph.nodes[waypoint]["position"].position
                wn.orientation = self.waypoint_graph.nodes[waypoint]["position"].orientation
            else:
                wn.positions = self.waypoint_graph.nodes[waypoint]["position"]
            response.waypoints.append(wn)

        i = 0
        while i < (len(waypoints)-1):
            edge_type = self.waypoint_graph.get_edge_data(waypoints[i], waypoints[i+1])['edge_type']
            if edge_type == EDGE_ELEVATOR:
                response.elevator_required = True
            elif edge_type == EDGE_DOOR:
                if self.door_data:
                    point1 = self.waypoint_graph.nodes[waypoints[i]]["position"]
                    if self.waypoint_graph.nodes[waypoints[i+1]]["waypoint_type"] == WAYPOINT_TERMINATING:
                        point2 = self.waypoint_graph.nodes[waypoints[i+1]]["position"].position
                    else:
                        point2 = self.waypoint_graph.nodes[waypoints[i+1]]["position"]
                    door_required = self.find_door_edge(point1, point2)
                    if door_required not in response.doors_required:
                        response.doors_required.append(door_required)
                    response.waypoints[i].door_required = door_required
                response.door_service_required =True
            i +=1

        self.show_active_path(waypoints)
        response.success = True
        rospy.loginfo("Waypoints:")
        rospy.loginfo(waypoints)

        return response

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


    def set_floor_level(self, request):
        response = SetFloorLevelResponse()
        filename = self.filename
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
            rospy.loginfo("Failed to load waypoints from floor_level: {0}".format(request.floor_level))
        return response

    def get_closest_waypoints_euclidean(self, x, y, amt=5):
        """Return 'amt' closest waypoints of a position, based on euclidean distance"""
        waypoints_distance_map = {}
        for waypoint in self.waypoint_graph.nodes:
            wp = self.waypoint_graph.node[waypoint]
            if wp["floor_level"] == self.floor_level and wp["waypoint_type"] == WAYPOINT_REGULAR:  # could remove waypoint type check
                start = Point()
                start.x = x
                start.y = y
                goal = Point()
                goal.x = wp["position"].x
                goal.y = wp["position"].y
                dist = euclidean_distance(start, goal)
                waypoints_distance_map[waypoint] = dist
                print dist
        closest_waypoints_tuple = sorted(waypoints_distance_map.items(), key=operator.itemgetter(1))
        closest_waypoints = [x[0] for x in closest_waypoints_tuple[:amt]]
        print closest_waypoints
        return closest_waypoints

    def get_closest_waypoint(self, x, y):
        closest_euclidean = self.get_closest_waypoints_euclidean(x,y)
        start = Point()
        start.x = x
        start.y = y
        goal = Point()
        shortest_length = 999999999
        closest_waypoint = None
        for waypoint in closest_euclidean:
            print "for loop x {}".format(x)
            print "for loop y {}".format(y)
            wp = self.waypoint_graph.node[waypoint]
            goal.x = wp["position"].x
            goal.y = wp["position"].y
            length = self.path_manager.efficient_path(start, goal)
            if length < shortest_length:
                shortest_length = length
                closest_waypoint = waypoint
            raw_input("Press Enter to continue...")
        print self.uuid_name_map[closest_waypoint]
        return closest_waypoint

if __name__ == "__main__":
    rospy.init_node("waypoint_server")
    rospy.loginfo("(Waypoint_Server) initializing...")
    server = WaypointServer()
    rospy.loginfo("[Waypoint_Server] ready.")
    rospy.sleep(5)
    server.get_closest_waypoint(152, 24)
    rospy.spin()
