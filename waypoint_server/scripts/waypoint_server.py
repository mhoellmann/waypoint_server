#!/usr/bin/python2
import rospy
from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from text_box_input import InputWindow
from waypoint_msgs.msg import WaypointEdge, WaypointNode, WaypointGraph

import sys
from PyQt5 import QtCore, QtWidgets

from waypoint_msgs.srv import RemoveEdge, RemoveEdgeResponse
from waypoint_msgs.srv import SaveWaypoints, SaveWaypointsResponse
from waypoint_msgs.srv import LoadWaypoints, LoadWaypointsResponse
from waypoint_msgs.srv import GetWaypointGraph, GetWaypointGraphResponse
from waypoint_msgs.srv import GetShortestPath, GetShortestPathResponse
from waypoint_msgs.srv import SetFloorLevel, SetFloorLevelResponse

from uuid_msgs.msg import UniqueID
import unique_id


from visualization_msgs.msg import *
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion, Pose
import networkx as nx
import math
import yaml

# define menu entry item ids (top menu item must start from 1)
MENU_CONNECT_EDGE = 1
MENU_CONNECT_EDGE_DOOR = 2
MENU_CONNECT_EDGE_ELEVATOR = 3
MENU_DISCONNECT_EDGE = 4
MENU_CLEAR = 5
MENU_RENAME = 6
MENU_REMOVE = 7

# define program states
STATE_REGULAR = 0
STATE_CONNECT = 1
STATE_DISCONNECT = 2
STATE_NONE = 3

# define edge types
EDGE_REGULAR = 0
EDGE_DOOR = 1
EDGE_ELEVATOR = 2

# define edge type colors
EDGE_REGULAR_COLOR = ColorRGBA()
EDGE_REGULAR_COLOR.r = 0
EDGE_REGULAR_COLOR.g = 0
EDGE_REGULAR_COLOR.b = 1
EDGE_REGULAR_COLOR.a = 1

EDGE_DOOR_COLOR = ColorRGBA()
EDGE_DOOR_COLOR.r = 0
EDGE_DOOR_COLOR.g = 1
EDGE_DOOR_COLOR.b = 1
EDGE_DOOR_COLOR.a = 1

EDGE_ELEVATOR_COLOR = ColorRGBA()
EDGE_ELEVATOR_COLOR.r = 0
EDGE_ELEVATOR_COLOR.g = 1
EDGE_ELEVATOR_COLOR.b = 0
EDGE_ELEVATOR_COLOR.a = 1

# define waypoint types
WAYPOINT_REGULAR = 0
WAYPOINT_TERMINATING = 1

def euclidean_distance(point1, point2):
    return math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2 + (point2.z - point1.z)**2)


class WaypointServer:
    def __init__(self):
        self.server = InteractiveMarkerServer("waypoint_server")
        self.waypoint_graph = nx.Graph()
        self.next_waypoint_id = 0
        self.next_edge_id = 0
        self.floor_level = 0
        self.state = STATE_REGULAR
        self.active_waypoints = None
        self.connect_from_marker = ""
        self.edge_type = EDGE_REGULAR
        self.waypoint_type = WAYPOINT_REGULAR
        self.edge_line_publisher = rospy.Publisher("~edges", MarkerArray, queue_size=10)
        self.marker_frame = rospy.get_param("~marker_frame", "map")
        self.uuid_name_map = {}
        self.filename = ""

        self.removeService = rospy.Service('~remove_edge', RemoveEdge, self.remove_edge_service_call)
        self.loadService = rospy.Service('~load_waypoints', LoadWaypoints, self.load_waypoints_service)
        self.saveService = rospy.Service('~save_waypoints', SaveWaypoints, self.save_waypoints_service)
        self.getShortestPathService = rospy.Service('~get_shortest_path', GetShortestPath, self.get_shortest_path_service)
        self.getWaypointGraphService = rospy.Service('~get_waypoint_graph', GetWaypointGraph, self.get_waypoint_graph_service_call)
        self.setFloorLevelService = rospy.Service('~set_floor_level', SetFloorLevel, self.set_floor_level)

        rospy.Subscriber("/clicked_point", PointStamped, self.insert_marker_callback)
        rospy.Subscriber("/clicked_pose", PoseWithCovarianceStamped, self.insert_terminating_marker_callback)
        rospy.on_shutdown(self.clear_all_markers)
        rospy.logwarn("The waypoint server is waiting for RViz to run and to be subscribed to {0}.".format(rospy.resolve_name("~edges")))
        while self.edge_line_publisher.get_num_connections() == 0:
            rospy.sleep(1.0)

        self.clear_all_markers()

        load_file = rospy.get_param("~waypoint_file", "") # yaml file with waypoints to load

        if len(load_file) != 0:
            rospy.loginfo("Waypoint_Server is loading initial waypoint file {0}.".format(load_file))
            self.load_waypoints_from_file(load_file)

    def insert_marker_callback(self, pos):
        rospy.logdebug("Inserting new waypoint at position ({0},{1},{2}).".format(pos.point.x, pos.point.y, pos.point.z))
        self.waypoint_type = WAYPOINT_REGULAR
        self.insert_marker(pos.point)

    def insert_terminating_marker_callback(self, pos):
        rospy.logdebug("Inserting new")
        self.waypoint_type = WAYPOINT_TERMINATING
        self.insert_marker(pos.pose.pose, waypoint_type=WAYPOINT_TERMINATING)

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

    def _connect_markers(self, u, v, cost=1.0, edge_type=EDGE_REGULAR):
            name_u = self.uuid_name_map[u]
            name_v = self.uuid_name_map[v]
            if name_u == name_v:
                rospy.logwarn("Cannot connect a marker to itself")
                return
            u_pos = self.server.get(u).pose.position
            v_pos = self.server.get(v).pose.position
            cost = euclidean_distance(u_pos,v_pos)
            # insert edge
            edge = self._make_edge(0.2, u_pos, v_pos, edge_type)
            edge.text = str(cost)
            # insert edge into graph
            self.waypoint_graph.add_edge(u, v, u=u, v=v, cost=cost, edge_type=edge_type, marker=edge, floor_level=self.floor_level)
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
                self._connect_markers(self.connect_from_marker, feedback.marker_name, edge_type=self.edge_type)
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

    def insert_marker(self, position, waypoint_type=WAYPOINT_REGULAR, name=None, uuid=None, frame_id=None):
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
        self.waypoint_graph.add_node(str(uuid), waypoint_type=waypoint_type, floor_level=self.floor_level, position=position)

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
                self.floor_level = wp["floor_level"]
                if wp_type == WAYPOINT_REGULAR:
                    point = Point(wp["x"], wp["y"], wp["z"])
                elif wp_type == WAYPOINT_TERMINATING:
                    point = Pose()
                    point.position = Point(wp["x"], wp["y"], wp["z"])
                    point.orientation = Quaternion(wp["orientation"]["x"], wp["orientation"]["y"],
                        wp["orientation"]["z"], wp["orientation"]["w"])
                self.insert_marker(position=point, uuid=uuid, name=wp['name'], waypoint_type=wp_type)

            for edge in data["edges"]:
                self.floor_level = edge["floor_level"]
                self._connect_markers(edge['u'], edge['v'], edge['cost'], edge['edge_type'])

            self.floor_level = 0
            self.load_waypoints_from_graph()

        except Exception as e:
            error_message = e
        return error_message

    def load_waypoints_from_graph(self):
        error_message = None  # assume file location and contents are correct
        try:
            self.clear_all_markers(clear_graph=False)
            self.server.clear()
            self.server.applyChanges()

            for uuid, waypoint_data in self.waypoint_graph.nodes(data=True):
                name = self.uuid_name_map[uuid]
                position = waypoint_data["position"]
                wp_type = waypoint_data['waypoint_type']
                if waypoint_data["floor_level"] == self.floor_level:
                    self.insert_marker(position=position, uuid=uuid, name=name, waypoint_type=wp_type)

            for u, v, edge_data in self.waypoint_graph.edges(data=True):
                if edge_data["floor_level"] == self.floor_level:
                    self._connect_markers(u, v, edge_data['cost'], edge_data['edge_type'])

        except Exception as e:
            error_message = e
        return error_message

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
        error_message = self.load_waypoints_from_graph()
        if error_message == None:
            response.success = True
            response.message = "Loaded waypoints from floor_level " + str(self.floor_level)
            rospy.loginfo("Loaded waypoints from {0}".format(self.floor_level))
        else:
            response.success = False
            self.floor_level = prev_floor_level
            response.message = str(error_message)
            rospy.loginfo("Failed to load waypoints from floor_level: {0}".format(request.floor_level))
        return response

if __name__ == "__main__":
    rospy.init_node("waypoint_server")
    rospy.loginfo("(Waypoint_Server) initializing...")
    server = WaypointServer()
    rospy.loginfo("[Waypoint_Server] ready.")
    rospy.spin()
