#!/usr/bin/python2
import time #todo remove
from collections import deque  # performance; faster than prepending regular list
import rospy
from nav_msgs.msg import OccupancyGrid, Path, GridCells
from geometry_msgs.msg import Point, PoseStamped

DIRECTION_UP = 0
DIRECTION_RIGHT = 1
DIRECTION_DOWN = 2
DIRECTION_LEFT = 3

class ShortestPath:

    occ_grid = OccupancyGrid()  # store map data in row (x) first order
    nodes = []  # all nodes in search
    index_nodes_map = {}
    frontier_nodes = deque()
    index_start = 0
    index_goal = 0

    def __init__(self):
        rospy.loginfo("ShortestPath Init . . .")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.path_pub = rospy.Publisher("/ratchet_path", Path, queue_size=1)
        self.cells_pub = rospy.Publisher("/ratchet_cells", GridCells, queue_size=10000)
        self.cells = deque()

    def index_from_xy(self, x, y):
        # TODO ACCOUNT FOR MAP ORIGIN
        res = self.occ_grid.info.resolution
        width = self.occ_grid.info.width
        return int(round(y / res)) * width + int(round(x / res))

    def xy_from_index(self, index):
        # TODO ACCOUNT FOR MAP ORIGIN
        res = self.occ_grid.info.resolution
        width = self.occ_grid.info.width
        y_ = index / width
        y = y_ * res
        x = (index - y_ * width) * res
        return x, y

    def point_from_node(self, node):
        point = Point()
        point.x, point.y = self.xy_from_index(node.index)
        point.z = 0.1
        return point

    def index_from_direction(self, index, direction):
        if direction == DIRECTION_UP:
            addition = self.occ_grid.info.width
        elif direction == DIRECTION_RIGHT:
            addition = 1
        elif direction == DIRECTION_DOWN:
            addition = -1
        elif direction == DIRECTION_LEFT:
            addition = -self.occ_grid.info.width

        return index + addition

    def check_neighbours(self, node):
        for i in range(4):  # loop through up, right, down, left neighbours
            neighbour_index = self.index_from_direction(node.index, i)
            neighbour = self.get_node_from_index(neighbour_index)
            if neighbour is None:  # neighbour not in list
                if self.check_valid(neighbour_index):
                    neighbour = Node()
                    neighbour.index = neighbour_index
                    neighbour.parent_index = node.index
                    neighbour.depth = node.depth + 1
                    self.frontier_nodes.appendleft(neighbour)
                    self.cells.appendleft(self.point_from_node(neighbour))
                    self.nodes.append(neighbour)
                    self.index_nodes_map[neighbour_index] = neighbour
                    if neighbour_index == self.index_goal:
                        return True  # goal found
            else:  # neighbour exists
                if neighbour.visited:
                    continue
                if neighbour.depth > node.depth + 1:
                    neighbour.depth = node.depth + 1
                    neighbour.parent_index = node.index

            if neighbour is not None:
                cells = GridCells()
                cells.header.frame_id = "map"
                cells.cell_height = 0.1
                cells.cell_width = 0.1
                cells.cells = self.cells
                self.cells_pub.publish(cells)
                if node.parent_index == neighbour.index:
                    rospy.logwarn("SEOMTHING AINT RIGHT")
                    time.sleep(20)



        node.visited = True
        return False  # goal not found

    def check_valid(self, index):
        if index >= 0 and index <= len(self.occ_grid.data):  # if within map
            if self.occ_grid.data[index] == 0:  # non-obstacle
                return True
        else:
            return False

    def get_node_from_index(self, index):
        try:
            return self.index_nodes_map[index]
        except KeyError:
            return None

    def shortest_path(self, start, goal):
        start_t = time.time()
        self.index_start = self.index_from_xy(start.x, start.y)
        self.index_goal = self.index_from_xy(goal.x, goal.y)

        start_node = Node()
        start_node.index = self.index_start
        self.nodes.append(start_node)
        self.frontier_nodes.append(start_node)
        self.index_nodes_map[start_node.index] = start_node
        while not self.check_neighbours(self.frontier_nodes.pop()):
            pass
            self.cells.pop()
            rospy.loginfo("len frontier nodes {0}".format(len(self.frontier_nodes)))

        shortest_path = []
        node = self.get_node_from_index(self.index_goal)
        while node.parent_index is not None:
            shortest_path.append(node)
            node = self.get_node_from_index(node.parent_index)

        path = Path()
        path.header.frame_id = "map"
        while len(shortest_path) > 0:
            node = shortest_path.pop()
            point = self.point_from_node(node)
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = point
            path.poses.append(pose_stamped)
        print(time.time() - start_t)
        return path


    def map_cb(self, map_data):
        self.occ_grid = map_data
        rospy.loginfo("Acquired map.")
        start = Point()
        start.x = 143.112
        start.y = 17.639
        goal = Point()
        goal.x = 148.4
        goal.y = 23.51
        self.path_pub.publish(self.shortest_path(start, goal))
        rospy.loginfo("nodes traversed: {}".format(len(self.nodes)))

class Node:
    index = 0
    parent_index = None
    visited = False
    depth = 0

if __name__ == '__main__':
    rospy.init_node('shortest_path_node', anonymous=False)
    sp = ShortestPath()
    rospy.spin()

