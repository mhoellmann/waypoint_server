#!/usr/bin/python2
import time #todo remove
import math
from collections import deque  # performance; faster than prepending regular list
from heapq import heappush, heappop  # priority queue for faster algorithm
import rospy
from nav_msgs.msg import OccupancyGrid, Path, GridCells
from geometry_msgs.msg import Point, PoseStamped

DIRECTION_UP = 0
DIRECTION_UP_RIGHT = 1
DIRECTION_RIGHT = 2
DIRECTION_DOWN_RIGHT = 3
DIRECTION_DOWN = 4
DIRECTION_DOWN_LEFT = 5
DIRECTION_LEFT = 6
DIRECTION_UP_LEFT = 7

SQRT_TWO = 1.414213562373095

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

class ShortestPath:

    occ_grid = OccupancyGrid()  # store map data in row (x) first order
    nodes = []  # all nodes in search
    index_nodes_map = {}
    frontier_nodes = deque()
    frontier_nodes_pl = []
    index_start = 0
    index_goal = 0

    def __init__(self):
        rospy.loginfo("Initialising shortest path")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.path_pub = rospy.Publisher("/ratchet_path", Path, queue_size=1)
        self.cells_pub = rospy.Publisher("/ratchet_cells", GridCells, queue_size=10000)
        self.cells = deque()  # visualization purposes

    def index_from_xy(self, x, y):
        # TODO: ACCOUNT FOR MAP ORIGIN
        # x = x + self.occ_grid.info.origin.x
        # y = y + self.occ_grid.info.origin.y
        res = self.occ_grid.info.resolution
        width = self.occ_grid.info.width
        return int(round(y / res)) * width + int(round(x / res))

    def xy_from_index(self, index):
        # TODO: ACCOUNT FOR MAP ORIGIN
        res = self.occ_grid.info.resolution
        width = self.occ_grid.info.width
        y_ = index / width
        y = y_ * res
        x = (index - y_ * width) * res
        # x = x - self.occ_grid.info.origin.x
        # y = y - self.occ_grid.info.origin.y
        return x, y

    def point_from_node(self, node):
        point = Point()
        point.x, point.y = self.xy_from_index(node.index)
        point.z = 0.1
        return point

    def index_from_direction(self, index, direction):
        if direction == DIRECTION_UP:
            addition = self.occ_grid.info.width
        elif direction == DIRECTION_UP_RIGHT:
            addition = self.occ_grid.info.width + 1
        elif direction == DIRECTION_RIGHT:
            addition = 1
        elif direction == DIRECTION_DOWN_RIGHT:
            addition = -self.occ_grid.info.width + 1
        elif direction == DIRECTION_DOWN:
            addition = -self.occ_grid.info.width
        elif direction == DIRECTION_DOWN_LEFT:
            addition = -self.occ_grid.info.width - 1
        elif direction == DIRECTION_LEFT:
            addition = -1
        elif direction == DIRECTION_UP_LEFT:
            addition = self.occ_grid.info.width - 1

        return index + addition

    def check_neighbours(self, node):
        """Check all neighbours of a node.
        Add any new neighbours to list of nodes and frontier nodes.
        Return true if neighbour's index is the target index.
        """
        for i in range(8):  # loop through all neighbours
            neighbour_index = self.index_from_direction(node.index, i)
            neighbour = self.get_node_from_index(neighbour_index)
            # set extra_depth bassed on direction
            if (i == DIRECTION_UP or i == DIRECTION_RIGHT or
                i == DIRECTION_DOWN or i == DIRECTION_LEFT):
                extra_depth = 1
            else:  # diagonal direction
                extra_depth = SQRT_TWO
            if neighbour is None:  # neighbour not in list yet
                if self.check_valid(neighbour_index):
                    neighbour = Node(neighbour_index,
                                     parent_index=node.index,
                                     depth=node.depth+extra_depth)
                    self.frontier_nodes.appendleft(neighbour)
                    dist = euclidean_distance(*(self.xy_from_index(neighbour_index)+
                                              self.xy_from_index(self.index_goal)))
                    item = (dist, neighbour)
                    heappush(self.frontier_nodes_pl, item)
                    #self.cells.appendleft(self.point_from_node(neighbour))
                    self.nodes.append(neighbour)
                    self.index_nodes_map[neighbour_index] = neighbour
                    if neighbour_index == self.index_goal:
                        return True  # goal found

            else:  # neighbour exists in list
                if neighbour.visited:
                    continue  # already dealt with this neighbour, so skip to next loop
                if neighbour.depth > node.depth + extra_depth:
                    neighbour.depth = node.depth + extra_depth
                    neighbour.parent_index = node.index

            #if neighbour is not None:
            #    cells = GridCells()
            #    cells.header.frame_id = "map"
            #    cells.cell_height = 0.1
            #    cells.cell_width = 0.1
            #    cells.cells = self.cells
            #    self.cells_pub.publish(cells)

        node.visited = True  # node 'visited' after checking all of its neighbours
        return False  # goal not found

    def check_valid(self, index):
        if index >= 0 and index <= len(self.occ_grid.data):  # if within map
            if self.occ_grid.data[index] == 0:  # if non-obstacle
                return True
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

        start_node = Node(self.index_start)
        self.nodes.append(start_node)
        self.frontier_nodes.append(start_node)
        self.index_nodes_map[start_node.index] = start_node
        cn_start_t = time.time()
        dist = euclidean_distance(*(self.xy_from_index(self.index_start)+
                                  self.xy_from_index(self.index_goal)))
        item = (dist, start_node)
        heappush(self.frontier_nodes_pl, item)
        while not self.check_neighbours(heappop(self.frontier_nodes_pl)[1]):
            #self.cells.pop()  # visualization purposes
            pass
            #rospy.loginfo("length of frontier nodes {0}".format(len(self.frontier_nodes)))
        print "below - while not self.check_neighbours(self.frontier_nodes.pop()):"
        print time.time() - cn_start_t

        shortest_path = []
        node = self.get_node_from_index(self.index_goal)
        print "node.parnet index below"
        print node.parent_index
        while node.parent_index is not None:
            shortest_path.append(node)
            node = self.get_node_from_index(node.parent_index)
            
        path = Path()
        path.header.frame_id = "map"
        while shortest_path:  # items in shortest_path > 0
            print "shortest path!!!!!!!!!!!!!!!!!"
            node = shortest_path.pop()
            point = self.point_from_node(node)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position = point
            path.poses.append(pose_stamped)
        print "shortest_path function time below"
        print time.time() - start_t
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

    def __init__(self, index, parent_index=None, visited=False, depth=0):
        self.index = index
        self.parent_index = parent_index
        self.visited = visited
        self.depth = depth

if __name__ == '__main__':
    rospy.init_node('shortest_path_node', anonymous=False)
    SP = ShortestPath()
    rospy.spin()
