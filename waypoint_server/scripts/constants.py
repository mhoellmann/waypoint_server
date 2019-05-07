from std_msgs.msg import ColorRGBA
# define menu entry item ids (top menu item must start from 1)
MENU_CONNECT_EDGE = 1
MENU_CONNECT_EDGE_DOOR = 2
MENU_CONNECT_EDGE_ELEVATOR = 3
MENU_DISCONNECT_EDGE = 4
MENU_CLEAR = 5
MENU_RENAME = 6
MENU_REMOVE = 7
MENU_ROOM_LEFT = 9
MENU_ROOM_RIGHT = 10

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

# define orientation of lines for intersection
COLINEAR = 0
CLOCKWISE = 1
COUNTERCLOCKWISE = 2

