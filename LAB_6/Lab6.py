'''
 IMPORTANT: Read through the code before beginning implementation!
 Your solution should fill in the various "TODO" items within this starter code.
'''
import copy
import math
import random
import argparse
from PIL import Image
from pprint import pprint
import numpy as np
import math
import sys
import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16

# FOR VISUALS
if sys.version_info[0] == 2:
  import Tkinter as tk # Python 2
else:
  import tkinter as tk # Python 3 [Use "sudo apt-get install python3-tk" to get tkinter]

# Create GUI
r = tk.Tk() 
r.title('World Map') 
canvas_width = 1200
canvas_height = 800
canvas = tk.Canvas(r,width=canvas_width,height=canvas_height)
canvas.pack()

def _create_circle(self, x, y, r, **kwargs):
    return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
tk.Canvas.create_circle = _create_circle
###### PART 4 GLOBALS




# GLOBALS 
pose2d_sparki_odometry = Pose2D(0,0,0) #Pose2D message object, contains x,y,theta members in meters and radians

# Track servo angle in radians
servo_deg = 80
servo_rad = math.radians(servo_deg)

# Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
ir_sensor_read = [0 for i in range(5)]

# Create data structure to hold map representation
# Map is 180 cm wide by 120 cm height thus an array of 60 wide and 40 height would give us a nice square map cell size of 0.03 m
map_cell_size = 0.03    # [m]
width_map = 60          
height_map = 40
max_map_dist = math.sqrt(width_map**2 + height_map**2)
map_rep = [0 for x in range(height_map * width_map)]
cost_map_rep = [0 for x in range(height_map * width_map)]

# Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
subscriber_odometry = None
subscriber_state = None
publisher_render = None

# CONSTANTS 
IR_THRESHOLD = 300 # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME = 0.5 # In seconds



########################

########################## PART 5 GLOBALS ############################
g_CYCLE_TIME = .100

# Parameters you might need to use which will be set automatically
MAP_SIZE_X = None
MAP_SIZE_Y = None

# Default parameters will create a 4x4 grid to test with
g_MAP_SIZE_X = 2. # 2m wide
g_MAP_SIZE_Y = 1.5 # 1.5m tall
g_MAP_RESOLUTION_X = 0.5 # Each col represents 50cm
g_MAP_RESOLUTION_Y = 0.375 # Each row represents 37.5cm
g_NUM_X_CELLS = int(g_MAP_SIZE_X // g_MAP_RESOLUTION_X) # Number of columns in the grid map
g_NUM_Y_CELLS = int(g_MAP_SIZE_Y // g_MAP_RESOLUTION_Y) # Number of rows in the grid map

# Map from Lab 4: values of 0 indicate free space, 1 indicates occupied space
g_WORLD_MAP = [0] * g_NUM_Y_CELLS*g_NUM_X_CELLS # Initialize graph (grid) as array

# Source and Destination (I,J) grid coordinates
g_dest_coordinates = (3,3)
g_src_coordinates = (0,0)

#How Many Cells are in map
g_Num_Cells = 0

######################################################################



###########PROBALY NOT NEEEDED ########################
def create_test_map(map_array):
  # Takes an array representing a map of the world, copies it, and adds simulated obstacles
  num_cells = len(map_array)
  new_map = copy.copy(map_array)
  # Add obstacles to up to sqrt(n) vertices of the map
  for i in range(int(math.sqrt(len(map_array)))):
    random_cell = random.randint(0, num_cells)
    new_map[random_cell] = 1
  return new_map


###################################################




def _load_img_to_intensity_matrix(img_filename):
  '''
  Helper function to read the world image containing obstacles
  YOu should not modify this
  '''
  global MAP_SIZE_X, MAP_SIZE_Y

  if img_filename is None:
      grid = np.zeros([800,1200])
      return grid

  img = Image.open(img_filename)

  MAP_SIZE_X = img.width
  MAP_SIZE_Y = img.height
  #print(MAP_SIZE_X, MAP_SIZE_Y)

  grid = np.zeros([img.height, img.width])
  for y in range(img.height):
      for x in range(img.width):
          pixel = img.getpixel((x,y))
          grid[y,x] = 255 - pixel[0] # Dark pixels have high values to indicate being occupied/having something interesting
          
  return grid


def vertex_index_to_ij(vertex_index):
  '''
  vertex_index: unique ID of graph vertex to be convered into grid coordinates
  Returns COL, ROW coordinates in 2D grid
  '''
  global g_NUM_X_CELLS
  i = vertex_index % g_NUM_X_CELLS
  j = vertex_index // g_NUM_Y_CELLS
  return i, j

def ij_to_vertex_index(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns integer 'vertex index'
  '''
  global g_NUM_X_CELLS
  return j*g_NUM_X_CELLS + i


def ij_coordinates_to_xy_coordinates(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return (i+0.5)*g_MAP_RESOLUTION_X, (j+0.5)*g_MAP_RESOLUTION_Y

def xy_coordinates_to_ij_coordinates(x,y):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return int(x // g_MAP_RESOLUTION_X), int(y // g_MAP_RESOLUTION_Y)

# **********************************
# *      Core Dijkstra Functions   *
# **********************************

def get_travel_cost(vertex_source, vertex_dest):
  # Returns the cost of moving from vertex_source (int) to vertex_dest (int)
  # INSTRUCTIONS:
  '''
      This function should return 1 if:
        vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)

      This function should return 1000 if:
        vertex_source corresponds to (i,j) coordinates outside the map
        vertex_dest corresponds to (i,j) coordinates outside the map
        vertex_source and vertex_dest are not adjacent to each other (i.e., more than 1 move away from each other)
  '''
  global g_NUM_Y_CELLS, g_NUM_X_CELLS, g_WORLD_MAP,g_NUM_X_CELLS  # Number of columns in the grid map 

  (i_source,j_source) = vertex_index_to_ij(vertex_source)
  (i_dest,j_dest) = vertex_index_to_ij(vertex_dest)

  #vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)
  if g_WORLD_MAP[vertex_dest]==1 or g_WORLD_MAP[vertex_source]==1:  
      return 1000
  if vertex_source == vertex_dest:
      return 0
  if vertex_source < len(g_WORLD_MAP) and vertex_dest < len(g_WORLD_MAP):
    if(vertex_dest == vertex_source + g_NUM_X_CELLS ):
      return 1
    if(vertex_dest == vertex_source - g_NUM_X_CELLS ):
      return 1
    if(vertex_dest == vertex_source + 1 ):
      return 1
    if(vertex_dest == vertex_source - 1 ):
      return 1
  return 100


def run_dijkstra(source_vertex):
    global g_NUM_X_CELLS, g_NUM_Y_CELLS, g_Num_Cells
    g_Num_Cells = g_NUM_X_CELLS * g_NUM_Y_CELLS
    # Array mapping vertex_index to distance of shortest path from vertex_index to source_vertex.
    dist = [99] * g_NUM_X_CELLS * g_NUM_Y_CELLS

    # Queue for identifying which vertices are up to still be explored:
    # Will contain tuples of (vertex_index, cost), sorted such that the min cost is first to be extracted (explore cheapest/most promising vertices first)
    Q_cost = {}
    for i in range(g_Num_Cells):
         Q_cost[i] = 2000
  
    # Array of ints for storing the next step (vertex_index) on the shortest path back to source_vertex for each vertex in the graph
    prev = [-1] * g_NUM_X_CELLS*g_NUM_Y_CELLS
    #set the source
    dist[source_vertex] = 0
    Q_cost[source_vertex] = 0
    # Insert your Dijkstra's code here. Don't forget to initialize Q_cost properly!
    while (Q_cost):
    
        currentVertexIndex = min(Q_cost, key=Q_cost.get)
        Around = []
        North = currentVertexIndex - g_NUM_X_CELLS
        #print("QCost:", Q_cost)
        Q_cost.pop(currentVertexIndex, None)

        ## Getting the Neighbors
        if(North >= 0 and North <= g_Num_Cells-1):
            Around.append(North)
        East = currentVertexIndex +1
        if(East >= 0 and East <= g_Num_Cells-1):
            Around.append(East)

        South = currentVertexIndex + g_NUM_X_CELLS
        if(South >= 0 and South <= g_Num_Cells-1):
            Around.append(South)

        West = currentVertexIndex - 1 
        if(West >= 0 and West <= g_Num_Cells-1):
            Around.append(West)

        for i in Around:
            alt1 = get_travel_cost(currentVertexIndex,i)
            alt2 = dist[currentVertexIndex]
            alt = alt1 + alt2
            if alt < dist[i]:
                dist[i] = alt
                prev[i] = currentVertexIndex
                Q_cost[i] = get_travel_cost(currentVertexIndex,i)
    return prev



def reconstruct_path(prev, source_vertex, dest_vertex):
  '''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
  '''

  # TODO: Insert your code herex

  final_path = []
	
  vertex = dest_vertex
  final_path.append(dest_vertex)
  while vertex != source_vertex:
    if prev[int(vertex)] == -1:
      print("no path available")
      return []
    final_path.insert(0,prev[int(vertex)])
    vertex = prev[int(vertex)]

  
  return final_path


def render_map(map_array): 
    global g_NUM_X_CELLS, g_NUM_Y_CELLS, g_WORLD_MAP
    i = 0
    y_count = 1
    string = ""
    while(i < len(map_array)):
      if(map_array[i] == 1):
            string = string+"|"
      else:
          string = string + "."
      if i == (g_NUM_X_CELLS*y_count)-1:
        print (string,"\n")
        y_count +=1
        string = ""
      i += 1 
    pass

def render_map2(map_array, path): 
    global g_NUM_X_CELLS, g_NUM_Y_CELLS, g_WORLD_MAP
    i = 0
    y_count = 1
    string = ""
    while(i < len(map_array)):
      if(map_array[i] == 1):
        string = string+"|"
      else:
        if i in path:
          string = string + 'X'
        else:
          string = string + "."
      if i == (g_NUM_X_CELLS*y_count)-1:
        print (string,"\n")
        y_count +=1
        string = ""
      i += 1 
    pass

def part_2(args):
  global g_dest_coordinates,g_MAP_SIZE_X,g_MAP_SIZE_Y,g_MAP_RESOLUTION_X,g_MAP_RESOLUTION_Y,g_NUM_X_CELLS,g_NUM_Y_CELLS
  global g_src_coordinates,MAP_SIZE_X,MAP_SIZE_Y
  global g_WORLD_MAP



  g_MAP_SIZE_X = 1.2 # 2m wide
  g_MAP_SIZE_Y = 1.8 # 1.5m tall
  g_MAP_RESOLUTION_X = 0.01 # Each col represents 50cm
  g_MAP_RESOLUTION_Y = 0.01 # Each row represents 37.5cm
  g_NUM_X_CELLS = int(g_MAP_SIZE_X // g_MAP_RESOLUTION_X) # Number of columns in the grid map
  g_NUM_Y_CELLS = int(g_MAP_SIZE_Y // g_MAP_RESOLUTION_Y) # Number of rows in the grid map
  g_NUM_X_CELLS = g_NUM_X_CELLS +1
  MAP_SIZE_X = 120
  MAP_SIZE_Y = 180




  g_src_coordinates = (float(args.src_coordinates[0]), float(args.src_coordinates[1]))
  g_dest_coordinates = (float(args.dest_coordinates[0]), float(args.dest_coordinates[1]))
  g_MAP_RESOLUTION_X
  # pixel_grid has intensity values for all the pixels
  # You will have to convert it to the earlier 0 and 1 matrix yourself
  
  pixel_grid = _load_img_to_intensity_matrix(args.obstacles)
        
  '''
  TODO -
  1) Compute the g_WORLD_MAP -- depending on the resolution, you need to decide if your cell is an obstacle cell or a free cell.
  2) Run Dijkstra's to get the plan
  3) Show your plan/path on the image
  Feel free to add more helper functions
  '''

  #### Your code goes here ####

  g_WORLD_MAP = []
  for i in range(0,len(pixel_grid)-1,4):
    for j in range(0,len(pixel_grid[i]),10):
      if pixel_grid[i][j] != 0:
        g_WORLD_MAP.append(1)
      else:
         g_WORLD_MAP.append(0)

  cord = ij_to_vertex_index(int(g_src_coordinates[0]*100),int(g_src_coordinates[1]*100))
  prev = run_dijkstra(cord)
  #print(prev)
  path = reconstruct_path(prev, ij_to_vertex_index(int(g_src_coordinates[0]*100),int(g_src_coordinates[1]*100)), ij_to_vertex_index(int(g_dest_coordinates[0]*100),int(g_dest_coordinates[1]*100)))
  #print(path)
  render_map2(g_WORLD_MAP, path)
  for i in range(0, len(path)): 
    path[i] = str(path[i]) 
  
  print("path: "," -> ".join(path))



def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry,map_rep, servo_rad
    # Set up your publishers and subscribers
    # Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    rospy.init_node('buffemup')
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
    publisher_ping = rospy.Publisher('sparki/ping_command', Empty, queue_size=10)
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)
    publisher_render = rospy.Publisher('/sparki/render_sim',Empty, queue_size=10)

    subscriber_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber('/sparki/state', String, callback_update_state)
    rospy.sleep(1)
    # Set sparki's servo to an angle pointing inward to the map (e.g., 45)
    publisher_servo.publish(servo_deg)
    publisher_render.publish(Empty())
    #print("Did Init")
    #print(map_rep)

def Main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, ir_sensor_read
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry
    # Init your node to register it with the ROS core
    init()
    while not rospy.is_shutdown():
        # Implement CYCLE TIME
        begin = time.time()

        publisher_ping.publish(Empty())
        # Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        msg = Float32MultiArray()
        msg.data = [1.0, 1.0]
        # Implement loop closure here
        # add the msg.data to the where the sparki moves
        #print(ir_sensor_read)
        if(ir_sensor_read[1] < IR_THRESHOLD):
            msg.data[0] = 0
            #print(ir_sensor_read[3],"stef")
            pass
        elif(ir_sensor_read[3] < IR_THRESHOLD):
            #print(ir_sensor_read[1],"3")
            msg.data[1] = 0
            
        #print("Odom", pose2d_sparki_odometry)
        #add the publish msg to the motor and to the ping
         
        publisher_motor.publish(msg)
        publisher_ping.publish(Empty())
        publisher_render.publish(Empty())
        display_map()                    # Update the GUI 

        if False:
            rospy.loginfo("Loop Closure Triggered")
            print("5")
        if((time.time() - begin) < 50):
            rospy.sleep(50 - time.time() - begin)

        # Implement CYCLE TIME
        rospy.sleep(0)


if __name__ == "__main__":
    main()


