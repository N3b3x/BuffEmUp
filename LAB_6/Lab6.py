'''
 IMPORTANT: Read through the code before beginning implementation!
 Your solution should fill in the various "TODO" items within this starter code.
'''
#====================================================================#
# NECESSARY IMPORTS FOR CALCULATIONS
#====================================================================#
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

#====================================================================#
# NECESSARY IMPORTS FOR VISUALIZATIONS
#====================================================================#
if sys.version_info[0] == 2:
  import Tkinter as tk # Python 2
else:
  import tkinter as tk # Python 3 [Use "sudo apt-get install python3-tk" to get tkinter]

#====================================================================#
# CREATE GUI
#====================================================================#
r = tk.Tk() 
r.title('World Map') 
canvas_width = 1200
canvas_height = 800
canvas = tk.Canvas(r,width=canvas_width,height=canvas_height)
canvas.pack()
# Helper functions for creating circles in GUI
def _create_circle(self, x, y, r, **kwargs):
    return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
tk.Canvas.create_circle = _create_circle

#====================================================================#
# PART 4 | GLOBAL VARIABLES
#====================================================================#
pose2d_sparki_odometry = Pose2D(0,0,0)    # Pose2D message object, contains x,y,theta members in meters and radians

servo_deg = 80                            # Desired servo angle in degrees
servo_rad = math.radians(servo_deg)       # Desired servo angle in radians which is what will be used for simulation

ir_sensor_read = [0 for i in range(5)]    # Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)

# Use these variables to hold your publishers
publisher_motor     = None
publisher_odom      = None
publisher_ping      = None
publisher_servo     = None

# Use these variables to hold your subscribers
subscriber_odometry = None
subscriber_state    = None
publisher_render    = None

#====================================================================#
# PART 4 | GLOBAL CONSTANT VARIABLES
#====================================================================#
IR_THRESHOLD = 300    # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME   = 0.5    # In seconds

#====================================================================#
# PART 5 | GLOBAL VARIABLES
#====================================================================#
g_CYCLE_TIME = .100
path         = []

# Parameters you might need to use which will be set automatically
MAP_SIZE_X = None
MAP_SIZE_Y = None

g_MAP_SIZE_X       = 1.8                          # 2 m wide map
g_MAP_SIZE_Y       = 1.2                          # 1.5 m tall map
g_NUM_X_CELLS      = 60                           # We want 60 collumns
g_NUM_Y_CELLS      = 40                           # and 40 rows
g_MAP_RESOLUTION_X = g_MAP_SIZE_X/g_NUM_X_CELLS   # Calculate the X resolution based on the number of collumns
g_MAP_RESOLUTION_Y = g_MAP_SIZE_Y/g_NUM_Y_CELLS   # Calculate the Y resolution based on the number of rows

# Map from Lab 4: values of 0 indicate free space, 1 indicates occupied space
g_NUM_CELLS = g_NUM_X_CELLS * g_NUM_Y_CELLS       # Caculate the number of cells in the map
g_WORLD_MAP = [0] * g_NUM_CELLS                   # Initialize graph (grid) as array with all 0 to indicate free space

# Source and Destination (I,J) grid coordinates
g_dest_coordinates = (3,3)  # Source grid coordinate (I,J)
g_src_coordinates = (0,0)   # Destination grid coordinate (I,J)

#====================================================================#
# Helper function to read the world image containing obstacles
#====================================================================#
def _load_img_to_intensity_matrix(img_filename):
  '''
  Helper function to read the world image containing obstacles
  YOu should not modify this
  '''
  global MAP_SIZE_X, MAP_SIZE_Y             # Bring the global variables we are gonna be using

  
  if img_filename is None:                  # If file name is not provided, 
      grid = np.zeros([800,1200])           # create a 0 grid of 800 rows and 1200 pixels
      return grid                           # Then return that grid

  img = Image.open(img_filename)            # if image file name was provided, open that file

  MAP_SIZE_X = img.width                    # Get the width pixel dimension of the image
  MAP_SIZE_Y = img.height                   # Get the height pixel dimension of the image
  #print(MAP_SIZE_X, MAP_SIZE_Y)            # Print the size of the image
  
  grid = np.zeros([img.height, img.width])  # Create a 0 grid of image height(rows) and width(collumns)
  for y in range(img.height):               # Go through each pixel in the image
      for x in range(img.width):
          pixel = img.getpixel((x,y))       # Grab the pixel value
          grid[y,x] = 255 - pixel[0]        # The black and white pixels in the image have a value of 0 and 255 respectively thus let's
                                            # substract their value from 255 to let dark be 255 and white be 0 such that 
                                            # Dark pixels have high values to indicate being occupied/having something interesting.
          
  return grid                               # Return that grid

#====================================================================#
# Helper function to calculate the grid coordinates given a vertex 
# number in the array
#====================================================================#
def vertex_index_to_ij(vertex_index):
  '''
  vertex_index: unique ID of graph vertex to be convered into grid coordinates
  Returns COL, ROW coordinates in 2D grid
  '''
  global g_NUM_X_CELLS
  i = vertex_index % g_NUM_X_CELLS      # To find the collumn number, just take the vertex index and divide by the number of collumns, and keep the remainder 
  j = vertex_index // g_NUM_X_CELLS     # To find the row number, just take the vertex index and divide by the number of collumns, then floor the number
  return i, j

#====================================================================#
# Helper function to calculate the vertex number given a grid
# coordinate
#====================================================================#
def ij_to_vertex_index(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns integer 'vertex index'
  '''
  global g_NUM_X_CELLS
  return j*g_NUM_X_CELLS + i

#====================================================================#
# Helper function to calculate real world coordinates given a grid
# coordinate.
#====================================================================#
def ij_coordinates_to_xy_coordinates(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return (i+0.5)*g_MAP_RESOLUTION_X, (j+0.5)*g_MAP_RESOLUTION_Y

#====================================================================#
# Helper function to calculate grid coordinate given real world 
# coordinates
#====================================================================#
def xy_coordinates_to_ij_coordinates(x,y):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return int(x // g_MAP_RESOLUTION_X), int(y // g_MAP_RESOLUTION_Y)

#====================================================================#
# CORE DJIKSTRA FUNCTIONS
#====================================================================#

#===============================================#
# CALCULATE TRAVEL COST
#================================================#
def get_travel_cost(vertex_source, vertex_dest):
  # Returns the cost of moving from vertex_source (int) to vertex_dest (int)
  # INSTRUCTIONS:
  '''
      This function should return 1 if:
        vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)

      This function should return 1000 if:
        vertex_source corresponds to (i,j) coordinates outside the map  [CHECK]
        vertex_dest corresponds to (i,j) coordinates outside the map    [CHECK]
        vertex_source and vertex_dest are not adjacent to each other (i.e., more than 1 move away from each other) [CHECK]
  '''
  global g_NUM_Y_CELLS, g_NUM_X_CELLS, g_WORLD_MAP,g_NUM_X_CELLS  # Number of columns in the grid map 

  # (i_source,j_source) = vertex_index_to_ij(vertex_source)
  # (i_dest,j_dest) = vertex_index_to_ij(vertex_dest)

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
  return 1000

#====================================================================#
# CORE DJIKSTRA FUNCTIONS
#====================================================================#
def run_dijkstra(source_vertex):
    global g_NUM_X_CELLS, g_NUM_Y_CELLS, g_NUM_CELLS
    
    # Array mapping vertex_index to distance of shortest path from vertex_index to source_vertex.
    # Create dist array all initialized with 99
    dist = [99] * g_NUM_X_CELLS * g_NUM_Y_CELLS

    # Queue for identifying which vertices are up to still be explored:
    # Will contain tuples of (vertex_index, cost), sorted such that the min cost is first to be extracted (explore cheapest/most promising vertices first)
    # Initalize all cost at 2000
    Q_cost = {}
    for i in range(g_NUM_CELLS):
         Q_cost[i] = 2000
  
    # Array of ints for storing the next step (vertex_index) on the shortest path back to source_vertex for each vertex in the graph
    prev = [-1] * g_NUM_X_CELLS*g_NUM_Y_CELLS

    # Set the source
    dist[source_vertex]   = 0                                         # Set the distance{Total Cost} taken to travel to the source to 0
    Q_cost[source_vertex] = 0                                         # Set the source cost to be 0 so that it's the minimum value in there, which will be
                                                                      # useful when we pop the first minimum cost vertex

    # Insert your Dijkstra's code here. Don't forget to initialize Q_cost properly!
    while (Q_cost):
    
        currentVertexIndex = min(Q_cost, key=Q_cost.get)              # Get the vertex we will explore by popping the minimum cost vertex, which should be the source vertex initially
        #print("QCost:", Q_cost)
        Q_cost.pop(currentVertexIndex, None)                          # Pop the current vertex from the cost queue

        # Get the Neighbors of the current Vertex, NESW
        Around = []                                                   # Array to hold the vertex number of the NESW neighbors
        North = currentVertexIndex - g_NUM_X_CELLS                    # Calculate the vertex number of the cell NORTH of the current vertex
        if(North >= 0 and North <= g_NUM_CELLS-1):                    # if it's value exist is greater than 0 and less than the Number of vertices that exist in our map
            Around.append(North)                                      # Append it's vertices number in the around array

        East = currentVertexIndex + 1                                 # Calculate the vertex number of the cell EAST of the current vertex
        if(East >= 0 and East <= g_NUM_CELLS-1):                      # if it's value exist is greater than 0 and less than the Number of vertices that exist in our map
            Around.append(East)                                       # Append it's vertices number in the around array

        South = currentVertexIndex + g_NUM_X_CELLS                    # Calculate the vertex of the cell SOUTH of the current vertex
        if(South >= 0 and South <= g_NUM_CELLS-1):                    # if it's value exist is greater than 0 and less than the Number of vertices that exist in our map
            Around.append(South)                                      # Append it's vertices number in the around array

        West = currentVertexIndex - 1                                 # Calculate the vertex of the cell WEST of the current vertex
        if(West >= 0 and West <= g_NUM_CELLS-1):                      # if it's value exist is greater than 0 and less than the Number of vertices that exist in our map
            Around.append(West)                                       # Append it's vertices number in the around array

        # Now that we have the neighboring vertices of the current vertex we're exploring
        # Let's go through each and calculate their cost
        for i in Around:
            alt1 = get_travel_cost(currentVertexIndex,i)              # Calculate the travel cost to the neighboring vertex
            alt2 = dist[currentVertexIndex]                           # Get the total dist cost it took to get to the current vertex we're exploring
            alt = alt1 + alt2                                         # Add the two costs
            if alt < dist[i]:                                         # if the total cost is less than the cost already at the vertex 
                dist[i] = alt                                         # Update to it's new cost
                prev[i] = currentVertexIndex                          # Also update which vertex the new travel cost was calulated from (which would be the current vertex we're exploring)
                Q_cost[i] = get_travel_cost(currentVertexIndex,i)     # Set the current cost to the calculated travel cost
    return prev

#====================================================================#
# Used to reconstruct the path taken from source to dest
#====================================================================#
def reconstruct_path(prev, source_vertex, dest_vertex):
  '''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
  '''

  # TODO: Insert your code here

  final_path = []                             # Create the final path array
	
  vertex = dest_vertex                        # The first vertex we'll append is the destination vertex
  final_path.append(dest_vertex)              # Append the destination vertex

  while vertex != source_vertex:              # while the vertex found does not equal to the source vertex
    if prev[int(vertex)] == -1:               # If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the path from dest to source,
      print("no path available")              # Print no path available, then
      return []                               # return an empty list

    final_path.insert(0,prev[int(vertex)])    # Otherwise, keep pre-pending the previous vertex of the current vertex to the final path array 
    vertex = prev[int(vertex)]                # Then set the new vertex equal to the previous vertex

  return final_path                           # Return the final path finally which, if a path was found, should return each vertice we took to get to the destination vertex from the source vertex

#====================================================================#
# MAP RENDERING FUNCTIONS
#====================================================================#
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

#====================================================================#
# GET GOALS FUNCTION
#====================================================================#
def GetGoals(path):
    Goals = []
    while(path):
        current = []
        current.append(path[0])
        south = False
        north = False
        east  = False
        west  = False

        if(path[1] == path[0] - g_NUM_X_CELLS ):
            north = True
        if(path[1] == path[0] + g_NUM_X_CELLS ):
            south = True
        if(path[1] == path[0] - 1 ):
            east = True
        if(path[1] == path[0] + 1 ):
            west = True
     
        path.pop(0)

        if (north == True and path[1] == path[0]-g_NUM_X_CELLS):
            path.pop(0)
        else:
            current.append(path[0])
            Goals.append(current)
            current = []

        if (south == True and path[1] == path[0]+g_NUM_X_CELLS):
            path.pop(0)
        else:
            current.append(path[0])
            Goals.append(current)
            current = []

        if (east == True and path[1] == path[0]+1):
          path.pop(0)
        else:
          current.append(path[0])
          Goals.append(current)
          current = []

        if (south == True and path[1] == path[0]+g_NUM_X_CELLS):
            path.pop(0)
        else:
            current.append(path[0])
            Goals.append(current)
            current = []
    return Goals
            
#====================================================================#
# INITIALIZING FUNCTION
#====================================================================#
def init(args):
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry, servo_rad,g_WORLD_MAP,path

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

    #==========================================#
    # LAB 5 CODE TO EXTRACT COLLISION FREE PATH
    #==========================================#
    pixel_grid = _load_img_to_intensity_matrix(args.obstacles)
    g_WORLD_MAP = []
    for i in range(0,len(pixel_grid)-1,4):
        for j in range(0,len(pixel_grid[i]),10):
            if pixel_grid[i][j] != 0:
                g_WORLD_MAP.append(1)
            else:
                g_WORLD_MAP.append(0)

    cord = ij_to_vertex_index(int(g_src_coordinates[0]*100),int(g_src_coordinates[1]*100))
    prev = run_dijkstra(cord)
    path = reconstruct_path(prev, ij_to_vertex_index(int(g_src_coordinates[0]*100),int(g_src_coordinates[1]*100)), ij_to_vertex_index(int(g_dest_coordinates[0]*100),int(g_dest_coordinates[1]*100)))
    
#====================================================================#
# CALLBACK & CALLBACK HELPER FUNCTIONS
#====================================================================#
def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    # Copy this data into your local odometry variable
    pose2d_sparki_odometry = data

def callback_update_state(data):
   # print("got new data",data)
    global ir_sensor_read, servo_rad
    global pose2d_sparki_odometry
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #servo_rad  = math.radians(state_dict['servo'])    
    ir_sensor_read = state_dict["light_sensors"]

def convert_ultrasonic_to_robot_coords(x_us):
    # Using US sensor reading and servo angle, return value in robot-centric coordinates [x_us is in meters]
    x_r, y_r = 0., 0.
    global servo_rad

    #print("Servo rad = ",servo_rad)
    x_r = x_us * math.cos(servo_rad)
    y_r = x_us * math.sin(servo_rad)

    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    # Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0., 0.
    global pose2d_sparki_odometry
    
    p_x = (pose2d_sparki_odometry.x)
    p_t = (pose2d_sparki_odometry.theta)
    p_y = (pose2d_sparki_odometry.y)

    cos_t = math.cos(p_t)
    sin_t = math.sin(p_t)

    x_w = cos_t*x_r - sin_t*y_r + p_x   
    y_w = sin_t*x_r + cos_t*y_r + p_y

    return x_w, y_w

#====================================================================#
# MAIN FUNCTION
#====================================================================#
def main(args):
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, ir_sensor_read
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry

    # Init your node to register it with the ROS core
    # Arguments are the source and destination vertices and the obstacles image file name
    init(args)

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
        #display_map()                    # Update the GUI 

        if False:
            rospy.loginfo("Loop Closure Triggered")
            print("5")
        if((time.time() - begin) < 50):
            rospy.sleep(50 - time.time() - begin)

        # Implement CYCLE TIME
        rospy.sleep(0)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-s','--src_coordinates', nargs=2, default=[1.2, 0.2], help='Starting x, y location in world coords')
  parser.add_argument('-g','--dest_coordinates', nargs=2, default=[0.3, 0.7], help='Goal x, y location in world coords')
  parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles_test1.png', help='Black and white image showing the obstacle locations')
  args = parser.parse_args()
  main(args)


