'''
 IMPORTANT: Read through the code before beginning implementation!
 Your solution should fill in the various "TODO" items within this starter code.
'''
#====================================================================#
# NECESSARY IMPORTS FOR CALCULATIONS
#====================================================================#
import sys
import copy
import math
import math
import json
import copy
import time
import rospy
import random
import argparse
import numpy as np
from PIL import Image
from pprint import pprint
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
# PART 3 | GLOBAL VARIABLES
#====================================================================#
# SPARKI STATIC VARIABLES
SPARKI_SERVO_LEFT    = 80
SPARKI_SERVO_CENTER  = 0
SPARKI_SERVO_RIGHT   = -80

SPARKI_SPEED         = 0.0278   # 100% speed in m/s
SPARKI_AXLE_DIAMETER = 0.085    # Distance between wheels, meters 
SPARKI_WHEEL_RADIUS  = 0.03     # Radius of wheels, meters

# VARIABLES FOR FEEDBACK CONTROL
DIR_CCW = -1
DIR_CW  = 1

gain1 = [0.01,0.7,0.0] # Gains used before the distance threshold is reached, focus on distance and bearing error, and not heading at all
gain2 = [0.3,0.1,0.0]
gain3 = [0.1,0.01,0.3] # Gains used after reaching the distance threshold, we'll neglect distance and bearing error, and only focus on heading error

left_speed_pct  = 0
right_speed_pct = 0

x_r_dot     = 0
theta_r_dot = 0

DISTANCE_THRESHOLD   = 0.03     # Distance threshold om where we use gain2
d_achieve_err = DISTANCE_THRESHOLD
b_achive_err  = 0.05
h_achieve_err = 0.05

#====================================================================#
# PART 4 | GLOBAL VARIABLES
#====================================================================#
pose2d_sparki_odometry = Pose2D(0,0,0)    # Pose2D message object, contains x,y,theta members in meters and radians
pose2d_sparki_goal = Pose2D(0,0,0)

servo_deg = 0                             # Desired servo angle in degrees
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
CYCLE_TIME   = 0.05   # In seconds [50ms]

#====================================================================#
# PART 5 | GLOBAL VARIABLES
#====================================================================#
g_CYCLE_TIME = .100
path         = []

# Parameters you might need to use which will be set automatically
MAP_SIZE_X = None
MAP_SIZE_Y = None

g_MAP_SIZE_X = 180                          # 180 cm wide map
g_MAP_SIZE_Y = 120                          # 120 cm tall map

use_cell_div = 2
if(use_cell_div == 1):
  g_NUM_X_CELLS      = 60                           # We want 60 collumns
  g_NUM_Y_CELLS      = 40                           # and 40 rows, to have an even resolution 0f 3 cm for both x and 
elif(use_cell_div == 2):
  g_NUM_X_CELLS      = 15                           # We want 15 collumns
  g_NUM_Y_CELLS      = 10                           # and 10 rows, to have an even resolution 0f 3 cm for both x and y

g_MAP_RESOLUTION_X = g_MAP_SIZE_X/g_NUM_X_CELLS   # Calculate the X resolution based on the number of collumns
g_MAP_RESOLUTION_Y = g_MAP_SIZE_Y/g_NUM_Y_CELLS   # Calculate the Y resolution based on the number of rows

# Map from Lab 4: values of 0 indicate free space, 1 indicates occupied space
g_NUM_CELLS = g_NUM_X_CELLS * g_NUM_Y_CELLS       # Caculate the number of cells in the map
g_WORLD_MAP = [0] * g_NUM_CELLS                   # Initialize graph (grid) as array with all 0 to indicate free space

# Source and Destination (X,Y) grid coordinates
# g_src_xy_coordinates  = (1.70,0.50)   # Destination grid coordinate (X,Y) in m
# g_dest_xy_coordinates = (1.00,1.10)  # Source grid coordinate (X,Y) in m

# g_src_xy_coordinates  = (1.2,0.2)       # Destination grid coordinate (X,Y) in m
# g_dest_xy_coordinates = (0.225,0.975)   # Source grid coordinate (X,Y) in m

# g_src_xy_coordinates  = (0.9,0.30)  # Destination grid coordinate (X,Y) in m
# g_dest_xy_coordinates = (0.9,0.75)  # Source grid coordinate (X,Y) in m

# g_src_xy_coordinates  = (1.20,0.20)   # Destination grid coordinate (X,Y) in m
# g_dest_xy_coordinates = (0.225,0.975) # Source grid coordinate (X,Y) in m

g_src_xy_coordinates  = (0.225,0.60)  # Destination grid coordinate (X,Y) in m
g_dest_xy_coordinates = (1.35,0.3)  # Source grid coordinate (X,Y) in m

goals = []  # Will store the goals for the robot to go through

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
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y, g_NUM_Y_CELLS

  x = (i+0.5)*g_MAP_RESOLUTION_X                 
  y = ((g_NUM_Y_CELLS-j)+0.5)*g_MAP_RESOLUTION_Y # Subtract the row number from the total number of rows since y = 0 is at the bottom left corner
  return x,y

#====================================================================#
# Helper function to calculate real world coordinates given a grid
# coordinate.num_goals = len(goals)
    # goals_pos = 0

    # pose2d_sparki_odometry.x     = goals[goals_pos][0]/100
    # pose2d_sparki_odometry.y     = goals[goals_pos][1]/100
    # pose2d_sparki_odometry.theta = 0 

    # for goals_pos in range(len(goals)-1):
    #   pose2d_sparki_odometry.x     = goals[goals_pos][0]/100
    #   pose2d_sparki_odometry.y     = goals[goals_pos][1]/100
    #   pose2d_sparki_odometry.theta = 0 

    #   publisher_odom.publish(pose2d_sparki_odometry)
    #   publisher_render.publish(Empty())

    #   time.sleep(2)

    # time.sleep(10)
num_goals = len(goals)
    # goals_pos = 0

    # pose2d_sparki_odometry.x     = goals[goals_pos][0]/100
    # pose2d_sparki_odometry.y     = goals[goals_pos][1]/100
    # pose2d_sparki_odometry.theta = 0 

    # for goals_pos in range(len(goals)-1):
    #   pose2d_sparki_odometry.x     = goals[goals_pos][0]/100
    #   pose2d_sparki_odometry.y     = goals[goals_pos][1]/100
    #   pose2d_sparki_odometry.theta = 0 

    #   publisher_odom.publish(pose2d_sparki_odometry)
    #   publisher_render.publish(Empty())

    #   time.sleep(2)

    # time.sleep(10)

#====================================================================#
def vertex_to_xy_coordinates(vertex):
  [i,j] = vertex_index_to_ij(vertex)
  [x,y] = ij_coordinates_to_xy_coordinates(i,j)
  return x,y

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
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y, g_NUM_Y_CELLS, g_MAP_SIZE_Y
  return int(x // g_MAP_RESOLUTION_X), (int((g_MAP_SIZE_Y-y) // g_MAP_RESOLUTION_Y))

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

  (i_source,j_source) = vertex_index_to_ij(vertex_source)
  if((i_source<0 or i_source>g_NUM_X_CELLS) or (j_source<0 or j_source>g_NUM_Y_CELLS)):
    return 1000

  (i_dest,j_dest) = vertex_index_to_ij(vertex_dest)
  if((i_dest<0 or i_dest>g_NUM_X_CELLS) or (j_dest<0 or j_dest>g_NUM_Y_CELLS)):
    return 1000

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

        
        South = currentVertexIndex + g_NUM_X_CELLS                    # Calculate the vertex of the cell SOUTH of the current vertex
        if(South >= 0 and South <= g_NUM_CELLS-1):                    # if it's value exist is greater than 0 and less than the Number of vertices that exist in our map
            Around.append(South)                                      # Append it's vertices number in the around array

        # For the east and west, we also have to make sure that their vertices are on the same row as the current vertex if not, we'll loop through the map which doesn't make sense
        East = currentVertexIndex + 1                                                                         # Calculate the vertex number of the cell EAST of the current vertex
        if(East >= 0 and East <= g_NUM_CELLS-1 and (East//g_NUM_X_CELLS==currentVertexIndex//g_NUM_X_CELLS)): # if it's value exist is greater than 0 and less than the Number of vertices that exist in our map
            Around.append(East)                                                                               # Append it's vertices number in the around array

        West = currentVertexIndex - 1                                                                         # Calculate the vertex of the cell WEST of the current vertex
        if(West >= 0 and West <= g_NUM_CELLS-1 and (West//g_NUM_X_CELLS==currentVertexIndex//g_NUM_X_CELLS)): # if it's value exist is greater than 0 and less than the Number of vertices that exist in our map
            Around.append(West)                                                                               # Append it's vertices number in the around array

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
  
  #print("src_v = ", source_vertex)
  #print("dest_v = ",dest_vertex)

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
def display_map(path):
  # Display the map
  global MAP_SIZE_X, MAP_SIZE_Y
  global g_NUM_X_CELLS, g_NUM_Y_CELLS, g_WORLD_MAP, r
  global canvas_height
  canvas.delete("all")        # Clears canvas

  start_pixel_coord_x = (MAP_SIZE_X//g_NUM_X_CELLS)/2    # First circle x coord
  start_pixel_coord_y = (MAP_SIZE_Y//g_NUM_Y_CELLS)/2    # First circle y coord
  circle_rad          = min([start_pixel_coord_x,start_pixel_coord_y])    # Circle radius

  circle_offset       = circle_rad*2    # Pixel offset for each circle

  for j in range (g_NUM_Y_CELLS):
      for i in range(g_NUM_X_CELLS):
          c_ind = ij_to_vertex_index(i,j)
          c_val = g_WORLD_MAP[c_ind]

          # print("c_ind = ",c_ind)
          # print("C_val = ",c_val)

          circ_x = (start_pixel_coord_x + circle_offset*i)
          circ_y = (start_pixel_coord_y + circle_offset*j)

          if(c_ind in path):
            #print("FOUND c_ind in path")
              # If this index is in the path, then fill the circle blue
            if path.index(c_ind) == 0:
              canvas.create_circle(circ_x, circ_y, circle_rad, fill="green", outline="#DDD", width=2)
            elif path.index(c_ind) == (len(path)-1):
              canvas.create_circle(circ_x, circ_y, circle_rad, fill="purple", outline="#DDD", width=2)
            else:
              canvas.create_circle(circ_x, circ_y, circle_rad, fill="red", outline="#DDD", width=2)
          elif(c_val == 0):
            # if value in cell is 0, draw a white filled circle at that point which signifies open point
            canvas.create_circle(circ_x, circ_y, circle_rad, fill="white", outline="#DDD", width=2)
          else:
            # Otherwise, it is an obstacle thus fill it with black
            canvas.create_circle(circ_x, circ_y, circle_rad, fill="black", outline="#DDD", width=2)
  
  r.update_idletasks()        # Clear all events in GUI (Not really necessary for us but recommended)
  r.update()                  # Update GUI
  return

#====================================================================#
# GET GOALS FUNCTION
#====================================================================#
def GetGoals(path):
    Goals = []
    for i in range(len(path)):
      coord = vertex_to_xy_coordinates(path[i])
      Goals.append(coord)
    return Goals
            
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
# FEEDBACK CONTROL HELPER FUNCTIONS
#====================================================================#

#=====================================================#
# USED TO BOUND ANGLES BETWEEN -PI and PI
#=====================================================#
def boundTheta(theta):
  M_PI = math.pi

  if theta > M_PI:
    theta = theta - 2*M_PI
  if theta <= -M_PI:
    theta = theta + 2*M_PI

  return theta

#=====================================================#
# CALCULATE ERRORS
#=====================================================#
def calculateErrors(pose2d_sparki_goal):
  global pose2d_sparki_odometry
  global d_achieve_err,b_achive_err,h_achieve_err

  x_r     = pose2d_sparki_odometry.x
  y_r     = pose2d_sparki_odometry.y
  theta_r = pose2d_sparki_odometry.theta

  x_g     = pose2d_sparki_goal.x
  y_g     = pose2d_sparki_goal.y
  theta_g = pose2d_sparki_goal.theta

  distance_err = math.sqrt((x_r-x_g)**2 + (y_r-y_g)**2)
  bearing_err  = theta_r - math.atan2((y_g-y_r),(x_g-x_r))
  heading_err  = theta_g - theta_r

  bearing_err = boundTheta(bearing_err)
  heading_err = boundTheta(heading_err)

  goal_achieved = 0
  if((distance_err<d_achieve_err) and (bearing_err<b_achive_err) and (heading_err < h_achieve_err)):
    goal_achieved = 1

  return distance_err,bearing_err,heading_err,goal_achieved

#=====================================================#
# CALCULATE WHEEL SPEEDS
#=====================================================#
def calculateWheelSpeeds(err):
  global gain1, gain2, gain3, DISTANCE_THRESHOLD
  global b_achive_err, SPARKI_AXLE_DIAMETER, SPARKI_WHEEL_RADIUS
  global SPARKI_SPEED

  d_gain1 = gain1[0]
  b_gain1 = gain1[1]
  h_gain1 = gain1[2]

  d_gain2 = gain2[0]
  b_gain2 = gain2[1]
  h_gain2 = gain2[2]

  d_gain3 = gain3[0]
  b_gain3 = gain3[1]
  h_gain3 = gain3[2]

  # Extract out the errors passed out to the function
  d_err = err[0]
  b_err = err[1]
  h_err = err[2]

  b_err = boundTheta(b_err)
  h_err = boundTheta(h_err)

  # We need to have multiple gain settings because
  # Initially, we want to put emphasis on fixing bearing
  if((d_err > DISTANCE_THRESHOLD) and (b_err > b_achive_err)):
    x_dot     = d_gain1 * d_err
    theta_dot = -(b_gain1 * b_err) + -(h_gain1 * h_err)

  # Then fixing the distance
  elif d_err > DISTANCE_THRESHOLD:               # If the distance error is greater than that of the set threshold
    x_dot     = d_gain2 * d_err
    theta_dot = -(b_gain2 * b_err) + -(h_gain2 * h_err)

  # Then finally the heading
  else:
    x_dot     = d_gain3 * d_err
    theta_dot = -(b_gain3 * b_err) + -(h_gain3 * h_err)

  phi_l = (((2 * x_dot) - (theta_dot* SPARKI_AXLE_DIAMETER))/(2 * SPARKI_WHEEL_RADIUS))
  phi_r = (((2 * x_dot) + (theta_dot* SPARKI_AXLE_DIAMETER))/(2 * SPARKI_WHEEL_RADIUS))
   
  # Since the sparki wheels can only go max of SPARKI_SPEED m/s, if the speed calculated for the wheels surpasses
  # what sparki can achieve, let's scale it down such that the highest speed is set to the max speed sparki can achive
  # and the other is scaled accordingly 
  if (max(math.fabs(phi_l),math.fabs(phi_r))*SPARKI_WHEEL_RADIUS) > SPARKI_SPEED: # Get the max speed and compare if it's greater than the sparki max speed
    # if the left wheel is the greatest then we'll set the speed of the left wheel to 100% of the SPARKI_SPEED, and scale the right accordingly
    if math.fabs(phi_l) > math.fabs(phi_r):   
      left_speed_pct  = 1
      right_speed_pct = math.fabs(phi_r/phi_l)

    # if the right wheel is the greatest then we'll set the speed of the right wheel to 100% of the SPARKI_SPEED, and scale the left accordingly
    elif math.fabs(phi_r) > math.fabs(phi_l):
      left_speed_pct  = math.fabs(phi_l/phi_r)
      right_speed_pct = 1

    # if they're equal, set them to both 100% of the SPARKI_SPEED
    else:
      left_speed_pct  = 1
      right_speed_pct = 1

    # We also need to make sure to keep it's original sign, thus lets's give the negative value if phi_l or phi_r is less than 0
    phi_l = -left_speed_pct*SPARKI_SPEED  if (phi_l<0) else left_speed_pct*SPARKI_SPEED
    phi_r = -right_speed_pct*SPARKI_SPEED if (phi_r<0) else right_speed_pct*SPARKI_SPEED

    x_dot = (SPARKI_WHEEL_RADIUS/2)*(phi_l+phi_r)
    theta_dot = (SPARKI_WHEEL_RADIUS/SPARKI_AXLE_DIAMETER)*(phi_r-phi_l)

  return phi_l, phi_r, x_dot, theta_dot

#=====================================================#
# UPDATE ODOMETRY
#=====================================================#
def update_odometry(x_r_dot, theta_r_dot):
  global pose2d_sparki_odometry, CYCLE_TIME
  theta = pose2d_sparki_odometry.theta

  # y_r_dot will always be zero since the robot can't move sideways, 
  # thus will not be accounted in the calculation.
  x_I_dot     = math.cos(theta)*x_r_dot
  y_I_dot     = math.sin(theta)*x_r_dot
  theta_I_dot = theta_r_dot

  delta_x_I     = x_I_dot     * CYCLE_TIME
  delta_y_I     = y_I_dot     * CYCLE_TIME
  delta_theta_I = theta_I_dot * CYCLE_TIME

  pose2d_sparki_odometry.x     = pose2d_sparki_odometry.x     + delta_x_I
  pose2d_sparki_odometry.y     = pose2d_sparki_odometry.y     + delta_y_I
  pose2d_sparki_odometry.theta = pose2d_sparki_odometry.theta + delta_theta_I

  pose2d_sparki_odometry.theta = boundTheta(pose2d_sparki_odometry.theta)

  publisher_odom.publish(pose2d_sparki_odometry)


#====================================================================#
# INITIALIZING FUNCTION
#====================================================================#
def init(args):
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry, servo_rad, g_WORLD_MAP, path, goals
    global g_src_xy_coordinates, g_dest_xy_coordinates
    global MAP_SIZE_X, MAP_SIZE_Y, g_NUM_X_CELLS, g_NUM_Y_CELLS

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

    #print(pixel_grid)
    pix_group_x = int(MAP_SIZE_X//g_NUM_X_CELLS)
    pix_group_y = int(MAP_SIZE_Y//g_NUM_Y_CELLS)
    #time.sleep(1000)
    #print(MAP_SIZE_X)
    #print(MAP_SIZE_Y)

    #print(pix_group_x)
    #print(pix_group_y)

    for j in range(0,MAP_SIZE_Y,pix_group_y):
      for i in range(0,MAP_SIZE_X,pix_group_x):
        found_obstacle = 0
        #print(i,j)
        for ii in range(pix_group_x):
          for jj in range(pix_group_y):
            if pixel_grid[j+jj][i+ii] == 255:
              g_WORLD_MAP.append(1)
              found_obstacle = 1
              break
          if found_obstacle:
            break
        if not found_obstacle: 
          g_WORLD_MAP.append(0) 

    g_src_xy_coordinates = (float(args.src_coordinates[0]), float(args.src_coordinates[1]))
    g_dest_xy_coordinates = (float(args.dest_coordinates[0]), float(args.dest_coordinates[1]))

    g_src_coordinates  = xy_coordinates_to_ij_coordinates(int(g_src_xy_coordinates[0]*100),int(g_src_xy_coordinates[1]*100))
    g_dest_coordinates = xy_coordinates_to_ij_coordinates(int(g_dest_xy_coordinates[0]*100),int(g_dest_xy_coordinates[1]*100))

    #print("src_ij  = ",g_src_coordinates)
    #print("dest_ij = ",g_dest_coordinates)

    source      = ij_to_vertex_index( g_src_coordinates[0],  g_src_coordinates[1])
    destination = ij_to_vertex_index( g_dest_coordinates[0], g_dest_coordinates[1])

    prev = run_dijkstra(source)
    path = reconstruct_path(prev, source, destination)
    goals = GetGoals(path)
    print(goals)
    #print(path)
    display_map(path)
    
#====================================================================#
# MAIN FUNCTION
#====================================================================#
def main(args):
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, ir_sensor_read
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry, pose2d_sparki_goal, goals, x_r_dot, theta_r_dot

    # Init your node to register it with the ROS core
    # Arguments are the source and destination vertices and the obstacles image file name
    init(args)
    
    ##########################################
    # num_goals = len(goals)
    # goals_pos = 0

    # pose2d_sparki_odometry.x     = goals[goals_pos][0]/100
    # pose2d_sparki_odometry.y     = goals[goals_pos][1]/100
    # pose2d_sparki_odometry.theta = 0 

    # for goals_pos in range(len(goals)-1):
    #   pose2d_sparki_odometry.x     = goals[goals_pos][0]/100
    #   pose2d_sparki_odometry.y     = goals[goals_pos][1]/100
    #   pose2d_sparki_odometry.theta = 0 

    #   publisher_odom.publish(pose2d_sparki_odometry)
    #   publisher_render.publish(Empty())

    #   time.sleep(2)

    # time.sleep(10)

    #########################################
    num_goals = len(goals)
    if(num_goals == 0):
      print("NO PATH FOUND, EXITING CODE")
      exit()

    goals_pos = 0

    pose2d_sparki_odometry.x     = goals[goals_pos][0]/100
    pose2d_sparki_odometry.y     = goals[goals_pos][1]/100
    pose2d_sparki_odometry.theta = 0 

    goal_achieved = 1

    while not rospy.is_shutdown():
        # Implement CYCLE TIME
        begin = time.time()

        # publisher_ping.publish(Empty())
        # # Implement line following code here
        # #      To create a message for changing motor speed, use Float32MultiArray()
        # #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        # msg = Float32MultiArray()
        # msg.data = [1.0, 1.0]
        # # Implement loop closure here
        # # add the msg.data to the where the sparki moves
        # #print(ir_sensor_read)
        # if(ir_sensor_read[1] < IR_THRESHOLD):
        #     msg.data[0] = 0
        #     #print(ir_sensor_read[3],"stef")
        #     pass
        # elif(ir_sensor_read[3] < IR_THRESHOLD):
        #     #print(ir_sensor_read[1],"3")
        #     msg.data[1] = 0
            
        #print("Odom", pose2d_sparki_odometry)
        #add the publish msg to the motor and to the ping
         
        # publisher_motor.publish(msg)
        # publisher_ping.publish(Empty())
        # publisher_render.publish(Empty())
        r.update()                        # Update GUI

        #============================================#
        # LAB 3 CODE GOES HERE, 
        #   variable goals has all the coordinates we 
        #   need to go through
        #============================================#

        if goals_pos < (num_goals-2):
          #print("GOOD")
          update_odometry(x_r_dot,theta_r_dot)      # Calculate and Update the odometry, also publish to see what it would look like on the simulator

          #====================================#
          # PUT CODE TO UPDATE GOALS HERE
          if goal_achieved:

            x_i = goals[goals_pos][0]/100
            y_i = goals[goals_pos][1]/100

            x_ip1 = goals[goals_pos + 1][0]/100
            y_ip1 = goals[goals_pos + 1][1]/100

            # dx = x_ip1 - x_i
            # dy = y_ip1 - y_i
            # pose_theta_goal = pose2d_sparki_odometry.theta + math.atan2(dy,dx)

            pose_theta_goal = math.pi

            #pose_theta_goal = math.atan2(dy,dx)

            pose2d_sparki_goal.x     = x_ip1
            pose2d_sparki_goal.y     = y_ip1
            pose2d_sparki_goal.theta = pose_theta_goal
            goals_pos = goals_pos + 1

            goal_achieved = 0
          #====================================#

          err = calculateErrors(pose2d_sparki_goal) # Calculate the distance, bearing, and heading errors
          goal_achieved = err[3]                    # 0 = distance err, 1 = bearing error, 2 = heading error, 3 = achived goal?
          
          #print("Distance err = ",err[0],"\tBearing Err = ",err[1],"\tHeading Err = ",err[2])

          # After calculating our new erros, if we've reached our goal we don't have to move until goal is updated
          # Thus, let's set the x_r_dot and theta_r_dot = 0
          if goal_achieved:
            #phi_l      = 0
            #phi_r      = 0
            x_r_dot     = 0
            theta_r_dot = 0

            # Let's print to see if the sparki odemtery is actually close to the goal
            print("CURR: X = ", pose2d_sparki_odometry.x," Y = ",pose2d_sparki_odometry.y," Theta = ",pose2d_sparki_odometry.theta)
            print("GOAL: X = ", pose2d_sparki_goal.x," Y = ",pose2d_sparki_goal.y, " Theta = ",pose2d_sparki_goal.theta,"\n")

          # However, if goal hasn't been achieved yet, calculate new speeds
          else:
            ret = calculateWheelSpeeds(err)           # Calculate the robots wheel speed but also the robots velocities 
                                                      # We're not calculating y_r_dot because it is always zero for robots that can't move sidways
            #phi_l      = ret[0]
            #phi_r      = ret[1]
            x_r_dot     = ret[2]
            theta_r_dot = ret[3]

          publisher_render.publish(Empty())           # Render robot

          # Implement CYCLE_TIME
          if((time.time() - begin) < 50):
              rospy.sleep(50 - time.time() - begin)

          # end = time.time()
          # time_run = end-begin

          # if(time_run<CYCLE_TIME):
          #   rospy.sleep(CYCLE_TIME-time_run)

        elif goals_pos == num_goals-2:
          update_odometry(x_r_dot,theta_r_dot)      # Calculate and Update the odometry, also publish to see what it would look like on the simulator
          publisher_render.publish(Empty())         # Render robot
          goals_pos = goals_pos + 1

        #============================================#
        #============================================#
        #============================================#


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-s','--src_coordinates', nargs=2, default=[0.225, 0.6], help='Starting x, y location in world coords')
  parser.add_argument('-g','--dest_coordinates', nargs=2, default=[1.35, 0.3], help='Goal x, y location in world coords')
  parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles_test1.png', help='Black and white image showing the obstacle locations')
  args = parser.parse_args()

  main(args)


