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
import numpy


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

##this is a list to check the spaces around the vertext (N,S,E,W)
aroundCurrent=[]
g_Num_Cells = 0

def create_test_map(map_array):
  # Takes an array representing a map of the world, copies it, and adds simulated obstacles
  num_cells = len(map_array)
  new_map = copy.copy(map_array)
  # Add obstacles to up to sqrt(n) vertices of the map
  for i in range(int(math.sqrt(len(map_array)))):
    random_cell = random.randint(0, num_cells)
    new_map[random_cell] = 1

  return new_map


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
  return vertex_index % g_NUM_X_CELLS, vertex_index // g_NUM_X_CELLS

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
  global g_NUM_Y_CELLS, g_NUM_X_CELLS, g_WORLD_MAP  # Number of columns in the grid map 

  (x_source,y_source) = vertex_index_to_ij(vertex_source)
  (x_dest,y_dest) = vertex_index_to_ij(vertex_dest)
  
  #vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)
  #print (g_WORLD_MAP)
  if g_WORLD_MAP[vertex_dest]==1 or g_WORLD_MAP[vertex_source]==1:
      #print(1000)
      return 1000
  if vertex_source == vertex_dest:
      # print(0)
      return 0
  if vertex_source < len(g_WORLD_MAP) and vertex_dest < len(g_WORLD_MAP):
    start_i, start_j = vertex_index_to_ij(vertex_source)
    dest_i, dest_j = vertex_index_to_ij(vertex_dest)
    manDist = abs(start_i - dest_i) + abs(start_j - dest_j)
    if manDist == 1 and g_WORLD_MAP[vertex_source] != 1 and g_WORLD_MAP[vertex_dest] != 1:
      # print(1)
      return 1

    return 100

def aroundCurrentVertex(currentX, currentY):
  global aroundCurrent
  #Appends the index values to the list
  aroundCurrent = []
  aroundCurrent.append(ij_to_vertex_index(currentX+1, currentY)) #North
  aroundCurrent.append(ij_to_vertex_index(currentX, currentY-1)) #South
  aroundCurrent.append(ij_to_vertex_index(currentX, currentY+1)) #East
  aroundCurrent.append(ij_to_vertex_index(currentX-1, currentY)) #West
  


def run_dijkstra(source_vertex):
    global g_NUM_X_CELLS, g_NUM_Y_CELLS, g_Num_Cells
    global aroundCurrent
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
        #currentVertex = min(Q_cost,key = lambda t: abs(t[1])) #might need to possibly change to first value of a sorted Q_cost
        currentVertexIndex = min(Q_cost, key=Q_cost.get)
        Around = []
        North = currentVertexIndex - 4
        Q_cost.pop(currentVertexIndex, None)
        if(North >= 0 and North <= g_Num_Cells-1):
            Around.append(North)
        East = currentVertexIndex +1
        if(East >= 0 and East <= g_Num_Cells-1):
            Around.append(East)

        South = currentVertexIndex +4
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

  # TODO: Insert your code here
  #set a temp holder for source_vertex
  # tempVertex = dest_vertex

  # #append the temp to our final path array
  # final_path.append(tempVertex)

  # vertex = True

  # #make sure the tempVertex is not the source vertex
  # if tempVertex != source_vertex:
  #       trueVertex = vertex
  #       #while its true
  #       while trueVertex:
  #             #if not == -1
  #             if prev[tempVertex] != -1:
  #                   #insert 0 in our final path array at the given index
  #                   final_path.insert(0, prev[tempVertex])
  #                   #set the temp vertex to the prev[vertex]
  #                   tempVertex = prev[tempVertex]
  #             else: # if prev == -1 return empty list
  #                 return []



  final_path = []
	
  vertex = dest_vertex
  final_path.append(dest_vertex)
  while vertex != source_vertex:
    if prev[vertex] == -1:
      print("no path available")
      return []
    final_path.insert(0,prev[vertex])
    vertex = prev[vertex]

  
  return final_path


def render_map(map_array): 
    global g_NUM_X_CELLS, g_NUM_Y_CELLS, g_WORLD_MAP
  
#   out = [["." for x in range(g_NUM_X_CELLS)] for y in range(g_NUM_Y_CELLS)]
#   for j in range(g_NUM_Y_CELLS):
#         for i in range(g_NUM_X_CELLS):
#             if (g_WORLD_MAP[ij_to_vertex_index(i,j)]== 0):
#                 out[i][j]=" . "
#             elif (g_WORLD_MAP[ij_to_vertex_index(i,j)]== 1):
#                 out[i][j]="[ ]"
#   out = numpy.transpose(out)
#   print('\n'.join([''.join(['{:2}'.format(item) for item in (rowOut)]) for rowOut in reversed(out)]))
#   print("\n")
    i = 0
    ## First Line
    string  = ""
    while (i < 4):
        if(map_array[i] == 1):
            string = string+" [] "
        else:
            string = string + " . "
        i = i +1 
    print (string,"\n")
    ## Line 2
    string  = ""
    while (i < 8):
        
        if(map_array[i] == 1):
            string = string+" [] "
        else:
            string = string + " . "
        i = i +1 
    print (string,"\n")
    ## line 3 
    string  = ""
    while (i < 12): 
        
        if(map_array[i] == 1):
            string = string+" [] "
        else:
            string = string + " . "
        i = i +1 
    print (string,"\n")
   ##line 4
    string  = ""
    while (i < 16):
        
        if(map_array[i] == 1):
            string = string+" [] "
        else:
            string = string + " . "
        i = i +1 
    print (string,"\n")
    pass


def part_1():
  global g_WORLD_MAP

  # TODO: Initialize a grid map to use for your test -- you may use create_test_map for this, or manually set one up with obstacles
  test_map = create_test_map(g_WORLD_MAP)
  g_WORLD_MAP = test_map

  # Use render_map to render your initialized obstacle map
  print("starting vertex: " ,g_src_coordinates)
  print("destination vertex: " ,g_dest_coordinates)

  render_map(test_map)
  # TODO: Find a path from the (I,J) coordinate pair in g_src_coordinates to the one in g_dest_coordinates using run_dijkstra and reconstruct_path
  prev = run_dijkstra(ij_to_vertex_index(g_src_coordinates[0],g_src_coordinates[1]))
  print(prev)
  path = reconstruct_path(prev, ij_to_vertex_index(g_src_coordinates[0],g_src_coordinates[1]), ij_to_vertex_index(g_dest_coordinates[0],g_dest_coordinates[1]))
  print(path)
  '''
  TODO-
    Display the final path in the following format:
    Source: (0,0)
    Goal: (3,1)
    0 -> 1 -> 2 -> 6 -> 7
  '''
  for i in range(0, len(path)): 
    path[i] = str(path[i]) 
  
  print("path: "," -> ".join(path))

def part_2(args):
  global g_dest_coordinates
  global g_src_coordinates
  global g_WORLD_MAP

  g_src_coordinates = (args.src_coordinates[0], args.src_coordinates[1])
  g_dest_coordinates = (args.dest_coordinates[0], args.dest_coordinates[1])

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




if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-s','--src_coordinates', nargs=2, default=[1.2, 0.2], help='Starting x, y location in world coords')
  parser.add_argument('-g','--dest_coordinates', nargs=2, default=[0.3, 0.7], help='Goal x, y location in world coords')
  parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles_test1.png', help='Black and white image showing the obstacle locations')
  args = parser.parse_args()


  part_1()
  # part_2(args)
