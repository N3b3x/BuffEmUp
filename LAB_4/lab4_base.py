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

def main():
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

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    # Copy this data into your local odometry variable
    pose2d_sparki_odometry = data
    #print(pose2d_sparki_odometry)
    #print(type(pose2d_sparki_odometry))

def callback_update_state(data):
   # print("got new data",data)
    global ir_sensor_read, servo_rad
    global pose2d_sparki_odometry
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #servo_rad  = math.radians(state_dict['servo'])    
    ir_sensor_read = state_dict["light_sensors"]

    if 'ping' in state_dict.keys():
        distance = state_dict['ping']
        if distance > 0:
            #print("distance",distance)
            #print("robot",pose2d_sparki_odometry.x,pose2d_sparki_odometry.y,pose2d_sparki_odometry.theta)
            x_r,y_r = convert_ultrasonic_to_robot_coords(distance)
            #print("US coord",x_r,y_r)
            x_w,y_w = convert_robot_coords_to_world(x_r,y_r)
            #print("World",x_w,y_w)
            populate_map_from_ping(x_w,y_w)
            #display_map()                           # Update the GUI 
            pass


def convert_ultrasonic_to_robot_coords(x_us):
    # Using US sensor reading and servo angle, return value in robot-centric coordinates [x_us is in meters]
    x_r, y_r = 0., 0.
    global servo_rad

    #print("Servo rad = ",servo_rad)
    x_r = x_us * math.cos(servo_rad)
    y_r = x_us * math.sin(servo_rad)

    #convert_robot_coords_to_world(x_r,y_r)
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

    #populate_map_from_ping(x_w,y_w)
    return x_w, y_w

# Given world coordinates of an object detected via ping, fill in the corresponding part of the map
def populate_map_from_ping(x_ping, y_ping):
    global map_rep, map_cell_size

    # Get i and j coordinate for map representation
    i = (int)(x_ping//map_cell_size)
    j = (int)(y_ping//map_cell_size)

    # Get cell index
    c_index = ij_to_cell_index(i,j)

    if(c_index<len(map_rep)):
        map_rep[c_index] = 1


def display_map():
    # Display the map
    global width_map, height_map, map_rep, r
    global canvas_height
    canvas.delete("all")        # Clears canvas

    start_pixel_coord_x = 10    # First circle x coord
    start_pixel_coord_y = 10    # First circle y coord
    circle_rad          = 10    # Circle radius

    circle_offset       = 20    # Pixel offset for each circle

    for j in range (height_map):
        for i in range(width_map):
            c_ind = ij_to_cell_index(i,j)
            c_val = map_rep[c_ind]

            circ_x = (start_pixel_coord_x + circle_offset*i)
            circ_y = canvas_height-(start_pixel_coord_y + circle_offset*j)

            if(c_val == 0):
                # if value in cell is 0, draw a white filled circle at that point
                canvas.create_circle(circ_x, circ_y, circle_rad, fill="white", outline="#DDD", width=2)
            elif(c_val == 1):
                # if value in cell is 1, draw a blue filled circle at that point
                canvas.create_circle(circ_x, circ_y, circle_rad, fill="blue", outline="#DDD", width=2)


    r.update_idletasks()        # Clear all events in GUI (Not really necessary for us but recommended)
    r.update()                  # Update GUI
    pass

# Convert from i,j coordinates to a single integer that identifies a grid cell
def ij_to_cell_index(i,j):
    # i represents x and j represents y
    global width_map
    cell_index = width_map*j+i
    return cell_index

# Convert from cell_index to (i,j) coordinates
def cell_index_to_ij(cell_index):
    global width_map
    j = cell_index/width_map
    i = cell_index%width_map
    return i, j

# Return cost of traversing from one cell to another
def cost(cell_index_from, cell_index_to):
    global cost_map_rep


    return cost

def assign_int_to_cell(i,j,val):
    global cost_map_rep
    # Get cell index
    c_ind = ij_to_cell_index(i,j)
    # Assign Value
    cost_map_rep[c_ind] = val


if __name__ == "__main__":
    main()


