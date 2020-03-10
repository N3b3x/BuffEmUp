import math
import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16

# FOR VISUALS
import tkinter as tk                        # Use "sudo apt-get install python3-tk" to get tkinter
r = tk.Tk() 
r.title('World Map') 
canvas = tk.Canvas(r,width=610,height=610)
canvas.pack()

def _create_circle(self, x, y, r, **kwargs):
    return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
tk.Canvas.create_circle = _create_circle


# GLOBALS 
pose2d_sparki_odometry = Pose2D(0,0,0) #Pose2D message object, contains x,y,theta members in meters and radians
#TODO: Track servo angle in radians
servo_rad = None
#TODO: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
ir_sensor_read = [0 for i in range(5)]
#TODO: Create data structure to hold map representation
map_cell_size = 1
height_map = 60
width_map = 42
max_map_dist = math.sqrt(width_map**2 + height_map**2)
map_rep = [0 for x in range(height_map * width_map)]
# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
subscriber_odometry = None
subscriber_state = None
publisher_render = rospy.Publisher('/sparki/render_sim',Empty, queue_size=10)

# CONSTANTS 
IR_THRESHOLD = 300 # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME = 0.5 # In seconds

def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, ir_sensor_read
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry
    #TODO: Init your node to register it with the ROS core
    init()
    while not rospy.is_shutdown():
        #TODO: Implement CYCLE TIME
        begin = time.time()

        publisher_ping.publish(Empty())
        #TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        msg = Float32MultiArray()
        msg.data = [1.0, 1.0]
        #TODO: Implement loop closure here
        # add the msg.data to the where the sparki moves
        #print(ir_sensor_read)
        if(ir_sensor_read[1] < IR_THRESHOLD):
            msg.data[0] = 0
            #print(ir_sensor_read[3],"stef")
            pass
        elif(ir_sensor_read[3] < IR_THRESHOLD):
            #print(ir_sensor_read[1],"3")
            msg.data[1] = 0
            

        #add the publish msg to the motor and to the ping
         
        publisher_motor.publish(msg)
        publisher_ping.publish(Empty())
        publisher_render.publish(Empty())
        display_map()                           # Update the GUI 

        if False:
            rospy.loginfo("Loop Closure Triggered")
            print("5")
        if((time.time() - begin) < 50):
            rospy.sleep(50 - time.time() - begin)

        #TODO: Implement CYCLE TIME
        rospy.sleep(.3)



def init():

    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry,map_rep
    #TODO: Set up your publishers and subscribers
    #TODO: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    rospy.init_node('buffemup')
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
    publisher_ping = rospy.Publisher('sparki/ping_command', Empty, queue_size=10)
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)
    subscriber_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber('/sparki/state', String, callback_update_state)
    rospy.sleep(1)
    #TODO: Set sparki's servo to an angle pointing inward to the map (e.g., 45)
    publisher_servo.publish(90)
    publisher_render.publish(Empty())
    print("Did Init")
    #print(map_rep)
    

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    #TODO: Copy this data into your local odometry variable
    pose2d_sparki_odometry = data

def callback_update_state(data):
   # print("got new data",data)
    global ir_sensor_read, servo_rad
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    servo_rad  = state_dict['servo']    
    ir_sensor_read = state_dict["light_sensors"]

    if 'ping' in state_dict.keys():
        distance = state_dict['ping']
        if distance > 0:
            x_r,y_r = convert_ultrasonic_to_robot_coords(distance)
            x_w,y_w = convert_robot_coords_to_world(x_r,y_r)
            populate_map_from_ping(x_w,y_w)


def convert_ultrasonic_to_robot_coords(x_us):
    #TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    x_r, y_r = 0., 0.

    x_r = x_us * math.cos(servo_rad)
    y_r = x_us * math.sin(servo_rad)

    #convert_robot_coords_to_world(x_r,y_r)
    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    #TODO: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0., 0.
    global pose2d_sparki_odometry
    p_x, p_y, p_t = pose2d_sparki_odometry

    cos_t = math.cos(p_t)
    sin_t = math.sin(p_t)

    x_w = cos_t*x_r - sin_t*y_r + p_x    # Return cost of traversing from one cell to another

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
    #TODO: Display the map
    global width_map, height_map, map_rep, r
    canvas.delete("all")        # Clears canvas

    start_pixel_coord_x = 10    # First circle x coord
    start_pixel_coord_y = 10    # First circle y coord
    circle_rad          = 10    # Circle radius

    circle_offset       = 10    # Pixel offset for each circle

    for i in range (height_map):
        for j in range(width_map):
            c_ind = ij_to_cell_index(i,j)
            c_val = map_rep[c_ind]

            circ_x = start_pixel_coord_x + circle_offset*i
            circ_y = start_pixel_coord_y + circle_offset*j

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
    global width_map
    cell_index = width_map*i+j
    return cell_index

# Convert from cell_index to (i,j) coordinates
def cell_index_to_ij(cell_index):
    global width_map
    i = cell_index/width_map
    j = cell_index%width_map
    return i, j

# Return cost of traversing from one cell to another
def cost(cell_index_from, cell_index_to):
    global max_map_dist

    pos_i, pos_j   = cell_index_to_ij(cell_index_from)
    goal_i, goal_j = cell_index_to_ij(cell_index_to)

    cost = math.sqrt((goal_i-pos_i)**2 + (goal_j-pos_j)**2) * (1/max_map_dist)

    return cost

if __name__ == "__main__":
    main()


