import math
import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16

# GLOBALS 
pose2d_sparki_odometry = Pose2D(0,0,0) #Pose2D message object, contains x,y,theta members in meters and radians
#TODO: Track servo angle in radians
servo_rad = None
#TODO: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
ir_sensor_read = [0 for i in range(5)]
#TODO: Create data structure to hold map representation
height_map = 10
width_map = 10
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
    global pose2d_sparki_odometry
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
    

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    #TODO: Copy this data into your local odometry variable
    pose2d_sparki_odometry = data
def callback_update_state(data):
   # print("got new data",data)
    global ir_sensor_read
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #TODO: Load data into your program's local state variables
    
    ir_sensor_read = state_dict["light_sensors"]
def convert_ultrasonic_to_robot_coords(x_us):
    #TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    x_r, y_r = 0., 0.

    x_r = x_us * math.cos(servo_rad)
    y_r = x_us * math.sin(servo_rad)

    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    #TODO: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0., 0.
    global pose2d_sparki_odometry
    p_x, p_y, p_t = pose2d_sparki_odometry

    cos_t = math.cos(p_t)
    sin_t = math.sin(p_t)

    x_w = cos_t*x_r - sin_t*y_r + p_x
    y_w = sin_t*x_r + cos_t*y_r + p_y

    return x_w, y_w

def populate_map_from_ping(x_ping, y_ping):
    #TODO: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    pass

def display_map():
    #TODO: Display the map
    pass

def ij_to_cell_index(i,j):
    #TODO: Convert from i,j coordinates to a single integer that identifies a grid cell
    return 0

def cell_index_to_ij(cell_index):
    #TODO: Convert from cell_index to (i,j) coordinates
    return 0, 0


def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    return 0

if __name__ == "__main__":
    main()


