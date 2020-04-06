import math
import sys
import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray, Empty, String, Int16, Header
from geometry_msgs.msg import PoseStamped


def MoveToGoal(x,y,theta):
    rospy.init_node('buffemup')
    publisher_Goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id =  'base_link'
    p = Pose()
    
    p.position.x = x
    p.position.y = y
    p.orientation.x = 0.0
    p.orientation.x = 0.0
    p.orientation.x = 0.0
    p.orientation.w = theta
    goal = PoseStamped(h,p)
    publisher_Goal.publish(goal)

if __name__ == "__main__":
    MoveToGoal(.5,0,1)
   
    