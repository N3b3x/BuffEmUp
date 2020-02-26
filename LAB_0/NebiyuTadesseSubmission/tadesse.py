#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def lab_zero(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def nodeFun():
    pub = rospy.Publisher('lab0',String,queue_size=10)
    rospy.init_node('tadesse', anonymous=False)          # initailize the node to my last name
    rospy.Subscriber('lab0', String,lab_zero)           # Create a subscriber that has a callback function "lab_zero()"
    rate = rospy.Rate(1)                                # Publish only 1 time per second

    # While the program doesn't need to exit
    while not rospy.is_shutdown():
        rob_str = "I will be the best Robotics Engineer in the world! %s" % rospy.get_time() # Create the string to be published
        #rospy.loginfo(rob_str)  # Log the 
        pub.publish(rob_str)    # Publish the message to the topic
        rate.sleep()

if __name__ == "__main__":
    try:
        nodeFun()
    except rospy.ROSInterruptException:
        pass