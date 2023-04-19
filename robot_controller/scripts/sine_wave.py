#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

PI = 3.1415926535897
theta = 0

def pose_callback(pose):
    global theta
    req =  2 * math.pi
    if pose.theta < 0:
        alpha = req - (pose.theta + (2 * math.pi))
    else:
        alpha = req - pose.theta

    alpha = 2 * math.pi - alpha
    theta = alpha

# sine wave
def sin_graph():
    global theta
    rospy.init_node('sin_graph', anonymous=True)

    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=10)
    
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    msg = Twist()

    speed = 0.2
    radius = 1

    msg.linear.x = speed
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = speed/radius

    rate = rospy.Rate(10);

    while not rospy.is_shutdown():
        
        msg.linear.x = speed * math.cos(theta)
        msg.angular.z =  math.sin(theta)
        velocity_publisher.publish(msg)
       
        rospy.loginfo("Moving in a sine curve")
        print(theta)
        rate.sleep()

    
    print("Goal Reached")
    msg.linear.x = 0
    msg.angular.z = 0
    velocity_publisher.publish(msg)
    rospy.spin()


if __name__ == '__main__':
    try:
        sin_graph()
    except rospy.ROSInterruptException:
        pass

