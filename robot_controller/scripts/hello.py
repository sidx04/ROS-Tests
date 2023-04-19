#!/usr/bin/env python3
import rospy

if __name__=="__main__":
    rospy.init_node("test_node")
    rospy.loginfo("test node has been started...")

    rate=rospy.Rate(1)
    c=0

    while not rospy.is_shutdown():
        rospy.loginfo(c)
        rate.sleep()
        c=c+1