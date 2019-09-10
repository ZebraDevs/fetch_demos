#! /usr/bin/env python
import rospy
from fetch_demos_common import fetch_api

if __name__ == '__main__':
    rospy.init_node("fetch_api")
    head_action = fetch_api.PointHeadClient()
    while not rospy.is_shutdown():
        # send the points to the client, the head will look at the point 
        # the sleep is for the head to fully move to the point 
        head_action.look_at(1.0, 1.0, 0.5, "base_link")
        rospy.sleep(1.5)
        head_action.look_at(1.0, 0.0, 0.5, "base_link")
        rospy.sleep(1.5)
        head_action.look_at(1.0, -1.0, 0.5, "base_link")
        rospy.sleep(1.5)
        head_action.look_at(1.0, 0.0, 0.5, "base_link")
        rospy.sleep(1.5)
        head_action.look_at(1.0, 0.0, 0.8, "base_link")
        rospy.sleep(1.5)
        head_action.look_at(1.0, 0.0, 0.3, "base_link")
        rospy.sleep(1.5)