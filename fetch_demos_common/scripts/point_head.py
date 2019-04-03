#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal


class PointHeadClient(object):
    def __init__(self):
        # init the action client of the robot and wait fot the server
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()
    
    def look_at(self, x, y, z, frame, duration=1.0):
        # init the control msgs and send it to the action server
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

if __name__ == "__main__":
    rospy.init_node("fetch_demo_node")
    
    # init the point head client 
    head_action = PointHeadClient()
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

        


