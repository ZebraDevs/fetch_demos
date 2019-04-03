#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

class GripperClient(object):
    def __init__(self):
        # init the action client of the robot and wait fot the server
        self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        rospy.loginfo("Waiting for gripper controller...")
        self.gripper_client.wait_for_server()
        rospy.loginfo("The gripper controller is launched")

    def close_gripper_to(self, position, max_effor=50):
        # send the position of the gripper, 0.0 is fully closed and 0.1 is fully open
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effor 
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        rospy.sleep(1.0)

    def fully_open_gripper(self):
        self.close_gripper_to(0.1)
    
    def fully_close_gripper(self):
        self.close_gripper_to(0.0)

if __name__ == "__main__":
    rospy.init_node("fetch_demo_node")
    
    # init the gripper client 
    gripper_action = GripperClient()
    while not rospy.is_shutdown():
        rospy.loginfo("Closing the gripper")
        gripper_action.fully_close_gripper()
        rospy.sleep(1.0)
        rospy.loginfo("Opening the gripper")
        gripper_action.fully_open_gripper()
        rospy.sleep(1.0)
        rospy.loginfo("Closing the gripper to 0.04")
        gripper_action.close_gripper_to(0.04)
        rospy.sleep(1.0)
        


