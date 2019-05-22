#! /usr/bin/env python
import rospy
from fetch_demos_common import fetch_api

if __name__ == "__main__":
    rospy.init_node("fetch_api")
    
    # init the gripper client 
    gripper_action = fetch_api.GripperClient()
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
        

