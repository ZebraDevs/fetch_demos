
import actionlib
import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal, GripperCommandAction,
                              GripperCommandGoal, PointHeadAction,
                              PointHeadGoal)


class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy .loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy .Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z      
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

class GripperClient(object):
    def __init__(self):
        # init the action client of the robot and wait fot the server
        self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        rospy.loginfo("Waiting for gripper controller...")
        self.gripper_client.wait_for_server()
        rospy.loginfo("The gripper controller is launched")

    def close_gripper_to(self, position, max_effor=90):
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