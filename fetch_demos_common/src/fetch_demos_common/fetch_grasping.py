#! /usr/bin/env python
import copy
from math import cos, sin, pi

import actionlib
import moveit_msgs.msg
import rospy
import tf2_geometry_msgs
import tf2_ros
from fetch_demos_common.fetch_api import GripperClient
from geometry_msgs.msg import (Point, Pose, PoseStamped)
from grasping_msgs.msg import Object
from moveit_msgs.msg import (CollisionObject, Grasp, MoveItErrorCodes,
                             PickupAction, PickupGoal, PlaceAction, PlaceGoal,
                             PlaceLocation, PlanningScene, AttachedCollisionObject)
from moveit_python import (MoveGroupInterface, PickPlaceInterface,
                           PlanningSceneInterface)
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import PoseStamped




class graspingClient(object):
    def __init__(self, move_group="arm", planner="KPIECEkConfigDefault", angle_min=40, angle_step=10.0, angle_max=90.0):
        self.move_group = move_group
        self.planner = planner
        self.angle = angle_min
        self.angle_step = angle_step
        self.angle_max = angle_max
        
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene_diff_publisher = rospy.Publisher("planning_scene", PlanningScene, queue_size=1)
        
        self.move_group = MoveGroupInterface(self.move_group, "base_link")
        self.move_group.setPlannerId(self.planner )
        self.gripper_client = GripperClient()
        self.attached_obj_pub = rospy.Publisher("attached_collision_object", AttachedCollisionObject, queue_size=10)
        self.planning_scene.is_diff = True 
        self._pick_action = actionlib.SimpleActionClient("place", PlaceAction)
        self._pick_action.wait_for_server()
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def stow(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def intermediate_stow(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.7, -0.3, 0.0, -0.3, 0.0, -0.57, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def print_planning_scene_objs(self):
        # print out the objects in the planning scene
        rospy.loginfo("The attach objects currently in the world: ")
        for obj in self.planning_scene.getKnownAttachedObjects():
            rospy.loginfo(obj)
        rospy.loginfo("The collision objects currently in the world: ")
        for obj in self.planning_scene.getKnownCollisionObjects():
            rospy.loginfo(obj)

    def remove_previous_objects(self):
        # remove previous objects
        rospy.loginfo("Removing objects in the world: ")
        for name in self.planning_scene.getKnownCollisionObjects():
            rospy.loginfo("Removing objects in collision: %s", name)
            self.planning_scene.removeCollisionObject(name, True)
        for name in self.planning_scene.getKnownAttachedObjects():
            rospy.loginfo("Removing attached objects: %s", name)
            self.remove_attached_object(name, "gripper_link")
        self.planning_scene.waitForSync()

    def make_poseStamped(self, frame, pose, orientation=None):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.header.frame_id = frame
        pose_stamped.pose = copy.deepcopy(pose)
        if orientation == None:
            pose_stamped.pose.orientation.x = 0.0 
            pose_stamped.pose.orientation.y = 0.707 
            pose_stamped.pose.orientation.z = 0.0 
            pose_stamped.pose.orientation.w = 0.707
        else:
            pose_stamped.pose.orientation.x = orientation[0]
            pose_stamped.pose.orientation.y = orientation[1]
            pose_stamped.pose.orientation.z = orientation[2]
            pose_stamped.pose.orientation.w = orientation[3]
        return pose_stamped
    def transform_pose(self, pose_stamped, target_frame):
        
        transform = self.tfBuffer.lookup_transform(target_frame,
                                        pose_stamped.header.frame_id, 
                                        rospy.Time(0), 
                                        rospy.Duration(1.0))

        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        return pose_transformed

        
    def makeAttach(self, obj, link_name = 'gripper_link',
                   touch_links=['gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link']):
        
        self.planning_scene.removeCollisionObject(obj.name, False)
        obj_pose = self.make_poseStamped('base_link', obj.primitive_poses[0])
        attached_pose = self.transform_pose(obj_pose, link_name)
        self.planning_scene.attachBox(obj.name, 
                                      obj.primitives[0].dimensions[0],
                                      obj.primitives[0].dimensions[1],
                                      obj.primitives[0].dimensions[2], 
                                      attached_pose.pose.position.x, 
                                      attached_pose.pose.position.y, 
                                      attached_pose.pose.position.z,
                                      link_name, 
                                      touch_links)
        self.print_planning_scene_objs()
        
    def makeDetach(self, obj):
        self.planning_scene.addSolidPrimitive(obj.name, 
                                    obj.primitives[0], 
                                    obj.primitive_poses[0],
                                    use_service=False)

        self.print_planning_scene_objs()

    def update_scene(self, object_list, support_surface_lists):
        # add objects as primitives
        for obj in object_list:
            self.planning_scene.addSolidPrimitive(obj.name, 
                                              obj.primitives[0], 
                                              obj.primitive_poses[0],
                                              use_service=True)
        for surface in support_surface_lists:
            height = surface.primitive_poses[0].position.z
            surface.primitives[0].dimensions = [surface.primitives[0].dimensions[0] + 0.02, 
                                             surface.primitives[0].dimensions[1] + 0.1, 
                                             surface.primitives[0].dimensions[2] + height]
            surface.primitive_poses[0].position.z -= height / 2
            self.planning_scene.addSolidPrimitive(surface.name,
                                        surface.primitives[0],
                                        surface.primitive_poses[0],
                                        use_service = True)
        self.planning_scene.waitForSync()

    def clear_scene(self):
        self.planning_scene.clear()
        self.planning_scene.waitForSync()

    def pick(self, obj, close_gripper_to=0.02, retry=2, tolerance=0.01, x_diff_pick=-0.01, z_diff_pick=0.1, x_diff_grasp=-0.01, z_diff_grasp=0.01):
        rospy.loginfo("plicking the object, %s", obj.name)
        self.gripper_client.fully_open_gripper()
        angle_tmp = self.angle
        input_retry = retry
        success = False
        while angle_tmp <= self.angle_max and not success:
            radien = (angle_tmp / 2.0) * (pi / 180.0)
            orientation = [0.0, sin(radien), 0.0, cos(radien)]

            first_poseStamped = self.make_poseStamped("base_link", obj.primitive_poses[0], orientation)
            first_poseStamped.pose.position.x += x_diff_pick
            first_poseStamped.pose.position.z += z_diff_pick
            while retry > 0:
                rospy.loginfo("picking try on first part: %i, angle: %i, radient: %f", retry, self.angle, radien)
                move_pose_result = self.move_group.moveToPose(first_poseStamped, "gripper_link", tolerance=tolerance, PLAN_ONLY=True)
                rospy.sleep(1.0)
                if move_pose_result.error_code.val == MoveItErrorCodes.SUCCESS:
                    success = True
                    break
                else:
                    if move_pose_result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                        rospy.loginfo("no valid IK found")                
                    rospy.loginfo(move_pose_result.error_code.val)
                retry -= 1
            angle_tmp += self.angle_step
            retry = input_retry
        if retry == 0:
            return False

        angle_tmp = self.angle
        success = False
        curr_retry = retry
        while angle_tmp  <= 90 and not success:
            radien = (angle_tmp  / 2) * (pi / 180)
            orientation = [0.0, sin(radien), 0.0, cos(radien)]
            gripper_pose_stamped = self.make_poseStamped("base_link", obj.primitive_poses[0], orientation)
            gripper_pose_stamped.pose.position.z += z_diff_grasp
            gripper_pose_stamped.pose.position.x += x_diff_grasp
            while curr_retry > 0:
                rospy.loginfo("picking try on second part: %i, angle: %i, radient: %f", curr_retry, self.angle , radien)
                move_pose_result = self.move_group.moveToPose(gripper_pose_stamped, "gripper_link", tolerance=tolerance)
                rospy.sleep(1.0)
                if move_pose_result.error_code.val == MoveItErrorCodes.SUCCESS:
                    success = True
                    break
                else:
                    if move_pose_result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                        rospy.loginfo("no valid IK found")       
                    rospy.loginfo(move_pose_result.error_code.val)
                curr_retry -= 1
            angle_tmp  += self.angle_step
            curr_retry = retry
        if curr_retry == 0:
            return False
        
        rospy.loginfo("closing the gripper")
        self.makeAttach(obj)
        self.gripper_client.close_gripper_to(close_gripper_to)

        rospy.loginfo("done picking")
        return True

    def place(self, place_poseStamped, obj, tolerance=0.01, z_diff_step=0.01, x_diff_step=0.01, x_diff_min=0.03, retry=5):
        rospy.loginfo("placing the object, %s", obj.name)
        current_retry = retry
        current_x_diff_total = 0.0
        while current_retry > 0:
            rospy.loginfo("placing try: %i", current_retry)
            move_pose_result = self.move_group.moveToPose(place_poseStamped, "gripper_link", tolerance=tolerance)
            rospy.sleep(1.5)
            if move_pose_result.error_code.val == MoveItErrorCodes.SUCCESS:
                break
            else:
                if move_pose_result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                    rospy.loginfo("no valid IK found")       
                rospy.loginfo(move_pose_result.error_code.val)
                current_retry -= 1
                if current_retry == 0:
                    place_poseStamped.pose.position.z += z_diff_step
                    place_poseStamped.pose.position.x += x_diff_step
                    current_x_diff_total += x_diff_step
                    if current_x_diff_total >= x_diff_min:
                        break
                    current_retry = retry
        if current_retry == 0:
            return False

        self.remove_attached_object(obj.name, "gripper_link")
        self.planning_scene.removeAttachedObject(obj.name, True)

        self.gripper_client.fully_open_gripper()
        return True
    def remove_attached_object(self, name, link_frame):
        o = AttachedCollisionObject()
        o.object.header.stamp = rospy.Time.now()
        o.object.header.frame_id = link_frame
        o.object.operation = CollisionObject.REMOVE
        o.object.id = name
        self.attached_obj_pub.publish(o)
    def remove_collision_object(self, name):
        self.planning_scene.removeCollisionObject(name, True)


