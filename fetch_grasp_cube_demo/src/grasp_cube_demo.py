#!/usr/bin/env python
import copy
from math import cos, sin, pi

import actionlib
import moveit_msgs.msg
import rospy
import tf2_geometry_msgs
from fetch_demos_common.fetch_api import PointHeadClient, GripperClient
from fetch_demos_common.fetch_grasping import graspingClient
from fetch_demos_common.msg import GetObjectsAction, GetObjectsGoal
from geometry_msgs.msg import (Point, PointStamped, Pose, PoseStamped,
                               Quaternion)
from grasping_msgs.msg import (FindGraspableObjectsAction,
                               FindGraspableObjectsGoal, GraspPlanningAction,
                               GraspPlanningGoal, Object)
from moveit_msgs.msg import (CollisionObject, Grasp, MoveItErrorCodes,
                             PickupAction, PickupGoal, PlaceAction, PlaceGoal,
                             PlaceLocation, PlanningScene, AttachedCollisionObject)
from moveit_python import (MoveGroupInterface, PickPlaceInterface,
                           PlanningSceneInterface)
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import String
from tf2_geometry_msgs import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander



class perceptionClient(object):
    def __init__(self):
        self.perception_client = actionlib.SimpleActionClient(clustering_topic_, GetObjectsAction)
        rospy.loginfo("Waiting for percetion node...")
        self.perception_client.wait_for_server()
        self.object_lists = []
        self.graspable_objects_list = []
        self.surface_lists = []
        self.bin_lists = []
        self.blue_bin_pose = Pose()
        self.green_bin_pose = Pose()
        self.yellow_bin_pose = Pose()
        self.red_bin_pose = Pose()

    def find_objects(self, sort=True):
        self.object_lists = []
        self.graspable_objects_list = []
        self.surface_lists = []
        self.bin_lists = []
        rospy.loginfo("the length of the object list: %i, the length of the grspable objects list: %i",
                     len(self.object_lists), len(self.graspable_objects_list))
        get_object_goal = GetObjectsGoal()
        rospy.loginfo("Sending goals to percetion node...")
        self.perception_client.send_goal(get_object_goal)
        self.perception_client.wait_for_result()
        get_object_result = self.perception_client.get_result()

        if get_object_result.objects :
            rospy.loginfo("got the objects!")
            for obj in get_object_result.objects:
                if obj.point_cluster.width > bin_pt_size_min_: 
                    obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0] + 0.025,
                                                    obj.primitives[0].dimensions[1] + 0.03,
                                                    obj.primitives[0].dimensions[2] + 0.02]
                    if len(obj.properties) > 0:
                        color = obj.properties[0].value
                        if color == "blue":
                            obj.name = "blue_bin"
                            rospy.loginfo("the pose of blue bin: %f, %f, %f",
                                           obj.primitive_poses[0].position.x,
                                           obj.primitive_poses[0].position.y,
                                           obj.primitive_poses[0].position.z)
                            self.blue_bin_pose = obj.primitive_poses[0]
                        elif color == "green":
                            obj.name = "green_bin"
                            rospy.loginfo("the pose of green bin: %f, %f, %f",
                                           obj.primitive_poses[0].position.x,
                                           obj.primitive_poses[0].position.y,
                                           obj.primitive_poses[0].position.z)
                            self.green_bin_pose = obj.primitive_poses[0]
                        elif color == "yellow":
                            obj.name = "yellow_bin"
                            rospy.loginfo("the pose of yellow bin: %f, %f, %f",
                                           obj.primitive_poses[0].position.x,
                                           obj.primitive_poses[0].position.y,
                                           obj.primitive_poses[0].position.z)
                            self.yellow_bin_pose = obj.primitive_poses[0]
                        elif color == "red":
                            obj.name = "red_bin"
                            rospy.loginfo("the pose of red bin: %f, %f, %f",
                                           obj.primitive_poses[0].position.x,
                                           obj.primitive_poses[0].position.y,
                                           obj.primitive_poses[0].position.z)
                            self.red_bin_pose = obj.primitive_poses[0]
                        self.bin_lists.append(obj)

            i = 0
            for obj in get_object_result.objects:
                rospy.loginfo(obj.properties[0].value)
                rospy.loginfo(obj.primitive_poses[0].position.x)
                rospy.loginfo(obj.primitive_poses[0].position.y)
                rospy.loginfo(obj.primitive_poses[0].position.z)
                rospy.loginfo("the size of the cluster pointcloud: %i:", 
                              obj.point_cluster.width)
                rospy.loginfo("object name: %s", obj.name)
                if self.check_graspable(obj):
                    if obj.point_cluster.width < cube_pt_size_max_: # param
                        obj.name = "cube" + str(i)
                        rospy.loginfo("appending object: %s", obj.name)
                        i = i + 1
                        self.graspable_objects_list.append(obj)

            self.object_lists = get_object_result.objects
            self.surface_lists = get_object_result.surfaces
            if sort:
                self.reorder_objects()

            return True
        else : 
            rospy.loginfo("there is no object in the current scene")
            return False


    def reorder_objects(self, priority="x"):
        if priority == "x":
            sorted(self.object_lists, key=lambda x : x.primitive_poses[0].position.x, reverse=False)

    def check_graspable(self, obj):
        for bin in self.bin_lists:
            x_min = bin.primitive_poses[0].position.x - bin.primitives[0].dimensions[0] / 2
            x_max = bin.primitive_poses[0].position.x + bin.primitives[0].dimensions[0] / 2
            y_min = bin.primitive_poses[0].position.y - bin.primitives[0].dimensions[1] / 2
            y_max = bin.primitive_poses[0].position.y + bin.primitives[0].dimensions[1] / 2
            z_min = bin.primitive_poses[0].position.z - bin.primitives[0].dimensions[2] / 2
            z_max = bin.primitive_poses[0].position.z + bin.primitives[0].dimensions[2] / 2
            rospy.loginfo("bin_name: %s, x_min: %f, x_max: %f, y_min: %f, y_max: %f, z_min: %f, z_max: %f"
                          , bin.name, x_min, x_max, y_min, y_max, z_min, z_max)
            if (obj.primitive_poses[0].position.x > x_min
                and  obj.primitive_poses[0].position.x < x_max
                and  obj.primitive_poses[0].position.y > y_min
                and  obj.primitive_poses[0].position.y < y_max
                and  obj.primitive_poses[0].position.z > z_min
                and  obj.primitive_poses[0].position.z < z_max) :
                return False
        return True

    def get_object_list(self):
        return self.object_lists
    
    def get_bin_list(self):
        return self.bin_lists

    def get_surface_list(self):
        return self.surface_lists

    def get_graspable_object_lists(self):
        return self.graspable_objects_list

    def get_bin_pose(self, color):
        if color == "blue":
            return self.blue_bin_pose
        elif color == "green":
            return self.green_bin_pose
        elif color == "yellow":
            return self.yellow_bin_pose
        elif color == "red":
            return self.red_bin_pose
        else:
            rospy.loginfo("the request color is not valid")
            return Pose()  


def make_poseStamped(frame, pose, orientation=None):
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

if __name__ == "__main__":
    node_name = "fetch_demos_grasp_cube"
    rospy.init_node(node_name)

    clustering_topic_ = rospy.get_param(node_name + '/clustering_topic')
    bin_pt_size_min_ = rospy.get_param(node_name + '/recognition/bin_pt_size_min')
    cube_pt_size_max_ = rospy.get_param(node_name + '/recognition/cube_pt_size_max')
    tolerance_ = rospy.get_param(node_name + '/planning/tolerance')
    planner_ = rospy.get_param(node_name + '/planning/planner')
    move_group_ = rospy.get_param(node_name + '/planning/move_group')
    x_diff_pick_ = rospy.get_param(node_name + '/grasping/x_diff_pick')
    z_diff_pick_ = rospy.get_param(node_name + '/grasping/z_diff_pick')
    x_diff_grasp_ = rospy.get_param(node_name + '/grasping/x_diff_grasp')
    z_diff_grasp_ = rospy.get_param(node_name + '/grasping/z_diff_grasp')
    angle_min_ = rospy.get_param(node_name + '/grasping/angle_min')
    angle_step_ = rospy.get_param(node_name + '/grasping/angle_step')
    angle_max_ = rospy.get_param(node_name + '/grasping/angle_max')
    close_gripper_to_ = rospy.get_param(node_name + '/grasping/close_gripper_to')
    z_diff_bin_ = rospy.get_param(node_name + '/placing/z_diff_bin')
    x_diff_bin_ = rospy.get_param(node_name + '/placing/x_diff_bin')
    z_diff_bin_step_ = rospy.get_param(node_name + '/placing/z_diff_bin_step')
    x_diff_bin_step_ = rospy.get_param(node_name + '/placing/x_diff_bin_step')
    x_diff_bin_min_ = rospy.get_param(node_name + '/placing/x_diff_bin_min')

    perception_client = perceptionClient()
    head_action = PointHeadClient()
    grasping_client = graspingClient(move_group=move_group_,
                                     planner=planner_, 
                                     angle_min=angle_min_,
                                     angle_step=angle_step_,
                                     angle_max=angle_max_)
    grasping_client.intermediate_stow()
    grasping_client.stow()
    rospy.loginfo("successfully initialized")

    head_action.look_at(1.0, 0.0, 0.5, "base_link")
    place_result = False
    picking_result = False
        

    while not rospy.is_shutdown():
        find_object_success = perception_client.find_objects()

        if find_object_success:
            obj_lists = perception_client.get_object_list()
            graspable_obj_lists = perception_client.get_graspable_object_lists()
            if len(graspable_obj_lists) <= 0:
                rospy.loginfo("there is no grspable object in the view")
                break
            surface_lists = perception_client.get_surface_list()
            grasping_client.clear_scene()

            obj = graspable_obj_lists[0]
            head_action.look_at(obj.primitive_poses[0].position.x, 
                                obj.primitive_poses[0].position.y,
                                obj.primitive_poses[0].position.z,
                                "base_link")
            rospy.sleep(0.5)

            grasping_client.remove_previous_objects()
            grasping_client.update_scene(obj_lists, surface_lists)
            grasping_client.print_planning_scene_objs()
            
            picking_result = grasping_client.pick(obj,
                                                  close_gripper_to=close_gripper_to_, 
                                                  tolerance=tolerance_, 
                                                  x_diff_pick=x_diff_pick_, 
                                                  z_diff_pick=z_diff_pick_, 
                                                  x_diff_grasp=x_diff_grasp_, 
                                                  z_diff_grasp=z_diff_grasp_)
            if picking_result:
                bin_pose = perception_client.get_bin_pose(obj.properties[0].value)
                place_poseStamped = make_poseStamped('base_link', bin_pose, [0.0, 0.0, 0.0, 0.0])
                place_poseStamped.pose.position.z += z_diff_bin_

                place_result = grasping_client.place(place_poseStamped,
                                                     obj,
                                                     tolerance=tolerance_, 
                                                     x_diff_step=x_diff_bin_step_, 
                                                     z_diff_step=z_diff_bin_step_, 
                                                     x_diff_min=x_diff_bin_min_)
                if place_result is False:
                    grasping_client.remove_attached_object(obj.name, "gripper_link")
                    grasping_client.clear_scene()                    
            else:
                grasping_client.remove_attached_object(obj.name, "gripper_link")
            
            grasping_client.remove_collision_object(obj.name)
            grasping_client.gripper_client.fully_open_gripper()
            grasping_client.intermediate_stow()
            grasping_client.stow()
            grasping_client.remove_previous_objects()
        
        else:
            continue


    rospy.spin()
