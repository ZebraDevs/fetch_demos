#!/usr/bin/env python
import copy
from math import cos, sin, pi, sqrt, pow, fabs, fmod
import actionlib
import rospy
import tf2_geometry_msgs
from fetch_demos_common.fetch_api import PointHeadClient, GripperClient
from fetch_demos_common.fetch_grasping import graspingClient
from fetch_demos_common.msg import GetObjectsAction, GetObjectsGoal
from geometry_msgs.msg import (Point, PointStamped, Pose, PoseStamped,
                               Quaternion)
from grasping_msgs.msg import Object
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf import transformations



class BuildSceneClient(object):
    def __init__(self):
        self.perception_client = actionlib.SimpleActionClient(clustering_topic_, GetObjectsAction)
        rospy.loginfo("Waiting for perception node...")
        self.perception_client.wait_for_server()
        self.object_lists = []
        self.graspable_objects_list = []
        self.surface_lists = []

        self.blue_bin = {"pose":Pose(), "detected":False, "primitive":SolidPrimitive(), "min_point":Point(), "max_point":Point()}
        self.green_bin = {"pose":Pose(), "detected":False, "primitive":SolidPrimitive(), "min_point":Point(), "max_point":Point()}
        self.red_bin = {"pose":Pose(), "detected":False, "primitive":SolidPrimitive(), "min_point":Point(), "max_point":Point()}
        self.yellow_bin = {"pose":Pose(), "detected":False, "primitive":SolidPrimitive(), "min_point":Point(), "max_point":Point()}
        self.bin_dict = {"blue_bin":self.blue_bin, "green_bin":self.green_bin, "red_bin":self.red_bin, "yellow_bin":self.yellow_bin}

    def find_objects(self, sort=True):
        self.object_lists = []
        self.graspable_objects_list = []
        self.surface_lists = []
        self.bin_lists = []
        rospy.loginfo("the length of the object list: %i, the length of the grspable objects list: %i",
                     len(self.object_lists), len(self.graspable_objects_list))
        get_object_goal = GetObjectsGoal()
        rospy.loginfo("Sending goals to perception node...")
        self.perception_client.send_goal(get_object_goal)
        self.perception_client.wait_for_result()
        get_object_result = self.perception_client.get_result()

        if get_object_result.objects :
            rospy.loginfo("got the objects!")
            if sort:
                self.reorder_objects()
            for obj in get_object_result.objects:
                # check if the orientation of the object is valid
                obj_ori = obj.primitive_poses[0].orientation
                obj_quat = [obj_ori.x, obj_ori.y, obj_ori.z, obj_ori.w]
                roll, pitch, yaw = transformations.euler_from_quaternion(obj_quat)
                rospy.loginfo("in find_objects, roll: %d, pitch: %d, yaw: %d", roll * (180.0 / pi), pitch * (180.0 / pi), yaw * (180.0 / pi))
                roll_abs = fabs(fmod(roll * (180.0 / pi), 90))
                pitch_abs = fabs(fmod(pitch * (180.0 / pi), 90))

                # if the orientation is off, rotate it back to horizontal
                if (roll_abs < 65 and roll_abs > 25) or (pitch_abs < 65 and pitch_abs > 25):
                    if roll_abs < 65 and roll_abs > 25:
                        width = obj.primitives[0].dimensions[0]
                        depth = obj.primitives[0].dimensions[1]  * sin(roll_abs)
                        height = obj.primitives[0].dimensions[1] * cos(roll_abs)
                        z_diff = obj.primitives[0].dimensions[2] / 2
                        obj.primitive_poses[0].position.z -= z_diff
                        quat = transformations.quaternion_from_euler(0.0, 0.0, yaw)

                    else:
                        width = obj.primitives[0].dimensions[0]
                        depth = obj.primitives[0].dimensions[1]  * cos(pitch_abs)
                        height = obj.primitives[0].dimensions[1] * sin(pitch_abs)
                        z_diff = obj.primitives[0].dimensions[2] / 2
                        obj.primitive_poses[0].position.z -= z_diff
                        quat = transformations.quaternion_from_euler(0.0, 0.0, yaw)
                        quat = transformations.quaternion_multiply(quat, [0.0, 0.0, 0.707, 0.707])

                    obj.primitives[0].dimensions = [width, depth, height]
                    new_orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
                    obj.primitive_poses[0].orientation = new_orientation

                # if the object is about the size of a bin
                if obj.point_cluster.width > bin_pt_size_min_:
                    # expand the bounding box a bit to prevent collision
                    obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0] + 0.025,
                                                    obj.primitives[0].dimensions[1] + 0.03,
                                                    obj.primitives[0].dimensions[2] + 0.03]
                    if len(obj.properties) > 0:
                        color = obj.properties[0].value
                        x_min = obj.primitive_poses[0].position.x - obj.primitives[0].dimensions[0] / 2
                        x_max = obj.primitive_poses[0].position.x + obj.primitives[0].dimensions[0] / 2
                        y_min = obj.primitive_poses[0].position.y - obj.primitives[0].dimensions[1] / 2
                        y_max = obj.primitive_poses[0].position.y + obj.primitives[0].dimensions[1] / 2
                        z_min = obj.primitive_poses[0].position.z - obj.primitives[0].dimensions[2] / 2
                        z_max = obj.primitive_poses[0].position.z + obj.primitives[0].dimensions[2] / 2
                        rospy.loginfo("bin_name: %s, x_min: %f, x_max: %f, y_min: %f, y_max: %f, z_min: %f, z_max: %f"
                          , color+"_bin", x_min, x_max, y_min, y_max, z_min, z_max)
                        min_point = Point(x=x_min, y=y_min, z=z_min)
                        max_point = Point(x=x_max, y=y_max, z=z_max)
                        if color == "blue" or color == "green" or color == "yellow" or color == "red":
                            obj.name = color+"_bin"
                        else:
                            rospy.loginfo("the cluster detected as a bin doesn't have color value")
                            continue
                        self.bin_dict[obj.name]["detected"] = True
                        self.bin_dict[obj.name]["pose"] = obj.primitive_poses[0]
                        self.bin_dict[obj.name]["primitive"] = obj.primitives[0]
                        self.bin_dict[obj.name]["min_point"] = min_point
                        self.bin_dict[obj.name]["max_point"] = max_point

                        self.bin_lists.append(obj)

            i = 0
            for obj in get_object_result.objects:
                # if the object might be a graspable object,
                # if the matching bin is not detected, the object will not be added to graspable list
                if self.check_graspable(obj):
                    color = obj.properties[0].value
                    if (obj.point_cluster.width < cube_pt_size_max_
                       and self.bin_dict[color+"_bin"]["detected"] == True): # param
                        obj.name = "cube" + str(i)
                        rospy.loginfo("appending object: %s", obj.name)
                        i = i + 1
                        self.graspable_objects_list.append(obj)

            rospy.loginfo("the pose of %s: %f, %f, %f",
                            obj.name,
                            obj.primitive_poses[0].position.x,
                            obj.primitive_poses[0].position.y,
                            obj.primitive_poses[0].position.z)
            self.object_lists = get_object_result.objects

            # adjust the surface
            for surface in get_object_result.surfaces:
                height = surface.primitive_poses[0].position.z
                primitive_height = surface.primitives[0].dimensions[2]
                surface.primitives[0].dimensions = [surface.primitives[0].dimensions[0] + 0.07,
                                                surface.primitives[0].dimensions[1] + 0.07,
                                                surface.primitives[0].dimensions[2] + height]
                surface.primitive_poses[0].position.z -= (height  + primitive_height) / 2
                self.surface_lists.append(surface)
            return True
        else :
            rospy.loginfo("there is no object in the current scene")
            return False


    def reorder_objects(self, priority="z"):
        if priority == "z":
            self.object_lists.sort(key=lambda x : x.primitive_poses[0].position.z, reverse=True)

    def check_graspable(self, obj):
        # param
        if (obj.primitives[0].dimensions[0] > 0.1
            and obj.primitives[0].dimensions[1] > 0.1
            and obj.primitives[0].dimensions[2] > 0.1):
            return False
        color = obj.properties[0].value
        if color != "blue"  and color != "green" and color != "yellow" and color != "red":
            return False
        for bin in self.bin_dict:
            min_p = self.bin_dict[bin]["min_point"]
            max_p = self.bin_dict[bin]["max_point"]
            if (obj.primitive_poses[0].position.x > min_p.x
                and  obj.primitive_poses[0].position.x < max_p.x
                and  obj.primitive_poses[0].position.y > min_p.y
                and  obj.primitive_poses[0].position.y < max_p.y
                and  obj.primitive_poses[0].position.z > min_p.z
                and  obj.primitive_poses[0].position.z < max_p.z) :
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

    def get_bin(self, color):
        if color == "blue" or color == "green" or color == "yellow" or color == "red":
            return self.bin_dict[color+"_bin"]
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

    perception_client = BuildSceneClient()
    head_action = PointHeadClient()
    grasping_client = graspingClient(move_group=move_group_,
                                     planner=planner_,
                                     angle_min=angle_min_,
                                     angle_step=angle_step_,
                                     angle_max=angle_max_)
    # grasping_client.intermediate_stow()
    # rospy.sleep(5.0)
    # grasping_client.stow()
    # rospy.sleep(5.0)
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

            for obj in graspable_obj_lists:
            # obj = graspable_obj_lists[0]
                head_action.look_at(obj.primitive_poses[0].position.x,
                                    obj.primitive_poses[0].position.y,
                                    obj.primitive_poses[0].position.z,
                                    "base_link")
                rospy.sleep(5.5)
                grasping_client.remove_previous_objects()
                grasping_client.update_scene(obj_lists, surface_lists)
                grasping_client.print_planning_scene_objs()

                grasping_client.stow()
                rospy.sleep(3.0)
                picking_result = grasping_client.pick(obj,
                                                    close_gripper_to=close_gripper_to_,
                                                    tolerance=tolerance_,
                                                    x_diff_pick=x_diff_pick_,
                                                    z_diff_pick=z_diff_pick_,
                                                    x_diff_grasp=x_diff_grasp_,
                                                    z_diff_grasp=z_diff_grasp_)
                rospy.sleep(2.5)

                if picking_result:
                    bin_object = perception_client.get_bin(obj.properties[0].value)
                    place_poseStamped = make_poseStamped('base_link', bin_object["pose"], [0.0, 0.0, 0.0, 0.0])
                    place_poseStamped.pose.position.z += z_diff_bin_

                    place_result = grasping_client.place(place_poseStamped,
                                                        obj,
                                                        tolerance=tolerance_,
                                                        x_diff_step=x_diff_bin_step_,
                                                        z_diff_step=z_diff_bin_step_,
                                                        x_diff_min=x_diff_bin_min_)
                    rospy.sleep(2.5)

                    if place_result is False:
                        grasping_client.gripper_client.fully_open_gripper()
                        grasping_client.stow()
                        rospy.sleep(1.5)
                        grasping_client.remove_attached_object(obj.name, "gripper_link")
                        grasping_client.clear_scene()
                else:
                    grasping_client.gripper_client.fully_open_gripper()
                    grasping_client.stow()
                    rospy.sleep(1.5)
                    grasping_client.remove_attached_object(obj.name, "gripper_link")
                    grasping_client.clear_scene()

                rospy.loginfo("scene updated, waiting")
                rospy.sleep(2.5)

                grasping_client.remove_collision_object(obj.name)
                grasping_client.gripper_client.fully_open_gripper()
                rospy.sleep(1.5)
                grasping_client.stow()
                grasping_client.remove_previous_objects()
                obj_lists = [x for x in obj_lists if x.name != obj.name]

        else:
            continue


    rospy.spin()
