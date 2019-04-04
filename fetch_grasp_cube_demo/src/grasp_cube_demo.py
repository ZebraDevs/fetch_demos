#!/usr/bin/env python
import copy
from math import cos, sin, pi

import actionlib
import moveit_msgs.msg
import rospy
import tf2_geometry_msgs
import tf2_ros
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal, GripperCommandAction,
                              GripperCommandGoal, PointHeadAction,
                              PointHeadGoal)
from fetch_demos_common.msg import GetObjectsAction, GetObjectsGoal
from geometry_msgs.msg import (Point, PointStamped, Pose, PoseStamped,
                               Quaternion)
from grasping_msgs.msg import (FindGraspableObjectsAction,
                               FindGraspableObjectsGoal, GraspPlanningAction,
                               GraspPlanningGoal, Object)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import (CollisionObject, Grasp, MoveItErrorCodes,
                             PickupAction, PickupGoal, PlaceAction, PlaceGoal,
                             PlaceLocation, PlanningScene, AttachedCollisionObject)
from moveit_python import (MoveGroupInterface, PickPlaceInterface,
                           PlanningSceneInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
# from pointcloud_process.msg import clusterObj
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import String
from tf2_geometry_msgs import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander

# gripper pose for placing
# TODO: replace all the hard-coded value for placing

place_pose_stamped = PoseStamped()
place_pose_stamped.header.stamp = rospy.Time(0)
place_pose_stamped.header.frame_id = 'base_link'

place_pose_stamped.pose.position.x = 0.3
place_pose_stamped.pose.position.y = 0.0
place_pose_stamped.pose.position.z = 1.0  
place_pose_stamped.pose.orientation.w = 0.707
place_pose_stamped.pose.orientation.x = 0.0
place_pose_stamped.pose.orientation.y = 0.707
place_pose_stamped.pose.orientation.z = 0.0

class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z      
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

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
                if obj.name == "bin":
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
                    if obj.name == "cube":
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
        
class graspingClient(object):
    def __init__(self):
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene_diff_publisher = rospy.Publisher("planning_scene", PlanningScene, queue_size=1)
        
        self.move_group = MoveGroupInterface(move_group_, "base_link")
        self.move_group.setPlannerId(planner_)
        self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        rospy.loginfo("Waiting for graspingClient...")
        self.gripper_client.wait_for_server()
        self.attached_obj_pub = rospy.Publisher("attached_collision_object", AttachedCollisionObject, queue_size=10)
        self.planning_scene.is_diff = True 
        self._pick_action = actionlib.SimpleActionClient("place", PlaceAction)
        self._pick_action.wait_for_server()
        self.angle = angle_min_


    def close_gripper_to(self, position, max_effor=90):
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effor # 50 is gentle grasp
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

    def fully_open_gripper(self):
        self.close_gripper_to(0.1)
    
    def fully_close_gripper(self):
        self.close_gripper_to(0.0)

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
        
    def makeAttach(self, obj):
        
        self.planning_scene.removeCollisionObject(obj.name, False)
        link_name = 'gripper_link'
        touch_links = [
        'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'
        ]
        pose_stamped = self.make_poseStamped("base_link", obj.primitive_poses[0])
        new_pose = transform_pose(pose_stamped, link_name)        
        self.planning_scene.attachBox(obj.name, 
                                      obj.primitives[0].dimensions[0],
                                      obj.primitives[0].dimensions[1],
                                      obj.primitives[0].dimensions[2], 
                                      new_pose.pose.position.x, 
                                      new_pose.pose.position.y, 
                                      new_pose.pose.position.z,
                                      link_name, 
                                      touch_links)
        self.print_planning_scene_objs()
        
    def makeDetach(self, obj):
        # self.planning_scene.removeAttachedObject(obj.name, False)
        self.planning_scene.addSolidPrimitive(obj.name, 
                                    obj.primitives[0], 
                                    obj.primitive_poses[0],
                                    use_service=False)

        self.print_planning_scene_objs()

    def update_scene(self, object_list, support_surface_lists):
        # add objects as primitives
        for obj in obj_lists:
            self.planning_scene.addSolidPrimitive(obj.name, 
                                              obj.primitives[0], 
                                              obj.primitive_poses[0],
                                              use_service=True)
        for surface in support_surface_lists:
            height = surface.primitive_poses[0].position.z
            # param: + the diff of the attached object
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

    def pick(self, obj, retry=2):
        rospy.loginfo("plicking the object, %s", obj.name)
        self.fully_open_gripper()
        
        input_retry = retry
        success = False
        while self.angle <= angle_max_ and not success:
            radien = (self.angle / 2.0) * (pi / 180.0)
            orientation = [0.0, sin(radien), 0.0, cos(radien)]

            first_poseStamped = self.make_poseStamped("base_link", obj.primitive_poses[0], orientation)
            first_poseStamped.pose.position.x += x_diff_pick_
            first_poseStamped.pose.position.z += z_diff_pick_
            while retry > 0:
                rospy.loginfo("picking try on first part: %i, angle: %i, radient: %f", retry, self.angle, radien)
                move_pose_result = self.move_group.moveToPose(first_poseStamped, "gripper_link", tolerance=tolerance_, PLAN_ONLY=True)
                rospy.sleep(1.0)
                if move_pose_result.error_code.val == MoveItErrorCodes.SUCCESS:
                    success = True
                    break
                else:
                    if move_pose_result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                        rospy.loginfo("no valid IK found")                
                    rospy.loginfo(move_pose_result.error_code.val)
                retry -= 1
            self.angle += angle_step_
            retry = input_retry
        if retry == 0:
            return False

        self.angle = angle_min_
        success = False
        curr_retry = retry
        while self.angle  <= 90 and not success:
            radien = (self.angle  / 2) * (pi / 180)
            orientation = [0.0, sin(radien), 0.0, cos(radien)]
            gripper_pose_stamped = self.make_poseStamped("base_link", obj.primitive_poses[0], orientation)
            gripper_pose_stamped.pose.position.z += z_diff_grasp_
            gripper_pose_stamped.pose.position.x += x_diff_grasp_
            while curr_retry > 0:
                rospy.loginfo("picking try on second part: %i, angle: %i, radient: %f", curr_retry, self.angle , radien)
                move_pose_result = self.move_group.moveToPose(gripper_pose_stamped, "gripper_link", tolerance=tolerance_)
                rospy.sleep(1.0)
                if move_pose_result.error_code.val == MoveItErrorCodes.SUCCESS:
                    success = True
                    break
                else:
                    if move_pose_result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                        rospy.loginfo("no valid IK found")       
                    rospy.loginfo(move_pose_result.error_code.val)
                curr_retry -= 1
            self.angle  += angle_step_
            curr_retry = retry
        if curr_retry == 0:
            return False
        
        rospy.loginfo("closing the gripper")
        self.makeAttach(obj)
        self.close_gripper_to(close_gripper_to_)

        rospy.loginfo("done picking")
        return True

    def place(self, bin_pose, obj, retry=5):
        rospy.loginfo("placing the object, %s", obj.name)
        radien = (self.angle  / 2) * (pi / 180)
        orientation = [0.0, sin(radien), 0.0, cos(radien)]
        first_poseStamped = self.make_poseStamped("base_link", obj.primitive_poses[0], orientation)
        z_diff_place = z_diff_place_
        x_diff_place = x_diff_place_
        first_poseStamped.pose.position.z += z_diff_place
        first_poseStamped.pose.position.x += x_diff_place
        
        place_poseStamped = PoseStamped()
        place_poseStamped.header.stamp = rospy.Time(0)
        place_poseStamped.header.frame_id = 'base_link'

        place_poseStamped.pose = bin_pose
        place_poseStamped.pose.position.z += z_diff_bin_
        # move to the first picking pose
        # move_pose_result = self.move_group.moveToPose(first_poseStamped, "gripper_link", tolerance=tolerance_)
        # rospy.sleep(1.0)
        # if move_pose_result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
        #     rospy.loginfo("no valid IK found")       
        # if move_pose_result.error_code.val == MoveItErrorCodes.SUCCESS:
        #     pass
        # else:
        #     rospy.loginfo(move_pose_result.error_code.val)
        current_retry = retry
        current_x_diff_total = 0.0
        while current_retry > 0:
            rospy.loginfo("placing try: %i", current_retry)
            move_pose_result = self.move_group.moveToPose(place_poseStamped, "gripper_link", tolerance=tolerance_)
            rospy.sleep(1.5)
            if move_pose_result.error_code.val == MoveItErrorCodes.SUCCESS:
                break
            else:
                if move_pose_result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                    rospy.loginfo("no valid IK found")       
                rospy.loginfo(move_pose_result.error_code.val)
                current_retry -= 1
                if current_retry == 0:
                    place_poseStamped.pose.position.z += z_diff_bin_step_
                    place_poseStamped.pose.position.x += x_diff_bin_step_
                    current_x_diff_total += x_diff_bin_step_
                    if current_x_diff_total >= x_diff_bin_min_:
                        break
                    current_retry = retry
        if current_retry == 0:
            return False

        self.remove_attached_object(obj.name, "gripper_link")
        self.planning_scene.removeAttachedObject(obj.name, True)

        self.fully_open_gripper()
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


def transform_pose(pose_stamped, target_frame):
    transform = tfBuffer.lookup_transform(target_frame,
                                       pose_stamped.header.frame_id, 
                                       rospy.Time(0), 
                                       rospy.Duration(1.0))

    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
    return pose_transformed

if __name__ == "__main__":
    node_name = "fetch_demos_grasp_cube"
    rospy.init_node(node_name)

    clustering_topic_ = rospy.get_param(node_name + '/clustering_topic')
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
    x_diff_place_ = rospy.get_param(node_name + '/placing/x_diff_place')
    z_diff_place_ = rospy.get_param(node_name + '/placing/z_diff_place')
    z_diff_bin_ = rospy.get_param(node_name + '/placing/z_diff_bin')
    x_diff_bin_ = rospy.get_param(node_name + '/placing/x_diff_bin')
    z_diff_bin_step_ = rospy.get_param(node_name + '/placing/z_diff_bin_step')
    x_diff_bin_step_ = rospy.get_param(node_name + '/placing/x_diff_bin_step')
    x_diff_bin_min_ = rospy.get_param(node_name + '/placing/x_diff_bin_min')

    perception_client = perceptionClient()
    head_action = PointHeadClient()
    grasping_client = graspingClient()
    grasping_client.intermediate_stow()
    grasping_client.stow()
    rospy.loginfo("successfully initialized")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

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

            picking_result = grasping_client.pick(obj)
            if picking_result:
                bin_pose = perception_client.get_bin_pose(obj.properties[0].value)
                place_result = grasping_client.place(bin_pose, obj)
                if place_result is False:
                    grasping_client.remove_attached_object(obj.name, "gripper_link")
                    grasping_client.clear_scene()                    
            else:
                grasping_client.remove_attached_object(obj.name, "gripper_link")
            
            grasping_client.remove_collision_object(obj.name)
            grasping_client.fully_open_gripper()
            grasping_client.intermediate_stow()
            grasping_client.stow()
            grasping_client.remove_previous_objects()
        
        else:
            continue


    rospy.spin()
