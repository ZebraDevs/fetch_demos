#include <fetch_demos_common/grasp_suggestion_interface.h>


namespace fetch_demmos_common
{
    GraspSuggestionInterface::GraspSuggestionInterface(ros::NodeHandle& nh)
    {
        client_.reset(new actionlib::SimpleActionClient<fetch_demos_common::GetObjectsAction>("fetch_demos_clustering/perception_cluser_objs", true));
        obj_list_.reset(new rail_manipulation_msgs::SegmentedObjectList);
        ROS_INFO("waiting for the clustering server to start");
        client_->waitForServer();
        ROS_INFO("clustering server started");


    }

    rail_manipulation_msgs::SegmentedObjectList
    GraspSuggestionInterface::getObject()
    {
        segmentedObjects_.clear();
        fetch_demos_common::GetObjectsGoal goal;
        client_->sendGoal(goal);
        ROS_INFO("sending goal to the clustering server");
        client_->waitForResult();
        fetch_demos_common::GetObjectsResult result = *(client_->getResult());
        for (auto &obj : result.objects){
            rail_manipulation_msgs::SegmentedObject segmentObject;
            segmentObject = object2SegmentedObject(obj);
            segmentedObjects_.push_back(segmentObject);
        }
        obj_list_.reset(new rail_manipulation_msgs::SegmentedObjectList);
        obj_list_->header.stamp = ros::Time::now();
        for (auto &obj : segmentedObjects_)
        {
            obj_list_->objects.push_back(obj);
        }
        return *obj_list_.get();
    }


    rail_manipulation_msgs::SegmentedObject
    GraspSuggestionInterface::object2SegmentedObject(grasping_msgs::Object obj_msg)
    {
        rail_manipulation_msgs::SegmentedObject transformed_msg;
        obj_msg.header.frame_id = "base_link";
        transformed_msg.name = obj_msg.name;
        transformed_msg.point_cloud = obj_msg.point_cluster;
        transformed_msg.point_cloud.header.frame_id = "base_link";
        transformed_msg.center = obj_msg.primitive_poses[0].position;
        transformed_msg.orientation = obj_msg.primitive_poses[0].orientation;

        transformed_msg.centroid = obj_msg.primitive_poses[0].position;  // TODO: change to the real centroid
        rail_manipulation_msgs::BoundingVolume bv;
        bv.pose.header = obj_msg.header;
        bv.pose.pose = obj_msg.primitive_poses[0];
        bv.dimensions.x = obj_msg.primitive_poses[0].position.x;
        bv.dimensions.y = obj_msg.primitive_poses[0].position.y;
        bv.dimensions.z = obj_msg.primitive_poses[0].position.z;

        transformed_msg.bounding_volume = bv;
        transformed_msg.width = obj_msg.primitives[0].dimensions[0];
        transformed_msg.depth = obj_msg.primitives[0].dimensions[1];
        transformed_msg.height = obj_msg.primitives[0].dimensions[2];

        transformed_msg.recognized = true;

        return transformed_msg;

    }



} // end of namespace
