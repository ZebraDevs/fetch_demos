#ifndef FETCH_DEMOS_COMMON_GRASP_SUGGESTION_INTERFACE_H
#define FETCH_DEMOS_COMMON_GRASP_SUGGESTION_INTERFACE_H
// takes in message type Object and output grasp suggestion
#include <vector>
#include <ros/ros.h>
#include <fetch_demos_common/GetObjectsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <rail_manipulation_msgs/SegmentedObject.h>
#include <rail_manipulation_msgs/BoundingVolume.h>
#include <grasping_msgs/Object.h>

namespace fetch_demmos_common
{
class GraspSuggestionInterface
{
    public:
        GraspSuggestionInterface(ros::NodeHandle& nh);
        std::vector<rail_manipulation_msgs::SegmentedObject> getObject();
    private:
        rail_manipulation_msgs::SegmentedObject object2SegmentedObject(grasping_msgs::Object obj_msg);
        boost::shared_ptr<actionlib::SimpleActionClient<fetch_demos_common::GetObjectsAction>> client_;
        std::vector<rail_manipulation_msgs::SegmentedObject> segmentedObjects_;
};

}// end of namespace
#endif
