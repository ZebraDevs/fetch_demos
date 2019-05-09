#ifndef FETCH_DEMOS_COMMON_GRASP_SUGGESTION_INTERFACE_H
#define FETCH_DEMOS_COMMON_GRASP_SUGGESTION_INTERFACE_H
// takes in message type Object and output grasp suggestion
#include <vector>
#include <ros/ros.h>
#include <fetch_demos_common/GetObjectsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <rail_manipulation_msgs/SegmentedObject.h>
<<<<<<< HEAD
#include <rail_manipulation_msgs/SegmentedObjectList.h>
=======
>>>>>>> 215b07473b0c805c0255e2ee65b574780f713b93
#include <rail_manipulation_msgs/BoundingVolume.h>
#include <grasping_msgs/Object.h>

namespace fetch_demmos_common
{
class GraspSuggestionInterface
{
    public:
        GraspSuggestionInterface(ros::NodeHandle& nh);
<<<<<<< HEAD
        rail_manipulation_msgs::SegmentedObjectList getObject();
=======
        std::vector<rail_manipulation_msgs::SegmentedObject> getObject();
>>>>>>> 215b07473b0c805c0255e2ee65b574780f713b93
    private:
        rail_manipulation_msgs::SegmentedObject object2SegmentedObject(grasping_msgs::Object obj_msg);
        boost::shared_ptr<actionlib::SimpleActionClient<fetch_demos_common::GetObjectsAction>> client_;
        std::vector<rail_manipulation_msgs::SegmentedObject> segmentedObjects_;
<<<<<<< HEAD
        boost::shared_ptr<rail_manipulation_msgs::SegmentedObjectList> obj_list_;
=======
>>>>>>> 215b07473b0c805c0255e2ee65b574780f713b93
};

}// end of namespace
#endif
