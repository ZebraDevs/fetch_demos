#include <fetch_demos_common/clustering_node.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(fetch_demos_common::PerceptionNode, nodelet::Nodelet)
namespace fetch_demos_common
{

    PerceptionNode::PerceptionNode()
    {
    }
    PerceptionNode::~PerceptionNode()
    {
    }
    void PerceptionNode::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");
        ros::NodeHandle nh("~");
        server_.reset(new server_t(nh, "perception_cluser_objs", 
                                   boost::bind(&PerceptionNode::cb, this, _1),
                                   false));
        clusterer_.reset(new fetch_demos_perception::PerceptionClustering(nh));
        server_->start();
        ros::spin();
        
    }

    void PerceptionNode::cb(const fetch_demos_common::GetObjectsGoalConstPtr& goal)
    {
        fetch_demos_common::GetObjectsResult result;
        clusterer_->getObjects(result);
        server_->setSucceeded(result);

    }

}
