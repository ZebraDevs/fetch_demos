// an actoin sever that sends object message 

#include <iostream>
// ROS 
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <fetch_demos_common/perception_clustering.h>
#include <fetch_demos_common/GetObjectsAction.h>

// PCL

class PerceptionNode
{
    typedef actionlib::SimpleActionServer<fetch_demos_common::GetObjectsAction> server_t;
public:
    PerceptionNode(ros::NodeHandle n)
    {
        server_.reset(new server_t(n, "perception_cluser_objs", 
                                   boost::bind(&PerceptionNode::cb, this, _1),
                                   false));
        clusterer_.reset(new fetch_demos::PerceptionClustering(n));
        server_->start();
    }

private:
    void cb(const fetch_demos_common::GetObjectsGoalConstPtr& goal)
    {
        fetch_demos_common::GetObjectsResult result;
        clusterer_->getObjects(result);
        server_->setSucceeded(result);

    }
    boost::shared_ptr<server_t> server_;
    boost::shared_ptr<fetch_demos::PerceptionClustering> clusterer_;

};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "fetch_demos_clustering");
    ros::NodeHandle nh("~");
    PerceptionNode perception(nh);
    ros::Duration(3.0).sleep();

    // add a subsriber here for receiveing the registered point cloud
    ros::spin();
    return 0;

}