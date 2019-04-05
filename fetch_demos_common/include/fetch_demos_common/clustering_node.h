#ifndef FETCH_DEMOS_COMMON_CLUSTERING_NODE_H
#define FETCH_DEMOS_COMMON_CLUSTERING_NODE_H
// an actoin sever that sends object message 

#include <iostream>
// ROS 
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <fetch_demos_common/perception_clustering.h>
#include <fetch_demos_common/GetObjectsAction.h>
#include <nodelet/nodelet.h>
#include "nodelet/loader.h"
namespace fetch_demos_common
{
class PerceptionNode  : public nodelet::Nodelet
{
    typedef actionlib::SimpleActionServer<fetch_demos_common::GetObjectsAction> server_t;
    public:
        PerceptionNode();
        ~PerceptionNode();

    private:
        boost::shared_ptr<server_t> server_;
        boost::shared_ptr<fetch_demos_perception::PerceptionClustering> clusterer_;
        virtual void onInit();

        void cb(const fetch_demos_common::GetObjectsGoalConstPtr& goal);

};

} // end of fetch_demos_common namespace

#endif