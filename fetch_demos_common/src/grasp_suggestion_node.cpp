#include <vector>
#include <ros/ros.h>
#include <fetch_demos_common/grasp_suggestion_interface.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_manipulation_msgs/SegmentedObject.h>

class GraspSuggestionNode
{
    public:
        GraspSuggestionNode(ros::NodeHandle &nh)
        {
            grasp_interface.reset(new fetch_demmos_common::GraspSuggestionInterface(nh));
            obj_list.reset(new rail_manipulation_msgs::SegmentedObjectList);
        }
        rail_manipulation_msgs::SegmentedObjectList getObjectList()
        {
            *obj_list = grasp_interface->getObject();
            return *obj_list;
        }

        void timerCallback(const ros::TimerEvent& event)
        {
            *obj_list = grasp_interface->getObject();

        }
    private:
        boost::shared_ptr<fetch_demmos_common::GraspSuggestionInterface> grasp_interface;
        boost::shared_ptr<rail_manipulation_msgs::SegmentedObjectList> obj_list;

};


int main(int argc, char** argv){
    ros::init(argc, argv, "grasp_suggestion_node");
    ros::NodeHandle nh("~");
    GraspSuggestionNode grasp_suggestion_node(nh);
    rail_manipulation_msgs::SegmentedObjectList obj_list;
    ros::Publisher pub = nh.advertise<rail_manipulation_msgs::SegmentedObjectList> ("segment_obj", 5);
    // ros::Timer timer = nh.createTimer(ros::Duration(3.0), &GraspSuggestionNode::timerCallback, &grasp_suggestion_node);
    ros::Rate loop_rate(10);
    double old_time =ros::Time::now().toSec();
    while(ros::ok()){
        double new_time =ros::Time::now().toSec();
        if(new_time - old_time > 5.0)
        {
            obj_list = grasp_suggestion_node.getObjectList();
            old_time = new_time;
        }
        pub.publish(obj_list);
        // ros::spinOnce();
        // loop_rate.sleep();

    }

    ros::spin();
    return 0;
}
