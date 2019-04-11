#include <vector>
#include <ros/ros.h>
#include <fetch_demos_common/grasp_suggestion_interface.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "grasp_suggestion_node");
    ros::NodeHandle nh("~");
    boost::shared_ptr<fetch_demmos_common::GraspSuggestionInterface> grasp_interface_;
    ros::Publisher pub = nh.advertise<rail_manipulation_msgs::SegmentedObject> ("segment_obj", 5);
    
    grasp_interface_.reset(new fetch_demmos_common::GraspSuggestionInterface(nh));
    while(ros::ok()){
        std::vector<rail_manipulation_msgs::SegmentedObject> obj_result = grasp_interface_->getObject();
        for (auto &obj : obj_result)
        {
            pub.publish(obj);
        }
        ros::Duration(10.0).sleep();

    }
    
    ros::spin();
    return 0;
}