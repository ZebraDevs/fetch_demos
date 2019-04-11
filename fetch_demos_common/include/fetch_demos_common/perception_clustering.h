#ifndef FETCH_DEMOS_COMMON_PERCEPTION_CLUSTERING_H
#define FETCH_DEMOS_COMMON_PERCEPTION_CLUSTERING_H
// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
// ros
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <grasping_msgs/Object.h>
#include <geometry_msgs/Pose.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <ros/node_handle.h>                  
#include <ros/publisher.h>                   
#include <ros/subscriber.h>                   
#include <tf2_ros/buffer.h>               
#include "fetch_demos_common/GetObjectsResult.h"    


#define PI 3.14159265
namespace fetch_demos_perception
{

class PerceptionClustering
{
public:
    PerceptionClustering(ros::NodeHandle& nh);
    virtual void getObjects(fetch_demos_common::GetObjectsResult& result);

private:
    void cb(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void pc2_filtering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& incloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outcloud_ptr);
    void planeRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inpcPtr);
    std::vector<grasping_msgs::Object> euclideanCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcPtr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud2msgtoPCL(const sensor_msgs::PointCloud2ConstPtr& cloud2_msg);
    sensor_msgs::PointCloud2 PCLtoPointCloud2msg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_pc,  std::string frame="head_camera_rgb_optical_frame");
    void transform_frame(geometry_msgs::PointStamped& transformed_pt, double x, double y, double z, std::string target_frame, std::string source_frame);
    std::string color_extractor(pcl::PointXYZHSV& pointHSV);
    void head_lookat(double x, double y, double z, std::string frame_name);
    geometry_msgs::TransformStamped lookup_transform(std::string target_frame, std::string source_frame);
    void scanScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_pcPtr, int degree_start=-30, int degree_end=30);
    bool extractUnorientedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>& input, shape_msgs::SolidPrimitive& shape, geometry_msgs::Pose& pose);
    void pcPtr2Objectmsg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_pcPtr, grasping_msgs::Object& input_object_msg, std::string name);

    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_Listener_;
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::PointHeadAction>> head_action_;

    ros::Subscriber head_camera_cloud_sub_;
    ros::Publisher debug_cloud_pub_;
    ros::Publisher obstacle_cloud_pub_;
    ros::Publisher objects_cloud_pub_;
    ros::Publisher plane_cloud_pub_;
    ros::Publisher sum_cloud_pub_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcPtr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_sum_pcPtr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcPtr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pcPtr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_pcPtr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacles_pcPtr_;

    std::vector<grasping_msgs::Object> surfaces_lists_;

    // params 
    double debug_msg_;
    double filter_z_min_, filter_z_max_;
    double dwnsample_size_;
    double plan_iterations_, plan_distance_;
    double table_height_min_, table_height_max_;
    double prism_z_min_, prism_z_max_;
    double cluster_tolerance_, cluster_min_size_, cluster_max_size_;
    int red_min_, red_max_, green_min_, green_max_, blue_min_, blue_max_, yelw_min_, yelw_max_;
    double s_threshold_, v_threshold_;
    std::string filter_z_axis_;
    bool debug_;

};

} // end of fetch_demos_perception namespace

#endif