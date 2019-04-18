#include <fetch_demos_common/perception_clustering.h>
#include <geometry_msgs/TransformStamped.h>    
#include <control_msgs/PointHeadGoal.h>        
#include <pcl/ModelCoefficients.h>             
#include <pcl/PCLPointCloud2.h>                
#include <pcl/PointIndices.h>                  
#include <pcl/filters/passthrough.h>           
#include <pcl/filters/voxel_grid.h>            
#include <pcl/io/pcd_io.h>                     
#include <pcl/segmentation/extract_clusters.h> 
#include <pcl/segmentation/sac_segmentation.h> 
#include <ros/duration.h>                      
#include <tf/LinearMath/Transform.h>           
#include <tf2/exceptions.h>                    
#include "ros/console.h"                       
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types_conversion.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "ApproxMVBB/ComputeApproxMVBB.hpp"

namespace fetch_demos_perception
{

PerceptionClustering::PerceptionClustering(ros::NodeHandle& nh): nh_(nh), debug_(true)
{
    plane_pcPtr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    objects_pcPtr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    obstacles_pcPtr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    input_pcPtr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    input_sum_pcPtr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    nh_.param("perception/debug_msg", debug_msg_, 0.165);
    tf_Listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
    head_action_.reset(new actionlib::SimpleActionClient<control_msgs::PointHeadAction>("head_controller/point_head",
                                                                                        true));
    ROS_INFO("Waiting for action server to start.");
    head_action_->waitForServer();
    ROS_INFO("Action server started.");
    head_camera_cloud_sub_ = nh_.subscribe("/head_camera/depth_registered/points",
                                            1,
                                            &PerceptionClustering::cb,
                                            this);
    if(debug_)
    {
        debug_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("debug_pointcloud", 1);
        plane_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("plane_pointcloud", 1);
    }
    obstacle_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacle_pointcloud", 1);
    objects_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("objects_pointcloud", 1);
    sum_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sum_pointcloud", 1);
    
    nh_.getParam("filter/z_min", filter_z_min_); 
    nh_.getParam("filter/z_max", filter_z_max_); 
    nh_.getParam("filter/axis", filter_z_axis_);
    nh_.getParam("filter/downsample_leaf_size", dwnsample_size_); 
    nh_.getParam("plane_removal/max_iterations", plan_iterations_); 
    nh_.getParam("plane_removal/distance_threshold", plan_distance_); 
    nh_.getParam("plane_removal/table_height_min", table_height_min_); 
    nh_.getParam("plane_removal/table_height_max", table_height_max_); 
    nh_.getParam("plane_removal/prism_z_min", prism_z_min_); 
    nh_.getParam("plane_removal/prism_z_max", prism_z_max_); 
    nh_.getParam("euclidean_cluster/cluster_tolerance", cluster_tolerance_); 
    nh_.getParam("euclidean_cluster/min_size", cluster_min_size_); 
    nh_.getParam("euclidean_cluster/max_size", cluster_max_size_); 
    nh_.getParam("color_extract/red_min", red_min_); 
    nh_.getParam("color_extract/red_max", red_max_); 
    nh_.getParam("color_extract/green_min", green_min_); 
    nh_.getParam("color_extract/green_max", green_max_); 
    nh_.getParam("color_extract/yelw_min", yelw_min_); 
    nh_.getParam("color_extract/yelw_max", yelw_max_); 
    nh_.getParam("color_extract/blue_min", blue_min_); 
    nh_.getParam("color_extract/blue_max", blue_max_); 
    nh_.getParam("color_extract/s_threshold", s_threshold_); 
    nh_.getParam("color_extract/v_threshold", v_threshold_); 
    ROS_INFO("in the init %f, %f", filter_z_min_, filter_z_max_);

}


void 
PerceptionClustering::getObjects(fetch_demos_common::GetObjectsResult& result)
{
    plane_pcPtr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    objects_pcPtr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    obstacles_pcPtr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    input_sum_pcPtr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    filtered_pcPtr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    scanScene(input_sum_pcPtr_);
    sum_cloud_pub_.publish(PCLtoPointCloud2msg(input_sum_pcPtr_));

    pc2_filtering(input_sum_pcPtr_, filtered_pcPtr_);
    debug_cloud_pub_.publish(PCLtoPointCloud2msg(filtered_pcPtr_));
    planeRemoval(filtered_pcPtr_);
    // ros::Duration(4.0).sleep();
    
    std::vector<grasping_msgs::Object> cluster_Objects_msgs = euclideanCluster(objects_pcPtr_);
    for (auto& iter : cluster_Objects_msgs)
    {
        result.objects.push_back(iter);
    }
    if(debug_)
    {
        ROS_INFO("in action server callback");
    }
    if (surfaces_lists_.size() > 0)
    {
       for (auto& iter : surfaces_lists_)
       {
        result.surfaces.push_back(iter);
       }
    }

}

void 
PerceptionClustering::cb(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    input_pcPtr_ = pointCloud2msgtoPCL(cloud);
    // std::cout << input_pcPtr_->points.size() << std::endl;
}

void 
PerceptionClustering::pc2_filtering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& incloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outcloud_ptr)
{
  
  // long-range filter only the point cloud within the limit will remain
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PassThrough<pcl::PointXYZRGB> range_filter;
  range_filter.setFilterFieldName(filter_z_axis_); 
  
  range_filter.setFilterLimits(filter_z_min_, filter_z_max_);
  range_filter.setInputCloud(incloud_ptr);
  range_filter.filter(*incloud_ptr);
  ROS_INFO("in the pc2_filtering %f, %f", 0.0, 2.5);

  // downsample the pointcloud
  pcl::VoxelGrid<pcl::PointXYZRGB> downsampler;
  downsampler.setInputCloud(incloud_ptr);
  downsampler.setLeafSize(dwnsample_size_, dwnsample_size_, dwnsample_size_);
  downsampler.filter(*outcloud_ptr);

}


// from http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction
// return the pointcloud with planes removed
void
PerceptionClustering::planeRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inpcPtr)
{
  ROS_INFO("in the plane removal");

  // RANSAC segmentation for plane removal
  // ROS_DEBUG("the size of the input pointcloud %i", inpcPtr->points.size());
  pcl::PointIndices::Ptr plan_inliers(new pcl::PointIndices);  
  pcl::PointIndices::Ptr obstacle_inliers(new pcl::PointIndices);  
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE); 
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(plan_iterations_);
  seg.setDistanceThreshold(plan_distance_); 
  
  // use a while loop to extract the planes
  int inpt_original_size = inpcPtr->points.size();
  while(inpcPtr->points.size() > 0.1 * inpt_original_size){
    ROS_INFO("inpt_original_size %i", inpt_original_size);
    seg.setInputCloud(inpcPtr);
    seg.segment(*plan_inliers, *coefficients);
    if(plan_inliers->indices.size() == 0){
      ROS_INFO("there is no plane in the provided PointCloud");
      ROS_INFO("out the plane removal");

      return;
    }

    // plan_inliers are the indicies for plans
    // extract the plane from the input PointCloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_plane; 
    extract_plane.setInputCloud(inpcPtr);
    extract_plane.setIndices(plan_inliers);
    // get the PointCloud of plane
    extract_plane.setNegative(false);
    extract_plane.filter(*plane_pcPtr_);
    // extract the rest
    extract_plane.setNegative(true);
    extract_plane.filter(*obstacles_pcPtr_);

    // extract the points above the tabletop 
    // only extract the plan that is roughly the same height as the tabletop, it avoid extract the other plan like the floor
    // get the transform from head to base first

    ROS_INFO("plane_pcPtr_->points[1].z %f", plane_pcPtr_->points[1].z);
    geometry_msgs::PointStamped pt_base;
    transform_frame(pt_base, plane_pcPtr_->points[1].x, plane_pcPtr_->points[1].y, plane_pcPtr_->points[1].z,
                    "base_link", "head_camera_rgb_optical_frame");
    ROS_INFO("pt_base.point.z %f", pt_base.point.z);
    
    if (pt_base.point.z > table_height_min_ && pt_base.point.z < table_height_max_)  // param
    {
      // create the polygon for point segmentation http://docs.pointclouds.org/1.7.1/classpcl_1_1_extract_polygonal_prism_data.html
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::ConvexHull<pcl::PointXYZRGB> hull;
      // TODO: furthur process the pointcloud 
      filter_planpc(plane_pcPtr_, plane_pcPtr_);
      hull.setInputCloud (plane_pcPtr_);
      hull.reconstruct(*hull_points);
      if(hull.getDimension() == 2){
        pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
        prism.setInputCloud(inpcPtr);
        prism.setInputPlanarHull(hull_points);
        prism.setHeightLimits(prism_z_min_, prism_z_max_);
        prism.segment(*plan_inliers);

        pcl::ExtractIndices<pcl::PointXYZRGB> extractor_poly;
        extractor_poly.setInputCloud(inpcPtr);
        extractor_poly.setIndices(plan_inliers);
        extractor_poly.setNegative(false);
        extractor_poly.filter(*objects_pcPtr_);
        extractor_poly.setNegative(true);
        extractor_poly.filter(*obstacles_pcPtr_);
        
        
        prism.setInputCloud(input_sum_pcPtr_);
        prism.setInputPlanarHull(hull_points);
        prism.setHeightLimits(-0.1, prism_z_max_+0.1);
        prism.segment(*obstacle_inliers);

        extractor_poly.setInputCloud(input_sum_pcPtr_);
        extractor_poly.setIndices(obstacle_inliers);
        extractor_poly.setNegative(true);
        extractor_poly.filter(*obstacles_pcPtr_);

        ROS_INFO("obstacles_pcPtr__size %i", obstacles_pcPtr_->points.size());

        grasping_msgs::Object surface_object_msg;
        pcPtr2Objectmsg(plane_pcPtr_, surface_object_msg, "plane");
        surfaces_lists_.push_back(surface_object_msg);

        extractor_poly.setInputCloud(obstacles_pcPtr_);
        extractor_poly.setIndices(plan_inliers);
        extractor_poly.setNegative(true);
        extractor_poly.filter(*obstacles_pcPtr_);

        // param
        // denoise the pbstacle point cloud
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> denoise_filter;
        denoise_filter.setInputCloud (obstacles_pcPtr_);
        denoise_filter.setMeanK (60);
        denoise_filter.setStddevMulThresh (1.0);
        denoise_filter.filter (*obstacles_pcPtr_);

        objects_cloud_pub_.publish(PCLtoPointCloud2msg(objects_pcPtr_));
        plane_cloud_pub_.publish(PCLtoPointCloud2msg(plane_pcPtr_));
        obstacle_cloud_pub_.publish(PCLtoPointCloud2msg(obstacles_pcPtr_));
        
        break;
      }
      else{
        ROS_ERROR("Didn't capture a plane.");
      }
    }

    *inpcPtr = *obstacles_pcPtr_;
  
  }
  ROS_INFO("out the plane removal");

  return;
  
}
// source http://pointclouds.org/documentation/tutorials/cluster_extraction.php
// source https://github.com/jupidity/PCL-ROS-cluster-Segmentation/blob/0634ff23afdf8a95223f58326513e0b1b1884e58/src/segmentation.cpp
std::vector<grasping_msgs::Object> 
PerceptionClustering::euclideanCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcPtr)
{
  ROS_INFO("in the clustering");
  std::vector<grasping_msgs::Object> cluster_result;

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>); 
  tree->setInputCloud(input_pcPtr);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec; 
  ec.setClusterTolerance(cluster_tolerance_); 
  ec.setMinClusterSize(cluster_min_size_); 
  ec.setMaxClusterSize (cluster_max_size_);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_pcPtr);
  ec.extract (cluster_indices);

  // define the msgs for the action server result
  grasping_msgs::Object clusters_object_msg;

  // write all the clustered object into one cloud
  int i = 0;
  pcl::PCDWriter writer;
  for (std::vector<pcl::PointIndices>::const_iterator pt_iter = cluster_indices.begin(); pt_iter != cluster_indices.end(); pt_iter++){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointXYZRGB avgPoint;
    pcl::PointXYZHSV avgPointHSV;
    pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
    // write the points belong to a cluster into the ponintcloud format, also get the center and average color
    for (std::vector<int>::const_iterator ind_iter = pt_iter->indices.begin(); ind_iter != pt_iter->indices.end(); ind_iter++){
      clusterPtr->points.push_back(input_pcPtr->points[*ind_iter]);
      centroid.add(input_pcPtr->points[*ind_iter]);
    }
    centroid.get(avgPoint);
    clusterPtr->width = clusterPtr->points.size();
    clusterPtr->height = 1;
    clusterPtr->is_dense = true;
    pcl::PointXYZRGBtoXYZHSV(avgPoint, avgPointHSV);
    size_t cluster_size = centroid.getSize();
    ROS_INFO("the size of the cluster: %i", clusterPtr->width);
 
    // write the information into an object msg
    grasping_msgs::Object clusters_object_msg;
    std::string obj_name = "object" + std::to_string(i);
    pcPtr2Objectmsg(clusterPtr, clusters_object_msg, obj_name);
    ROS_INFO("num of obj: %i", i);
    
    clusters_object_msg.properties.resize(1);
    clusters_object_msg.properties[0].name = "color";
    clusters_object_msg.properties[0].value = color_extractor(avgPointHSV);
   
    cluster_result.push_back(clusters_object_msg);
    i++;

  }
  ROS_INFO("out the clustering");

  return cluster_result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
PerceptionClustering::pointCloud2msgtoPCL(const sensor_msgs::PointCloud2ConstPtr& cloud2_msg)
{
    pcl::PCLPointCloud2* cloud_pc2 = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloud_pc2Ptr(cloud_pc2);
    pcl_conversions::toPCL(*cloud2_msg, *cloud_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pc_XYZRGBPtr (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromPCLPointCloud2(*cloud_pc2Ptr, *input_pc_XYZRGBPtr);
    return input_pc_XYZRGBPtr;
}

sensor_msgs::PointCloud2
PerceptionClustering::PCLtoPointCloud2msg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_pc, std::string frame)
{
    sensor_msgs::PointCloud2 output_pcmsg;
    pcl::PCLPointCloud2 output_pc2;
    pcl::toPCLPointCloud2(*input_pc, output_pc2);
    pcl_conversions::fromPCL(output_pc2, output_pcmsg);

    output_pcmsg.header.frame_id = frame;
    output_pcmsg.header.stamp = ros::Time::now();
    
    return output_pcmsg;
}

void
PerceptionClustering::transform_frame(geometry_msgs::PointStamped& pt_transformed, double x, double y, double z, std::string target_frame, std::string source_frame)
{
    geometry_msgs::PointStamped pt_input;
    pt_input.header.frame_id = source_frame;
    pt_input.header.stamp = ros::Time::now();
    pt_input.point.x = x;
    pt_input.point.y = y;
    pt_input.point.z = z;
    try{ 
      tf_buffer_.transform( pt_input, pt_transformed, target_frame, ros::Duration(3.0));
    }
    catch(tf2::TransformException ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    return;

}

std::string
PerceptionClustering::color_extractor(pcl::PointXYZHSV& pointHSV)
{
    // red
    if (pointHSV.h > red_min_ || (pointHSV.h < red_max_ && pointHSV.h > 0) 
        && pointHSV.s > s_threshold_ && pointHSV.v > v_threshold_)
    {
        return "red";
    }
    // green
    if (pointHSV.h > green_min_ && pointHSV.h < green_max_ 
        && pointHSV.s > s_threshold_ && pointHSV.v > v_threshold_)
    {
        return "green";
    }
    // yellow
    if (pointHSV.h > yelw_min_ && pointHSV.h < yelw_max_ 
        && pointHSV.s > s_threshold_ && pointHSV.v > v_threshold_)
    {
        return "yellow";
    }
    // blue
    if (pointHSV.h > blue_min_ && pointHSV.h < blue_max_ 
        && pointHSV.s > s_threshold_ && pointHSV.v > v_threshold_)
    {
        return "blue";
    }
    else
    {
        ROS_INFO("the color detected was not one of the four color\n h value: %f, s value: %f, v value: %f", pointHSV.h, pointHSV.s, pointHSV.v);
        return "";
    }
    
}

void 
PerceptionClustering::head_lookat(double x, double y, double z, std::string frame_name)
{
      control_msgs::PointHeadGoal head_goal;
      head_goal.target.header.stamp = ros::Time::now();
      head_goal.target.header.frame_id = frame_name;
      head_goal.target.point.x = x;
      head_goal.target.point.y = y;
      head_goal.target.point.z = z;      
      head_action_->sendGoal(head_goal);
      head_action_->waitForResult();
      ros::Duration(0.5).sleep();

}
geometry_msgs::TransformStamped
PerceptionClustering::lookup_transform(std::string target_frame, std::string source_frame)
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_buffer_.lookupTransform(target_frame,
                                                  source_frame,
                                                  ros::Time::now(), 
                                                  ros::Duration(3.0));
  }
  catch(tf2::TransformException &ex) 
  {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();

  }

  return transformStamped;
}

void
PerceptionClustering::scanScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_pcPtr, int degree_start, int degree_end)
{
  // get the height of the head pose and look at the origin at half of the head height
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sum_pcPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
  geometry_msgs::PointStamped pt_head;
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::TransformStamped initial_transformStamped;
  tf::Transform initial_transform;
  transform_frame(pt_head, 0.0, 0.0, 0.0, "base_link", "head_camera_rgb_optical_frame");
  ROS_INFO("the head position point: %f, %f, %f", pt_head.point.x, pt_head.point.y, pt_head.point.z);
  head_lookat(1.0, 0.0, pt_head.point.z / 2, "base_link");
  ros::Duration(1.0).sleep();
  initial_transformStamped = lookup_transform( "head_camera_rgb_optical_frame", "base_link");
  tf::transformMsgToTF(initial_transformStamped.transform, initial_transform);
  int full_range = degree_end - degree_start;
  int sampling_time = full_range / 30 + 1;

  for (int i = 0; i < sampling_time; i++)
  {
    
    double cur_degree = degree_start + i*30;
    ROS_INFO("current degree %f, point %f, %f, %f ", cur_degree, 1.0, sin( cur_degree * PI / 180.0 ), pt_head.point.z);
    head_lookat(1.0, sin( cur_degree * PI / 180.0 ), pt_head.point.z / 2, "base_link");
    // ros::Duration(6.0).sleep();
    ros::Duration(1.5).sleep();

    transformStamped = lookup_transform("base_link", "head_camera_rgb_optical_frame");
    tf::Transform transform;
    tf::transformMsgToTF(transformStamped.transform, transform);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl_ros::transformPointCloud (*input_pcPtr_, *transformed_cloud, transform);
    // ros::Duration(4.0).sleep();
    ros::Duration(1.0).sleep();
    *sum_pcPtr += *transformed_cloud;
    ros::Duration(1.0).sleep();

  }
  ros::Duration(1.0).sleep();
  head_lookat(1.0, 0.0, pt_head.point.z / 2, "base_link");
  
  pcl_ros::transformPointCloud (*sum_pcPtr, *input_pcPtr, initial_transform);

  ROS_INFO("finish scanning");
  ros::Duration(1.0).sleep();
}

bool 
PerceptionClustering::extractUnorientedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>& input,
                                                   shape_msgs::SolidPrimitive& shape,
                                                   geometry_msgs::Pose& pose)
{
  double x_min = 1000.0;
  double x_max = -1000.0;
  double y_min = 1000.0;
  double y_max = -1000.0;
  double z_min = 1000.0;
  double z_max = -1000.0;

  for (size_t i = 0; i < input.size(); ++i)
  {
    if (input[i].x < x_min)
      x_min = input[i].x;
    if (input[i].x > x_max)
      x_max = input[i].x;

    if (input[i].y < y_min)
      y_min = input[i].y;
    if (input[i].y > y_max)
      y_max = input[i].y;

    if (input[i].z < z_min)
      z_min = input[i].z;
    if (input[i].z > z_max)
      z_max = input[i].z;
  }

  pose.position.x = (x_min + x_max)/2.0;
  pose.position.y = (y_min + y_max)/2.0;
  pose.position.z = (z_min + z_max)/2.0;

  shape.type = shape.BOX;
  shape.dimensions.push_back(x_max-x_min);
  shape.dimensions.push_back(y_max-y_min);
  shape.dimensions.push_back(z_max-z_min);

  return true;
}
void
PerceptionClustering::pcPtr2Objectmsg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_pcPtr,
                                       grasping_msgs::Object& input_object_msg,
                                       std::string name)
{
  // write the plane into the result
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pcPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
  geometry_msgs::TransformStamped transformStamped;
  tf::Transform transform;
  transformStamped = lookup_transform( "base_link", "head_camera_rgb_optical_frame");
  tf::transformMsgToTF(transformStamped.transform, transform);
  pcl_ros::transformPointCloud (*in_pcPtr, *transformed_pcPtr, transform);

  input_object_msg.name = name;
  
  input_object_msg.point_cluster = PCLtoPointCloud2msg(transformed_pcPtr);
  input_object_msg.primitive_poses.resize(1);
  input_object_msg.primitives.resize(1);
  shape_msgs::SolidPrimitive primitive;
  geometry_msgs::Pose pose;
  if (name == "plane")
  {
    extractUnorientedBoundingBox(*transformed_pcPtr, primitive, pose);
  }
  else
  {
    extract_boundingbox(transformed_pcPtr, primitive, pose);
  }
  
  input_object_msg.primitive_poses[0] = pose;
  input_object_msg.primitives[0] = primitive;
  input_object_msg.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  ROS_INFO("%s: ", name);
  ROS_INFO("cluster primitives: %f, %f, %f ",
            input_object_msg.primitives[0].dimensions[0],
            input_object_msg.primitives[0].dimensions[1], 
            input_object_msg.primitives[0].dimensions[2] );
  ROS_INFO("cluster poses: %f, %f, %f ",
            input_object_msg.primitive_poses[0].position.x,
            input_object_msg.primitive_poses[0].position.y, 
            input_object_msg.primitive_poses[0].position.z );

}

void 
PerceptionClustering::filter_planpc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plan_source_pcPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr plan_out_pcPtr)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_plan_pcPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
  double z_min_plan = plan_source_pcPtr->points[0].z;
  double z_max_range = 0.46;
  double z_max_plan = z_min_plan + z_max_range;
  ROS_INFO("point cloud size before filtering: %i", plan_source_pcPtr->size());
  for (int i = 0; i < plan_source_pcPtr->size(); i++)
  {
    if (plan_source_pcPtr->points[i].z < z_max_plan)
    {
      tmp_plan_pcPtr->points.push_back(plan_source_pcPtr->points[i]);
    }
  }
  ROS_INFO("point cloud size after filtering: %i", tmp_plan_pcPtr->size());
  *plan_out_pcPtr = *tmp_plan_pcPtr;

}

void
PerceptionClustering::extract_boundingbox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pcPtr,
                                         shape_msgs::SolidPrimitive& primitive,
                                         geometry_msgs::Pose& pose)
{
  ApproxMVBB::Matrix3Dyn mvbb_points(3, object_pcPtr->points.size());
  for (int i = 0; i < object_pcPtr->points.size(); i++)
  {
    mvbb_points.col(i) << object_pcPtr->points[i].x, object_pcPtr->points[i].y, object_pcPtr->points[i].z;
  }

  ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(mvbb_points, 0.001, object_pcPtr->points.size(), 5, 0, 5);
  // store the position of the object
  float position_x =(oobb.m_maxPoint.x() + oobb.m_minPoint.x()) / 2;
  float position_y =(oobb.m_maxPoint.y() + oobb.m_minPoint.y()) / 2;
  float position_z =(oobb.m_maxPoint.z() + oobb.m_minPoint.z()) / 2;
  Eigen::Vector3f oobb_position(position_x, position_y, position_z);

  Eigen::Quaternionf rot(oobb.m_q_KI.w(), oobb.m_q_KI.x(), oobb.m_q_KI.y(), oobb.m_q_KI.z());
  oobb_position = rot.matrix()*oobb_position;
  pose.position.x = oobb_position.x();
  pose.position.y = oobb_position.y();
  pose.position.z = oobb_position.z();

  // get the orientation of the object
  pose.orientation.x = rot.x();
  pose.orientation.y = rot.y();
  pose.orientation.z = rot.z();
  pose.orientation.w = rot.w();

  // store the width, height and depth of the object
  primitive.dimensions.push_back(oobb.m_maxPoint.x() - oobb.m_minPoint.x());
  primitive.dimensions.push_back(oobb.m_maxPoint.y() - oobb.m_minPoint.y());
  primitive.dimensions.push_back(oobb.m_maxPoint.z() - oobb.m_minPoint.z());
  
  
  ROS_INFO("in exract_boundingbox, max point: %d, %d, %d, %d", oobb.m_maxPoint[0], oobb.m_maxPoint[1], oobb.m_maxPoint[2]);
}

}// end of fetch_demos_perception namespace