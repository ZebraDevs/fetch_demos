#include <string>
#include <math.h>
#include <Eigen/Eigen>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <simple_grasping/shape_extraction.h>
#include <pcl/io/pcd_io.h>
#include "ros/console.h"                       
#include <fetch_demos_common/shape_extractor.h>
#include <ctime>

namespace shape_extractor{

void 
projectPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr object,
                  const pcl::ModelCoefficients::Ptr coeff, 
                  std::string obj_name)
{
    Eigen::Matrix3f transform_matrix;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr project_object(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::ProjectInliers<pcl::PointXYZRGB> projection;
    projection.setModelType(pcl::SACMODEL_PLANE);
    projection.setInputCloud(object); 
    ROS_INFO("input in projection, coeff %i", coeff->values.size());

    projection.setModelCoefficients(coeff);
    projection.filter(*project_object);
    ROS_INFO("input in projection, projected cloud %i", project_object->width);

    pcl::io::savePCDFileASCII ("/home/ijwo/ros/melodic/fetch-sim/shape" + obj_name + ".pcd", *project_object);

}

void
collectPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr object,
                  std::string obj_name)
{
    // std::time_t result = std::time(nullptr);
    std::time_t rawtime;
    struct tm * timeinfo;
    char buffer[25];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%d-%m_%H_%M_%S",timeinfo);
    std::string str(buffer);
    pcl::io::savePCDFileASCII ("/home/ijwo/ros/melodic/fetch-sim/shape" + std::string(buffer) + obj_name + ".pcd", *object);

}


}