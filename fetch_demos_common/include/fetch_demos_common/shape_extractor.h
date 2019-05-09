#ifndef SHAPE_EXTRACTOR_H
#define SHAPE_EXTRACTOR_H

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>


namespace shape_extractor{
 
void projectPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr object,
                       const pcl::ModelCoefficients::Ptr coeff,
                       std::string obj_name);


}// end of namespace
#endif
