#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


int main (int argc, char** argv)
{
    ros::init(argc, argv, "jaguar_pose_tf_listener");
    ros::NodeHandle node;
    tf::TransformListener listener;
    ros::Rate rate(10.0);
    while (true){
      tf::StampedTransform transform;
      try{
        listener.lookupTransform("/map", "/velodyne",
                                 ros::Time(0), transform);
        std::cout<<"tf stamp: " << transform.stamp_<<std::endl;
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("icp_tut.cpp %s",ex.what());
        ros::Duration(0.01).sleep();
        continue;
      }
    }
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

//  // Fill in the CloudIn data
//  cloud_in->width    = 5;
//  cloud_in->height   = 1;
//  cloud_in->is_dense = false;
//  cloud_in->points.resize (cloud_in->width * cloud_in->height);
//  for (size_t i = 0; i < cloud_in->points.size (); ++i)
//  {
//    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//  }
//  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
//      << std::endl;
//  for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
//      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
//      cloud_in->points[i].z << std::endl;
//  *cloud_out = *cloud_in;
//  std::cout << "size:" << cloud_out->points.size() << std::endl;
//  for (size_t i = 0; i < cloud_in->points.size (); ++i)
//    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
//  std::cout << "Transformed " << cloud_in->points.size () << " data points:"
//      << std::endl;
//  for (size_t i = 0; i < cloud_out->points.size (); ++i)
//    std::cout << "    " << cloud_out->points[i].x << " " <<
//      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
//  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//  icp.setInputSource(cloud_in);
//  icp.setInputTarget(cloud_out);
//  pcl::PointCloud<pcl::PointXYZ> Final;
//  icp.align(Final);
//  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//  icp.getFitnessScore() << std::endl;
//  std::cout << icp.getFinalTransformation() << std::endl;

 return (0);
}
