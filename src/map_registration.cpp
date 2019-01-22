#include "ros/ros.h"
#include "iostream"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/registration/icp.h"

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PCLPointCloud2* prev_cloud_filtered_global = new pcl::PCLPointCloud2;

// void perform_icp(pcl::PCLPointCloud2ConstPtr& prev_cloud_filtered, pcl::PCLPointCloud2ConstPtr& cloud_filtered)
// {
//   pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
//   icp.setInputSource(prev_cloud_filtered);
//   icp.setInputTarget(cloud_filtered);
//   pcl::PointCloud<pcl::PointXYZI> Final;
//   icp.align(Final);
//   std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//   icp.getFitnessScore() << std::endl;
//   std::cout << icp.getFinalTransformation() << std::endl;
// }

void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	//Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	//Perform the actual filtering 
	//std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
	//  << " data points (" << pcl::getFieldsList (*cloud) << ").";

	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.1f, 0.1f, 0.1f);
	sor.filter (cloud_filtered);
	//std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
	//     << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
	sensor_msgs::PointCloud2 output_cloud;
	pcl_conversions::fromPCL(cloud_filtered,output_cloud); //not pointer here?

	pub.publish (output_cloud);

}

int main (int argc, char** argv){
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	ros::Subscriber sub=nh.subscribe("velodyne_points", 1, pointcloudCallback);
	pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_downsampled", 1);
	//ros::spinOnce();
	//loop_rate.sleep();
	ros::spin();

}