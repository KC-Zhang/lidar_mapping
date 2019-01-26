#include "ros/ros.h"
#include "iostream"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/registration/icp.h"

ros::Publisher pub;
ros::Publisher registration_pub;
bool first_msg = true;
sensor_msgs::PointCloud2 prev_cloud;

void icpCallback(const sensor_msgs::PointCloud2Ptr& msg)
{
    //initial pointcloud
    if (first_msg == true) {
        prev_cloud = *msg;
        first_msg = false;
        return;
    }

    if ((msg->header.stamp - prev_cloud.header.stamp).toSec() < 0.5) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr prev_PointTCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(prev_cloud, *prev_PointTCloud); //void 	fromROSMsg (const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud< T > &pcl_cloud)

    pcl::PointCloud<pcl::PointXYZI>::Ptr PointTCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *PointTCloud);

    //icp
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(prev_PointTCloud);
    icp.setInputTarget(PointTCloud);
    pcl::PointCloud<pcl::PointXYZI> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
        //get and save the transform
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), targetToSource;  //define 2 variables, Ti, targetToSource
    Ti = icp.getFinalTransformation() * Ti;
    std::cout << "getFinalTransformation \n"<< icp.getFinalTransformation() << std::endl;
    //get transformation from target to source
    targetToSource = Ti.inverse();
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud (*PointTCloud, *temp, targetToSource);
    *temp += *prev_PointTCloud;
    sensor_msgs::PointCloud2 aggregated_cloud;
    pcl::toROSMsg(*temp, aggregated_cloud);
    registration_pub.publish (aggregated_cloud);
    prev_cloud=*msg;
}

void downSampleCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
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
	ros::Subscriber sub=nh.subscribe("velodyne_points", 1, downSampleCallback);
	pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_downsampled", 1);
    registration_pub=nh.advertise<sensor_msgs::PointCloud2> ("pairwise_registration_pointclouds", 1);
    ros::Subscriber icpsub=nh.subscribe("velodyne_points_downsampled", 1, icpCallback);
	//ros::spinOnce();
	//loop_rate.sleep();
	ros::spin();

}
