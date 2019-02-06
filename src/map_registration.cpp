#include "ros/ros.h"
#include "iostream"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/registration/icp.h"

class PointProcessor
{
    public:
        //state variables
        sensor_msgs::PointCloud2 lastCloudMsg;
        sensor_msgs::PointCloud2 pairwiseCloudMsg;
        sensor_msgs::PointCloud2 globalCloudMsg;
        ros::Publisher pub;
        ros::Publisher pairwise_pub;
        ros::Publisher registration_pub;
        Eigen::Matrix4f globalTargetToSource;
        pcl::PointCloud<pcl::PointXYZI>::Ptr globalPointTCloud;
        bool first_msg;
        void icpCallback(const sensor_msgs::PointCloud2Ptr& msg);
        void downSampleCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
        void registerPair(pcl::PointCloud<pcl::PointXYZI>::Ptr newTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr globalCloud, Eigen::Matrix4f targetToSource);
        void performIcp(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud, Eigen::Matrix4f outTransformation);
        pcl::PointCloud<pcl::PointXYZI>::Ptr downSampleCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn);

        PointProcessor(){
            globalPointTCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            globalTargetToSource = Eigen::Matrix4f::Identity ();
        }

        ~PointProcessor(){

        }
};
//callbacks
void PointProcessor::downSampleCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
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

pcl::PointCloud<pcl::PointXYZI>::Ptr PointProcessor::downSampleCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn){
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (cloudIn);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZI>);
    sor.filter (*filteredCloud);
    return filteredCloud;
}

void PointProcessor::registerPair(pcl::PointCloud<pcl::PointXYZI>::Ptr newTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr globalCloud, Eigen::Matrix4f targetToSource)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempIcpOutput (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud (*newTCloud, *tempIcpOutput, targetToSource);
    *tempIcpOutput += *globalCloud;
}

void PointProcessor::performIcp(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud, Eigen::Matrix4f outTransformation)
{
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(sourceTCloud);
    icp.setInputTarget(targetTCloud);
    pcl::PointCloud<pcl::PointXYZI> outputTransformedSource;
    icp.align(outputTransformedSource);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
        //get and save the transform
    outTransformation = icp.getFinalTransformation() * outTransformation;
    std::cout << "getFinalTransformation \n"<< icp.getFinalTransformation() << std::endl;
}
void PointProcessor::icpCallback(const sensor_msgs::PointCloud2Ptr& msg)
{
    //initial pointcloud
    if (first_msg == true) {
        lastCloudMsg = *msg;
        pcl::fromROSMsg(*msg, *globalPointTCloud);
        first_msg = false;
        return;
    }

    if ((msg->header.stamp - lastCloudMsg.header.stamp).toSec() < 0.2) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr prevPointTCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(lastCloudMsg, *prevPointTCloud); //void 	fromROSMsg (const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud< T > &pcl_cloud)

    pcl::PointCloud<pcl::PointXYZI>::Ptr PointTCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *PointTCloud);

    //icp
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), targetToSource;  //define 2 variables, Ti, targetToSource
    performIcp(prevPointTCloud, PointTCloud, Ti);

    //pairwise registration
    targetToSource = Ti.inverse();    //get transformation from target to source
    registerPair(PointTCloud, prevPointTCloud, targetToSource);

    //global registration
    globalTargetToSource = globalTargetToSource*targetToSource;
    registerPair(PointTCloud, globalPointTCloud, globalTargetToSource);

    //publish ROStopics
    pcl::toROSMsg(*prevPointTCloud, pairwiseCloudMsg);
    pairwise_pub.publish(pairwiseCloudMsg);
    pcl::toROSMsg(*globalPointTCloud, globalCloudMsg);
    registration_pub.publish(globalCloudMsg);

    lastCloudMsg = *msg;
}


int main (int argc, char** argv){
	ros::init (argc, argv, "my_pcl_tutorial");
    PointProcessor pointProcessor;
    pointProcessor.first_msg = true;

    //ros variables
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);


    //subscribers and publishers
    pointProcessor.pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_downsampled", 1);
    pointProcessor.registration_pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_aggregated", 1);
    pointProcessor.pairwise_pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_pairwise", 1);
    ros::Subscriber sub=nh.subscribe("velodyne_points", 1, &PointProcessor::downSampleCallback, &pointProcessor);
    ros::Subscriber icpsub=nh.subscribe("velodyne_points_downsampled", 1, &PointProcessor::icpCallback, &pointProcessor);


    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        //ros::spin();
    }
}
