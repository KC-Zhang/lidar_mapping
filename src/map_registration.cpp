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
        sensor_msgs::PointCloud2 prevCloudMsg;
        sensor_msgs::PointCloud2 pairwiseCloudMsg;
        ros::Publisher pub;
        ros::Publisher pairwise_pub;
        Eigen::Matrix4f globalTargetToSource;
        pcl::PointCloud<pcl::PointXYZI>::Ptr newPointTCloud, globalPointTCloud;
        bool first_msg;
        bool newPointCloudReceived;
        void lidarCallback(const sensor_msgs::PointCloud2Ptr& msg);
//        void downSampleCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
        void registerNewFrame();
        void registerPair(pcl::PointCloud<pcl::PointXYZI>::Ptr newTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr globalCloud, Eigen::Matrix4f targetToSource);
        void performIcp(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud, Eigen::Matrix4f &outTransformation);
        void downsampleTCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& inAndOutCloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr downSampleCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn);

        PointProcessor(){
            globalPointTCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            globalTargetToSource = Eigen::Matrix4f::Identity ();
            newPointTCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            newPointCloudReceived = false;
        }

        ~PointProcessor(){

        }
};
////callbacks

void PointProcessor::lidarCallback(const sensor_msgs::PointCloud2Ptr& msg)
{
    //initial pointcloud
    if (first_msg == true) {
//        prevCloudMsg = *msg;
        pcl::fromROSMsg(*msg, *globalPointTCloud);
        PointProcessor::downsampleTCloud(globalPointTCloud);  //downsample
        first_msg = false;
        return;
    }

    if ((msg->header.stamp - prevCloudMsg.header.stamp).toSec() < 0.5) {
        return;
    }

    //    pcl::PointCloud<pcl::PointXYZI>::Ptr prevPointTCloud(new pcl::PointCloud<pcl::PointXYZI>);
    //    pcl::fromROSMsg(prevCloudMsg, *prevPointTCloud); //void 	fromROSMsg (const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud< T > &pcl_cloud)

    pcl::fromROSMsg(*msg, *newPointTCloud);
    newPointCloudReceived = true;

//    prevCloudMsg = *msg;
}

void PointProcessor::registerNewFrame()
{
    //downsample incoming pointcloud and return to the same variable
    PointProcessor::downsampleTCloud(newPointTCloud);

    //icp
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), targetToSource;  //define 2 variables, Ti, targetToSource
    performIcp(globalPointTCloud, newPointTCloud, Ti);
    //performIcpWithNormal

    //pairwise registration
    targetToSource = Ti.inverse();    //get transformation from target to source
    registerPair(newPointTCloud, globalPointTCloud, targetToSource);

    //downsample global pointcloud
    PointProcessor::downsampleTCloud(globalPointTCloud);

}
void PointProcessor::registerPair(pcl::PointCloud<pcl::PointXYZI>::Ptr newTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr globalCloud, Eigen::Matrix4f targetToSource)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempOutput (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud (*newTCloud, *tempOutput, targetToSource);
    *globalCloud += *tempOutput;
}

void PointProcessor::performIcp(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud, Eigen::Matrix4f &outTransformation)
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

void PointProcessor::downsampleTCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &inAndOutCloud)
{    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempFiltered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (inAndOutCloud);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*tempFiltered);
    *inAndOutCloud=*tempFiltered;
}

int main (int argc, char** argv){
	ros::init (argc, argv, "my_pcl_tutorial");
    PointProcessor pointProcessor;
    ros::Publisher registration_pub;
    sensor_msgs::PointCloud2 globalCloudMsg;
    pointProcessor.first_msg = true;

    //ros variables
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    //subscribers and publishers
    pointProcessor.pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_downsampled", 1);
    registration_pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_aggregated", 1);
    pointProcessor.pairwise_pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_pairwise", 1);
//    ros::Subscriber sub=nh.subscribe("velodyne_points", 1, &PointProcessor::downSampleCallback, &pointProcessor);
    ros::Subscriber icpsub=nh.subscribe("velodyne_points", 1, &PointProcessor::lidarCallback, &pointProcessor);


    while(ros::ok()){
        ros::spinOnce();
        if (pointProcessor.newPointCloudReceived == false){
            continue;}
        pointProcessor.registerNewFrame();
        pcl::toROSMsg(*pointProcessor.globalPointTCloud, globalCloudMsg);
        registration_pub.publish(globalCloudMsg);
        pointProcessor.newPointCloudReceived = false;
        loop_rate.sleep();
        //ros::spin();
    }
}
