#include "ros/ros.h"
#include "iostream"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/registration/icp.h"
#include <pcl/features/normal_3d.h>
#include <tf/transform_broadcaster.h>

class PointProcessor
{
    public:
        //state variables
        sensor_msgs::PointCloud2 prevCloudMsg;
        sensor_msgs::PointCloud2 pairwiseCloudMsg;
        ros::Publisher pub;
        ros::Publisher pairwise_pub;
        tf::TransformBroadcaster tf_broadcast;
        pcl::PointCloud<pcl::PointXYZI>::Ptr newPointTCloud, prevPointTCloud, globalPointTCloud;
        bool first_msg;
        bool newPointCloudReceived;
        Eigen::Matrix4f globalTargetToSource;
        void lidarCallback(const sensor_msgs::PointCloud2Ptr& msg);
        void registerNewFrame();
        void registerPair(pcl::PointCloud<pcl::PointXYZI>::Ptr newTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr globalCloud, Eigen::Matrix4f targetToSource);
        void downsampleTCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& inAndOutCloud);
        void addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);
        void performIcpWNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud,
                                               pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud,
                                               Eigen::Matrix4f &outTransformatio);
        void performIcp(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud,
                                        Eigen::Matrix4f &outTransformation);
        void plottf(Eigen::Matrix4f h, std::string parentName, std::string childName);

        pcl::PointCloud<pcl::PointXYZI>::Ptr downSampleCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn);

        PointProcessor(){
            globalPointTCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            newPointTCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            prevPointTCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            newPointCloudReceived = false;
            globalTargetToSource = Eigen::Matrix4f::Identity ();
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
        pcl::copyPointCloud<pcl::PointXYZI>(*globalPointTCloud, *prevPointTCloud);
        first_msg = false;
        return;
    }

//    if ((msg->header.stamp - prevCloudMsg.header.stamp).toSec() < 0.5) {
//        return;
//    }

    //    pcl::PointCloud<pcl::PointXYZI>::Ptr prevPointTCloud(new pcl::PointCloud<pcl::PointXYZI>);
    //    pcl::fromROSMsg(prevCloudMsg, *prevPointTCloud); //void 	fromROSMsg (const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud< T > &pcl_cloud)

    pcl::fromROSMsg(*msg, *newPointTCloud);
    newPointCloudReceived = true;

//    prevCloudMsg = *msg;
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

void PointProcessor::plottf(Eigen::Matrix4f h, std::string parentName, std::string childName){

  tf::Transform frame;
  frame.setOrigin(tf::Vector3(h(0,3), h(1,3), h(2,3)));
  frame.setBasis(tf::Matrix3x3(h(0,0), h(0,1), h(0,2),
                               h(1,0), h(1,1), h(1,2),
                               h(2,0), h(2,1), h(2,2)));
  ros::Time newTCloudTime;
  pcl_conversions::fromPCL(newPointTCloud->header.stamp, newTCloudTime);
  tf_broadcast.sendTransform(tf::StampedTransform(frame, newTCloudTime, parentName, childName));
}


void PointProcessor::registerPair(pcl::PointCloud<pcl::PointXYZI>::Ptr newTCloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr globalCloud,
                                  Eigen::Matrix4f targetToSource)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempOutput (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud (*newTCloud, *tempOutput, targetToSource);
    *globalCloud += *tempOutput;
}

void PointProcessor::addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                               pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZI>);
    searchTree->setInputCloud (cloud);

    pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> normalEstimator;
    normalEstimator.setInputCloud (cloud);
    normalEstimator.setSearchMethod (searchTree);
    normalEstimator.setKSearch (15);
    normalEstimator.compute (*cloud_with_normals);
    pcl::copyPointCloud(*cloud, *cloud_with_normals);
}


void PointProcessor::performIcpWNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud,
                                       Eigen::Matrix4f &sourceToTarget)
{
    // add normal to the clouds
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr sourceTCloudNormals ( new pcl::PointCloud<pcl::PointXYZINormal> () );
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr targetTCloudNormals ( new pcl::PointCloud<pcl::PointXYZINormal>());
    addNormal( sourceTCloud, sourceTCloudNormals);
    addNormal( targetTCloud, targetTCloudNormals);

    //setup icp with normal
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
    icp.setTransformationEpsilon (1e-6);
    icp.setMaxCorrespondenceDistance (0.3);   // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    icp.setMaximumIterations (1);
    icp.setInputSource(sourceTCloudNormals);
    icp.setInputTarget(targetTCloudNormals);

    //perform icp and align pointcloud
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr icp_result = sourceTCloudNormals;

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;

    for (int i = 0; i <30; ++i){
        PCL_INFO("iteration Nr. %d. \n", i);

        sourceTCloudNormals=icp_result;
        // Estimate
        icp.setInputSource (sourceTCloudNormals);
        icp.align(*icp_result);

        //accumulate transformation between each Iteration
        Ti = icp.getFinalTransformation() * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((icp.getLastIncrementalTransformation()-prev).sum()) <icp.getTransformationEpsilon())
            icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance()-0.001);
        prev=icp.getLastIncrementalTransformation();
        //???????????need to be implement : wait for converge

        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    }

    sourceToTarget = Ti;
        //get and save the transform
}

void PointProcessor::performIcp(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud,
                                Eigen::Matrix4f &outTransformation)
{
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(sourceTCloud);
    icp.setInputTarget(targetTCloud);
    pcl::PointCloud<pcl::PointXYZI> Final;
    icp.align(Final);
    outTransformation = icp.getFinalTransformation();
}
void PointProcessor::registerNewFrame()
{
    //downsample incoming pointcloud and return to the same variable
    PointProcessor::downsampleTCloud(newPointTCloud);

    //performIcpWithNormal
    Eigen::Matrix4f sourceToTarget = Eigen::Matrix4f::Identity(), targetToSource;  //define 2 variables, Ti, targetToSource
    //performIcpWNormals(globalPointTCloud, newPointTCloud, Ti); //void performIcpWNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud, Eigen::Matrix4f &outTransformation)
    performIcpWNormals(prevPointTCloud, newPointTCloud, sourceToTarget);

    //pairwise registration
    targetToSource = sourceToTarget.inverse();    //get transformation from target to source
    globalTargetToSource = targetToSource*globalTargetToSource;
    std::cout<< globalTargetToSource<< std::endl;
    registerPair(newPointTCloud, globalPointTCloud, globalTargetToSource);
    plottf(globalTargetToSource, "velodyne", "map");

    //downsample global pointcloud
    PointProcessor::downsampleTCloud(globalPointTCloud);

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
    ros::Subscriber icpsub=nh.subscribe("velodyne_points", 1, &PointProcessor::lidarCallback, &pointProcessor);


    while(ros::ok()){
        //check for new sensor message
        ros::spinOnce();
        if (pointProcessor.newPointCloudReceived == false){
            continue;}

        //alignment and registration
        pointProcessor.registerNewFrame();

        //convert to msg
        pcl::toROSMsg(*pointProcessor.globalPointTCloud, globalCloudMsg);
        registration_pub.publish(globalCloudMsg);
        std::cout<<globalCloudMsg.header.stamp<<"pointcloud time"<<std::endl;

        //finishing the loop
        pointProcessor.newPointCloudReceived = false;
        loop_rate.sleep();
        //ros::spin();
    }
}
