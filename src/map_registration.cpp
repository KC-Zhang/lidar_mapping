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
#include <pcl/filters/passthrough.h>

class PointProcessor
{
    public:
        //state variables
        sensor_msgs::PointCloud2 prevCloudMsg;
        sensor_msgs::PointCloud2 pairwiseCloudMsg;
        ros::Publisher pub;
        ros::Publisher pairwise_pub;
        tf::TransformBroadcaster tf_broadcast;
        pcl::PointCloud<pcl::PointXYZI>::Ptr newPointTCloud, prevPointTCloud, globalPointTCloud, zFiltered;
        bool first_msg;
        bool newPointCloudReceived;
        Eigen::Matrix4f priorNewToGlobal;
        void lidarCallback(const sensor_msgs::PointCloud2Ptr& msg);
        void registerNewFrame();
        void registerPair(pcl::PointCloud<pcl::PointXYZI>::Ptr newTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr globalCloud, Eigen::Matrix4f targetToSource);
        void downsampleTCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& inAndOutCloud);
        void addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);
        void performIcpWNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud,
                                               pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud,
                                               Eigen::Matrix4f &priorSToT,
                                               Eigen::Matrix4f &outTransformatio);
        void performIcp(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud,
                                        Eigen::Matrix4f &outTransformation);
        void plottf(Eigen::Matrix4f h, std::string parentName, std::string childName);
        void passthroughZFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudFiltered);


        pcl::PointCloud<pcl::PointXYZI>::Ptr downSampleCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn);

        PointProcessor(){
            globalPointTCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            newPointTCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            prevPointTCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            zFiltered = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

            newPointCloudReceived = false;
            priorNewToGlobal = Eigen::Matrix4f::Identity ();
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
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*tempFiltered);
    *inAndOutCloud=*tempFiltered;
}

void PointProcessor::passthroughZFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudFiltered)
{    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (inCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits (-0.15, 5.0);
    pass.filter (*cloudFiltered);
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
                                  Eigen::Matrix4f tToS)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempOutput (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud (*newTCloud, *tempOutput, tToS);
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
    normalEstimator.setRadiusSearch (0.25);
    normalEstimator.compute (*cloud_with_normals);
    pcl::copyPointCloud(*cloud, *cloud_with_normals);
}


void PointProcessor::performIcpWNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud,
                                       Eigen::Matrix4f &priorSToT,
                                       Eigen::Matrix4f &sToT)
{
    //transform the source cloud by applying prior transform, the source pointcloud should be the smaller one compared to the target for computational speed.
    pcl::PointCloud<pcl::PointXYZI>::Ptr SourceTrans (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*sourceTCloud, *SourceTrans, priorSToT);

    // add normal to the clouds
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr sourceTransNormals ( new pcl::PointCloud<pcl::PointXYZINormal> () );
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr targetTCloudNormals ( new pcl::PointCloud<pcl::PointXYZINormal>());
    addNormal( SourceTrans, sourceTransNormals);
    addNormal( targetTCloud, targetTCloudNormals);

    // setup icp with normal
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
    icp.setTransformationEpsilon (1e-8);
    icp.setMaxCorrespondenceDistance (0.12);   // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    icp.setMaximumIterations (200);
    icp.setInputSource(sourceTransNormals);
    icp.setInputTarget(targetTCloudNormals);

    // perform icp and get incremental transform to align pointcloud
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr icp_result = sourceTransNormals;

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity ();

    // Estimate
    icp.setInputSource (sourceTransNormals);
    icp.align(*icp_result);

    //accumulate transformation between each Iteration
    Ti = icp.getFinalTransformation();

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;

    //compose prior with incremental transform
    sToT = priorSToT*Ti;
    priorSToT = sToT;
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
    //voxel grid downsample
    PointProcessor::downsampleTCloud(newPointTCloud);
    //passthrough filter z value
    passthroughZFilter(newPointTCloud,zFiltered);

    //performIcpWithNormal
    Eigen::Matrix4f newToGlobal = Eigen::Matrix4f::Identity();  //define 2 variables, Ti, targetToSource
    //performIcpWNormals(globalPointTCloud, newPointTCloud, Ti); //void performIcpWNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud, Eigen::Matrix4f &outTransformation)
    performIcpWNormals(zFiltered, globalPointTCloud, priorNewToGlobal, newToGlobal); //icp with prior, the newPointTCloud should be the smaller one compared to the globalPointTCloud for computational speed, as the newPointTCloud will be transformed by prior transform before performing icp

    //pairwise registration
    std::cout<< newToGlobal<< std::endl;
    registerPair(newPointTCloud, globalPointTCloud, newToGlobal);
    plottf(newToGlobal, "velodyne", "map");

    //downsample global pointcloud
    PointProcessor::downsampleTCloud(globalPointTCloud);
}

int main (int argc, char** argv){
	ros::init (argc, argv, "my_pcl_tutorial");
    PointProcessor pointProcessor;
    ros::Publisher registration_pub;
    sensor_msgs::PointCloud2 zFilteredMsg, globalCloudMsg;
    pointProcessor.first_msg = true;

    //ros variables
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Time lastSaveTime=ros::Time::now();
    ros::Duration saveIntervalSec(10);

    //subscribers and publishers
    pointProcessor.pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_downsampled", 1);
    registration_pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_aggregated", 1);
    pointProcessor.pairwise_pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_pairwise", 1);
    ros::Subscriber icpsub=nh.subscribe("velodyne_points", 1, &PointProcessor::lidarCallback, &pointProcessor);
    int count =1;
    while(ros::ok()){
        //check for new sensor message
        ros::spinOnce();
        if (pointProcessor.newPointCloudReceived == false){
            continue;}

        //alignment and registration
        pointProcessor.registerNewFrame();

        //convert to msg
        pcl::toROSMsg(*pointProcessor.zFiltered, zFilteredMsg);
        pointProcessor.pairwise_pub.publish(zFilteredMsg);
        pcl::toROSMsg(*pointProcessor.globalPointTCloud, globalCloudMsg);
        registration_pub.publish(globalCloudMsg);

        std::cout<<globalCloudMsg.header.stamp<<"pointcloud time"<<std::endl;

        //save pcds
//        ros::Duration duration = ros::Time::now() - lastSaveTime;
//        bool saveOrNot = (duration > saveIntervalSec);
        count +=1;
        if (count>10){
            pcl::io::savePCDFileASCII ("/home/kaicheng/pcds/mc202.pcd", *pointProcessor.globalPointTCloud);
            std::cerr << "Saved " << pointProcessor.globalPointTCloud->points.size () << " data points to mc202.pcd." << std::endl;
//            lastSaveTime = ros::Time::now();
            count=1;
        }

        //finishing the loop
        pointProcessor.newPointCloudReceived = false;
        loop_rate.sleep();                           
        //ros::spin();
    }
}
