#include "ros/ros.h"
#include "iostream"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/passthrough.h>
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/registration/icp.h"
#include <pcl/features/normal_3d.h>
#include <tf/transform_broadcaster.h>
#include <list>
#include <thread>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>        // std::abs
#include <cstdio>
#include <ctime>




class PointProcessor
{
    public:
        //state variables
        sensor_msgs::PointCloud2 prevCloudMsg;
        sensor_msgs::PointCloud2 pairwiseCloudMsg;
        ros::Publisher pub;
        ros::Publisher pairwise_pub;
        tf::TransformBroadcaster tf_broadcast;
        pcl::PointCloud<pcl::PointXYZI>::Ptr newPointTCloud, prevPointTCloud, initialPointTCloud, globalVisualizationPointTCloud,zFiltered;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr globalPointTCloudNormal;

        std::list<sensor_msgs::PointCloud2> newPointCloud2List;
        std::list<Eigen::Matrix4f> tfEigenOdomToVelodyneList;
        Eigen::Matrix4f prevTfEigenOdomToVelodyne, newTfEigenOdomToVelodyne, prevCallbackTfEigenOdomToVelodyne, prevNewToGlobal;
        int callbackSkipFrameCounter, pointRegistrationCounter;

        bool first_msg, globalCloudsUninitialized;
        void getNextPointCloudAndTf();
        int getTransformation(const std::string baseFrame, const std::string targetFrame, const ros::Time timeStamp, tf::StampedTransform &tf);
        void lidarCallback(const sensor_msgs::PointCloud2Ptr& msg);
        void registerNewFrame();
        void registerPair(pcl::PointCloud<pcl::PointXYZI>::Ptr newTCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr globalCloud, Eigen::Matrix4f targetToSource);
        void registerPairNormal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr newTCloud, pcl::PointCloud<pcl::PointXYZINormal>::Ptr globalCloud, Eigen::Matrix4f targetToSource);
        void downsampleTCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& inAndOutCloud);
        void downsampleTCloudN(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &inAndOutCloud);
        void addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_with_normals);
        void updateNormal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &inAndOutCloud);
        void performIcpWNormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr sourceTCloud,
                                               pcl::PointCloud<pcl::PointXYZINormal>::Ptr targetTCloud,
                                               Eigen::Matrix4f odomPriorNewToPrev,
                                               Eigen::Matrix4f &prevSToT,
                                               Eigen::Matrix4f &outTransformatio);
        void performIcp(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceTCloud,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr targetTCloud,
                                        Eigen::Matrix4f &outTransformation);
        void plottf(Eigen::Matrix4f h, std::string parentName, std::string childName);
        void passthroughFilterZ(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudFiltered);
        void passthroughFilterI(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudFiltered);
        void passthroughFilterINormal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &inAndOutCloud);
        void getTransformationMagnitude(const Eigen::Matrix4f transformation, float &principleAxisRotation, float &linearL1Norm);


        tf::TransformListener tfListener;

        pcl::PointCloud<pcl::PointXYZI>::Ptr downSampleCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn);


        PointProcessor(){
            initialPointTCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            globalVisualizationPointTCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            globalPointTCloudNormal = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
            newPointTCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            prevNewToGlobal = Eigen::Matrix4f::Identity();
            zFiltered = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            prevTfEigenOdomToVelodyne = Eigen::Matrix4f::Identity();
            callbackSkipFrameCounter = 0;
            pointRegistrationCounter =0;
            globalCloudsUninitialized=true;
        }

        ~PointProcessor(){

        }
};

void PointProcessor::getNextPointCloudAndTf(){
    if (globalCloudsUninitialized){
        pcl::copyPointCloud<pcl::PointXYZI>(*initialPointTCloud, *globalVisualizationPointTCloud);
        addNormal(initialPointTCloud,globalPointTCloudNormal);
        globalCloudsUninitialized=false;
    }
    prevTfEigenOdomToVelodyne=newTfEigenOdomToVelodyne;
    pcl::fromROSMsg(newPointCloud2List.front(), *newPointTCloud);
    newPointCloud2List.pop_front();
    //get tf eigen
    newTfEigenOdomToVelodyne=tfEigenOdomToVelodyneList.front();
    tfEigenOdomToVelodyneList.pop_front();

    std::cout<<"new pointcloud processed, remaining list size is: "<<newPointCloud2List.size()<<std::endl;
}
////callbacks
int PointProcessor::getTransformation(const std::string sourceFrame, const std::string targetFrame, const ros::Time timeStamp, tf::StampedTransform &tf){
    try{
        tfListener.waitForTransform(sourceFrame,targetFrame, timeStamp,ros::Duration(3.0));
        tfListener.lookupTransform(sourceFrame, targetFrame, timeStamp, tf);
        return 1;
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("map_registration.cpp %s",ex.what());
        ros::Duration(0.01).sleep();
        return 0;
    }
}
void PointProcessor::getTransformationMagnitude(const Eigen::Matrix4f transformation, float &principleAxisRotation, float &linearL1Norm){
    //check rotation
    Eigen::Matrix3f rotationNewToPrev;
    rotationNewToPrev=transformation.block<3,3>(0,0); //convert f to double and get the top left 3x3 matrix as the rotation matrix, double is needed for eigen::angleaxisd
    Eigen::AngleAxisf rotationAngleAxis(rotationNewToPrev);
    principleAxisRotation=rotationAngleAxis.angle();
    //check linear
    linearL1Norm = std::abs(transformation(0,3)) + std::abs(transformation(1,3)) + std::abs(transformation(2,3)); //get l1 norm of linear movement
}

void PointProcessor::lidarCallback(const sensor_msgs::PointCloud2Ptr& msg)
{
    //initial pointcloud
    if (first_msg == true) {
        tf::StampedTransform tfOdomToVelodyne;
        bool success = getTransformation("zed_odom", "velodyne", msg->header.stamp, tfOdomToVelodyne);
        if (success = 0){
            std::cout<<"lidarCallback: fail to get tf"<<std::endl;
            return;
        }
        //get pointcloud and convert to pcl
        prevCloudMsg = *msg;
        globalCloudsUninitialized = true;
        pcl::fromROSMsg(*msg, *initialPointTCloud);
        //get tf and convert to eigen
        pcl_ros::transformAsMatrix(tfOdomToVelodyne, newTfEigenOdomToVelodyne);
        prevCallbackTfEigenOdomToVelodyne=newTfEigenOdomToVelodyne;
        //finishing if
        first_msg = false;
        return;
    }

    //get tf and convert to eigen
    tf::StampedTransform tfOdomToVelodyne;
    Eigen::Matrix4f tfCallbackEigenOdomToVelodyne, odomPriorNewToPrev;
    getTransformation("zed_odom", "velodyne", msg->header.stamp, tfOdomToVelodyne);
    pcl_ros::transformAsMatrix(tfOdomToVelodyne, tfCallbackEigenOdomToVelodyne);
    odomPriorNewToPrev = tfCallbackEigenOdomToVelodyne.inverse()*prevCallbackTfEigenOdomToVelodyne;

    //get angular and linear motion
    float rotationMagnitude, linearL1Norm;
    getTransformationMagnitude(odomPriorNewToPrev, rotationMagnitude, linearL1Norm);

    if (linearL1Norm > 1) {
        //get pointcloud and convert to pcl
        prevCloudMsg = *msg;
        globalCloudsUninitialized = true;
        pcl::fromROSMsg(*msg, *initialPointTCloud);
        //get tf and convert to eigen
        newTfEigenOdomToVelodyne=tfCallbackEigenOdomToVelodyne;
        prevCallbackTfEigenOdomToVelodyne=newTfEigenOdomToVelodyne;
        return;
    }
    if ((msg->header.stamp - prevCloudMsg.header.stamp).toSec() < 5 && rotationMagnitude < 0.001 && linearL1Norm < 0.005) {
        callbackSkipFrameCounter = callbackSkipFrameCounter+1;
        return;
    }

    //pushback tf and pointcloud to lists
    tfEigenOdomToVelodyneList.push_back(tfCallbackEigenOdomToVelodyne);
    newPointCloud2List.push_back(*msg);
    std::cout<<"new list size is : "<<newPointCloud2List.size()<<", skipped "<< (msg->header.stamp - prevCloudMsg.header.stamp).toSec()<<" second"<<", rotational angle is : "<<rotationMagnitude<<", linear displacement is: "<< linearL1Norm<<std::endl;

    //finishing the callback
    prevCloudMsg.header.stamp=msg->header.stamp;
    prevCallbackTfEigenOdomToVelodyne = tfCallbackEigenOdomToVelodyne;
    callbackSkipFrameCounter = 0;
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
void PointProcessor::downsampleTCloudN(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &inAndOutCloud)
{    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr tempFiltered (new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::VoxelGrid<pcl::PointXYZINormal> sor;
    sor.setInputCloud (inAndOutCloud);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*tempFiltered);
    *inAndOutCloud=*tempFiltered;
}

void PointProcessor::passthroughFilterZ(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudFiltered)
{    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (inCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits (-0.15, 5.0);
    pass.filter (*cloudFiltered);
}

void PointProcessor::passthroughFilterI(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudFiltered)
{    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (inCloud);
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits (1, 200);
    pass.filter (*cloudFiltered);
}

void PointProcessor::passthroughFilterINormal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &inAndOutCloud)
{    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr tempFiltered ( new pcl::PointCloud<pcl::PointXYZINormal> () );
    pcl::PassThrough<pcl::PointXYZINormal> pass;
    pass.setInputCloud (inAndOutCloud);
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits (1, 200);
    pass.filter (*tempFiltered);
    *inAndOutCloud = *tempFiltered;
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

void PointProcessor::registerPairNormal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr newTCloud,
                                  pcl::PointCloud<pcl::PointXYZINormal>::Ptr globalCloud,
                                  Eigen::Matrix4f tToS)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr tempOutput (new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::transformPointCloud (*newTCloud, *tempOutput, tToS);
    *globalCloud += *tempOutput;
}

void PointProcessor::addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                               pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_with_normals)
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZI>);
    searchTree->setInputCloud (cloud);

    pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> normalEstimator;
    normalEstimator.setInputCloud (cloud);
    normalEstimator.setSearchMethod (searchTree);
    normalEstimator.setRadiusSearch (0.40);
    normalEstimator.compute (*cloud_with_normals);
    pcl::copyPointCloud(*cloud, *cloud_with_normals);
}
void PointProcessor::updateNormal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &inAndOutCloud)
{
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZINormal>);
    searchTree->setInputCloud (inAndOutCloud);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr tempCloud ( new pcl::PointCloud<pcl::PointXYZINormal> () );

    pcl::NormalEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal> normalEstimator;
    normalEstimator.setInputCloud (inAndOutCloud);
    normalEstimator.setSearchMethod (searchTree);
    normalEstimator.setRadiusSearch (0.40);
    normalEstimator.compute (*tempCloud);
    pcl::copyPointCloud(*inAndOutCloud, *tempCloud);
    pcl::copyPointCloud(*tempCloud, *inAndOutCloud);
}


void PointProcessor::performIcpWNormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr sourceTCloudNormals,
                                       pcl::PointCloud<pcl::PointXYZINormal>::Ptr targetTCloudNormals,
                                       Eigen::Matrix4f odomPriorNewToPrev,
                                       Eigen::Matrix4f &prevSToT,
                                       Eigen::Matrix4f &sToT)
{

    //transform the source cloud by applying prior transform, the source pointcloud should be the smaller one compared to the target for computational speed.
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr sourceTransNormals (new pcl::PointCloud<pcl::PointXYZINormal>);
    Eigen::Matrix4f prevSToTTransformed=odomPriorNewToPrev*prevSToT;
    pcl::transformPointCloud(*sourceTCloudNormals, *sourceTransNormals, prevSToTTransformed);

//    //downsample pointcloud
//    pcl::PointCloud<pcl::PointXYZI>::Ptr targetFiltered (new pcl::PointCloud<pcl::PointXYZI>);
//    passthroughFilterI(sourceTrans, sourceTrans);
//    passthroughFilterI(targetTCloud, targetFiltered);

    //downsample pointcloud
    passthroughFilterINormal(sourceTransNormals);
    passthroughFilterINormal(targetTCloudNormals);


    //---------------------------------------------------------------------------------------------------------------//
    // setup icp with normal
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
    icp.setTransformationEpsilon (1e-10);
    icp.setMaxCorrespondenceDistance (0.75);   // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    icp.setMaximumIterations (100);
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
    sToT = prevSToTTransformed*Ti;
    prevSToTTransformed = sToT;
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
    //passthrough filter z value
//    passthroughZFilter(newPointTCloud,zFiltered);

    //Apply visual odom prior and performIcpWithNormal
    Eigen::Matrix4f newToGlobal = Eigen::Matrix4f::Identity();  //define 2 variables, Ti, targetToSource
    Eigen::Matrix4f odomPriorNewToPrev;
    odomPriorNewToPrev = newTfEigenOdomToVelodyne.inverse()*prevTfEigenOdomToVelodyne;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr newPointTCloudNormal ( new pcl::PointCloud<pcl::PointXYZINormal> () );
    addNormal(newPointTCloud, newPointTCloudNormal);
    performIcpWNormals(newPointTCloudNormal, globalPointTCloudNormal, odomPriorNewToPrev, prevNewToGlobal, newToGlobal); //icp with prior, the newPointTCloud should be the smaller one compared to the globalPointTCloud for computational speed, as the newPointTCloud will be transformed by prior transform before performing icp
//    performIcpWNormals(zFiltered, globalPointTCloudNormal, prevNewToGlobal, newToGlobal);  //z filtered cloud for icp

    //register globalpointcloud for final pcd construction and visualization
    registerPair(newPointTCloud, globalVisualizationPointTCloud, newToGlobal);
    PointProcessor::downsampleTCloud(globalVisualizationPointTCloud);

    //register globalpointcloud for scan matching
    registerPairNormal(newPointTCloudNormal, globalPointTCloudNormal, newToGlobal);
    PointProcessor::downsampleTCloudN(globalPointTCloudNormal);

    updateNormal(globalPointTCloudNormal);

    if(pointRegistrationCounter>=5){
        pointRegistrationCounter=0;
    }
    //finishing
    pointRegistrationCounter++;
}

int main (int argc, char** argv){
    ros::init (argc, argv, "my_pcl_tutorial");
    PointProcessor pointProcessor;
    ros::Publisher registration_pub;
    sensor_msgs::PointCloud2 globalCloudMsg, currentNewCloud;

    pointProcessor.first_msg = true;

    //ros variables
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Time lastSaveTime=ros::Time::now();
    ros::Duration saveIntervalSec(10);

    //subscribers and publishers
    pointProcessor.pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_downsampled", 1);
    registration_pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_aggregated", 1);
    pointProcessor.pairwise_pub=nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_under_processing", 1);
    ros::Subscriber icpsub=nh.subscribe("velodyne_points", 1, &PointProcessor::lidarCallback, &pointProcessor);
    int count =0, nameCount=0;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration(100).sleep();
    while(ros::ok()){
        //check for new sensor message
        if (pointProcessor.newPointCloud2List.empty() || pointProcessor.first_msg==true){
//            std::cout<<"newPointCloud2List size is: "<<pointProcessor.newPointCloud2List.size()<<std::endl;
//            std::cout<<"first msg: "<<pointProcessor.first_msg<<std::endl;
              std::cout<<"waiting for the first message"<<std::endl;
            continue;}

        //get and visualize new data
        pointProcessor.getNextPointCloudAndTf();
        pcl::toROSMsg(*pointProcessor.newPointTCloud, currentNewCloud);
        pointProcessor.pairwise_pub.publish(currentNewCloud);
        pointProcessor.plottf(pointProcessor.newTfEigenOdomToVelodyne, "zed_odom", "velodyne_current");
        pointProcessor.plottf(pointProcessor.newTfEigenOdomToVelodyne.inverse()*pointProcessor.prevTfEigenOdomToVelodyne, "velodyne_current", "prev_velodyne");
        //alignment and registration
        pointProcessor.registerNewFrame();
        pcl::toROSMsg(*pointProcessor.globalVisualizationPointTCloud, globalCloudMsg);
        registration_pub.publish(globalCloudMsg);
        count ++;
        nameCount++;
        if (count>=10 && nameCount>=100){
            pcl::io::savePCDFileASCII ("/home/kaicheng/pcds/mc202_" + std::to_string(nameCount/10) +".pcd", *pointProcessor.globalVisualizationPointTCloud);
            std::cerr << "Saved " << pointProcessor.globalVisualizationPointTCloud->points.size () << " data points to /home/kaicheng/pcds/mc202_" + std::to_string(nameCount/10) +".pcd" << std::endl;
            count=0;
        }
        //finishing the loop
        loop_rate.sleep();
    }
}
