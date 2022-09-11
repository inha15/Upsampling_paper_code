#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <algorithm>
#include <string>
#include <time.h>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>   //noise filtering
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>

#include <sampling_algorithm/sampling.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> PCXYZI;
typedef pcl::PointXYZ PXYZ;
typedef pcl::PointXYZI PXYZI;

ros::Publisher pub;

float voxel_size_x = 0.02, voxel_size_y = 0.02, voxel_size_z = 0.05;
double US_searchRadius = 0.03, US_upsamplingRadius = 0.03, US_upsamplingStepSize = 0.02;

void UPSMP(PCXYZI::Ptr input_cloud, PCXYZI::Ptr upsampledCloud){
    pcl::search::KdTree<PXYZI>::Ptr tree(new pcl::search::KdTree<PXYZI>);
    tree->setInputCloud(input_cloud);

    PCXYZI::Ptr new_point(new PCXYZI);
    SMPG<PXYZI> smp;
    smp.setUpsampleStepSize(US_upsamplingStepSize);
    smp.setSearchMethod(tree);
    smp.setInputCloud(input_cloud);
    smp.upsample(new_point);
    *upsampledCloud = *new_point + *input_cloud;
    cout << "Input Cloud : " << input_cloud->points.size() << endl;
    cout << "ADD : " << new_point->points.size();
}

void DownSMP(PCXYZI::Ptr rawData, PCXYZI::Ptr downsampledCloud){ //Voxelization = DownSampling 
    PCXYZI::Ptr Data_for_voxel(new PCXYZI);
    pcl::VoxelGrid<PXYZI> vg;                            //declare voxel

    copyPointCloud(*rawData, *Data_for_voxel);            //rawData  ->  Data_for_voxel  ... just copy for modify
    vg.setInputCloud (Data_for_voxel);     //Data_for_voxel  ->  vg space  ... deep copy , makeShared() return PC
    vg.setLeafSize (voxel_size_x, voxel_size_y, voxel_size_z);		        //voxel size setting(x,y,z)
    vg.filter (*downsampledCloud);                      //voxelized datas are included in downsampledCloud

    cout << "  \tREMOVE : " << rawData->points.size() - downsampledCloud->points.size() << endl;
    cout << "Final Points : " << downsampledCloud->points.size() << endl << endl;

}

void smp(const sensor_msgs::PointCloud2ConstPtr& rawdata){

    PCXYZI::Ptr tmp(new PCXYZI);
    PCXYZI::Ptr upsampled_cloud(new PCXYZI);
    PCXYZI::Ptr resampled_cloud(new PCXYZI);
    PCXYZI::Ptr ROI_cloud(new PCXYZI);
    pcl::fromROSMsg(*rawdata,*tmp);

    UPSMP(tmp, upsampled_cloud);
    DownSMP(upsampled_cloud, resampled_cloud);

    sensor_msgs::PointCloud2 output; 
    pcl::PCLPointCloud2 tmp_PCL;                               //declare PCL_PC2
    pcl::toPCLPointCloud2(*resampled_cloud, tmp_PCL);           //PC -> PCL_PC2
    pcl_conversions::fromPCL(tmp_PCL, output);                 //PCL_PC2 -> sensor_msg_PC2
    output.header.frame_id = "velodyne";
    pub.publish(output);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "test_sampling"); //node name 
	ros::NodeHandle nh;         //nodehandle

    nh.getParam("/sampling_node/voxel_size_x", voxel_size_x);
    nh.getParam("/sampling_node/voxel_size_y", voxel_size_y);
    nh.getParam("/sampling_node/voxel_size_z", voxel_size_z);
    nh.getParam("/sampling_node/upsamplingStepSize", US_upsamplingStepSize);

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 100, smp);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_resampling", 100);
    
	ros::spin();
}
