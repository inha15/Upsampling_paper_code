#ifndef SMPG_H
#define SMPG_H

#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <algorithm>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

class PointInfo_SAM{ 
    template<typename PointT> friend class SMPG;
public:
    PointInfo_SAM();
    PointInfo_SAM(int self_idx, double eps, pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZI>::Ptr tree);
    void setSearchMethod(pcl::search::KdTree<pcl::PointXYZI>::Ptr tree) {search_method = tree;}  
    void calc_eps();
    void get_nearPoint_info();
    void PointAngle();

protected:
    int self_idx;
    double eps;
    int laserId;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr search_method;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
};

template <typename PointT>
class SMPG {
public:
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;

    KdTreePtr search_method;

    SMPG();
    void setInputCloud(PointCloudPtr cloud) {input_cloud = cloud;}
    void setSearchMethod(KdTreePtr tree) {search_method = tree;}
    void setUpsampleStepSize(double stepSize) {upsampleStepSize = stepSize;}
    void initializer_pointinfo();
    void upsample(PointCloudPtr cloud);

private:
    PointCloudPtr input_cloud;
    PointCloudPtr channelCloudPtr[16];
    KdTreePtr grid[16];

    double eps = 0;
    double upsampleStepSize = 0.1;
    vector<PointInfo_SAM> PCinfo;
};

#endif //smpg


PointInfo_SAM::PointInfo_SAM(){};

PointInfo_SAM::PointInfo_SAM(int self_idx_, double eps_, pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_, pcl::search::KdTree<pcl::PointXYZI>::Ptr tree)
    :self_idx(self_idx_), eps(eps_), input_cloud(input_cloud_), search_method(tree)
{
    calc_eps();
    PointAngle();
}

void PointInfo_SAM::calc_eps(){
    double safe_coeff = 0.05;
    double cur_dist = sqrt(input_cloud->points[self_idx].x * input_cloud->points[self_idx].x 
                         + input_cloud->points[self_idx].y * input_cloud->points[self_idx].y 
                         + input_cloud->points[self_idx].z * input_cloud->points[self_idx].z);
    this -> eps = 0.037 * cur_dist;
    if(cur_dist > 2) this -> eps += safe_coeff;
}

void PointInfo_SAM::PointAngle(){
        double z = input_cloud->points[self_idx].z;
        double xy_dist = sqrt(input_cloud->points[self_idx].x * input_cloud->points[self_idx].x + input_cloud->points[self_idx].y * input_cloud->points[self_idx].y);
        this -> laserId = round(atan2(z, xy_dist) * 180 / M_PI);
}

template <typename PointT>
SMPG<PointT>::SMPG(){
        for(int i=0;i<16;i++){
            channelCloudPtr[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
            grid[i].reset(new pcl::search::KdTree<pcl::PointXYZI>());
        }
}

template <typename PointT>
void SMPG<PointT>::initializer_pointinfo(){
    for(int i = 0; i < input_cloud -> points.size(); i++){
        PointInfo_SAM tmp(i, eps, input_cloud, search_method);
        PCinfo.push_back(tmp);
        channelCloudPtr[(tmp.laserId + 15) / 2]->push_back(input_cloud->points[tmp.self_idx]);        
    }

    for (int i = 0; i < 16; i++){
        if (!channelCloudPtr[i]->points.size()) continue;
        grid[i]->setInputCloud(channelCloudPtr[i]);
    }
}

template <typename PointT>
void SMPG<PointT>::upsample(PointCloudPtr cloud){
    initializer_pointinfo();

    //for (int i = 0; i < 16; i++)
        //cout << "channel " << i << " : " << channelCloudPtr[i]->points.size() << endl;

    // intensity로 채널 부여
    for(int i = 0; i < input_cloud -> points.size(); i++){
        this->input_cloud->points[i].intensity = PCinfo[i].laserId;
    }

    for(int i = 0; i < input_cloud -> points.size(); i++){

        vector<int> idx;
        vector<float> sqr_dist;
        if((PCinfo[i].laserId + 15) / 2 + 1 == 16 || channelCloudPtr[(PCinfo[i].laserId + 15) / 2 + 1]->points.size() == 0) continue;
        grid[(PCinfo[i].laserId + 15) / 2 + 1]->nearestKSearch(input_cloud->points[i], 1, idx, sqr_dist);
        if(idx.size()==0 || sqrt(sqr_dist[0]) > PCinfo[i].eps) continue;

        int line_num = sqrt(sqr_dist[0]) / upsampleStepSize;
    
        double line_dist_x, line_dist_y, line_dist_z;
        if (line_num != 0) {
            line_dist_x = abs(input_cloud->points[i].x - channelCloudPtr[(PCinfo[i].laserId + 15) / 2 + 1]->points[idx[0]].x) / (line_num + 1);
            line_dist_y = abs(input_cloud->points[i].y - channelCloudPtr[(PCinfo[i].laserId + 15) / 2 + 1]->points[idx[0]].y) / (line_num + 1);
            line_dist_z = abs(input_cloud->points[i].z - channelCloudPtr[(PCinfo[i].laserId + 15) / 2 + 1]->points[idx[0]].z) / (line_num + 1);
        }
        for(int j = 0; j < line_num; j++){
            double jmp_dist_x = (j+1) * line_dist_x, jmp_dist_y = (j+1) * line_dist_y, jmp_dist_z = (j+1) * line_dist_z;
            pcl::PointXYZI tmp;
            tmp.x = input_cloud->points[i].x + jmp_dist_x;
            tmp.y = input_cloud->points[i].y + jmp_dist_y;
            tmp.z = input_cloud->points[i].z + jmp_dist_z;
            tmp.intensity = input_cloud->points[i].intensity;
            cloud->push_back(tmp);
        }
    }
}