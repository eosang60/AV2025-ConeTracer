#ifndef LIDAR_WALL_DETECTOR_HPP
#define LIDAR_WALL_DETECTOR_HPP

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "carsa_msgs/wall.h"
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class WallDetector
{
public:
    WallDetector(const ros::NodeHandle &nh_);

    void handler(const sensor_msgs::PointCloud2ConstPtr &msg);
    void ros_to_pcl(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void ground_remover();
    void Voxel();
    void set_roi();
    void DetectWall();
    void reset_pcl();

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_lidar;
    ros::Publisher pub_wall_info;

    std::string sub_lidar_topic;
    std::string pub_wall_info_topic;

    float wall_iter;
    float wall_dis_threshold;
    float wall_ang_tolerance;
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    pcl::SACSegmentation<pcl::PointXYZ> groundremover_;
    bool GroundRemover;
    int GR_iter;
    float gr_threshold;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_;
    bool Voxelization;
    float vc_size;

    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
};

#endif // LIDAR_WALL_DETECTOR_HPP