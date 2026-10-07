#ifndef LIDAR_CONE_DETECTOR
#define LIDAR_CONE_DETECTOR

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

class ConeDetector
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_pcl;
    ros::Publisher pub_left_cones;
    ros::Publisher pub_right_cones;
    ros::Publisher pub_left_marker;
    ros::Publisher pub_right_marker;

    std_msgs::Header cloud_header;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr left;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_L;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_M;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_H;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr right;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_L;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_M;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_H;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left_cones;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right_cones;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_rgb;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_rgb;

    sensor_msgs::PointCloud2::Ptr output_left;
    sensor_msgs::PointCloud2::Ptr output_right;

    visualization_msgs::MarkerArray left_marker_array;
    visualization_msgs::MarkerArray right_marker_array;

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;

    pcl::ModelCoefficients::Ptr left_coefficients;
    pcl::PointIndices::Ptr left_inliers;
    pcl::ModelCoefficients::Ptr right_coefficients;
    pcl::PointIndices::Ptr right_inliers;

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclidean_cluster;
    std::vector<pcl::PointIndices> left_cluster_indices;
    std::vector<pcl::PointIndices> right_cluster_indices;

    std::string sub_lidar_topic;
    std::string pub_left_cone_topic;
    std::string pub_right_cone_topic;
    std::string pub_left_cone_marker_topic;
    std::string pub_right_cone_marker_topic;

    float rate;

    float cone_distance;
    float cone_distance_threshold;

    float ec_distance;
    float vc_size;

    float min_x;
    float max_x;

    float min_Lx;
    float max_Lx;
    float min_Mx;
    float max_Mx;
    float min_Hx;
    float max_Hx;
    float min_y1;
    float max_y1;
    float min_y2;
    float max_y2;
    float min_z;
    float max_z;

    float dy;
    float dy_L;
    float dy_M;
    float dy_H;
    float dyL_dt;
    float dyM_dt;
    float dyH_dt;

public:
    ConeDetector(const ros::NodeHandle &nh_, const float r);
    static bool sort_cone(const visualization_msgs::Marker &p1, const visualization_msgs::Marker &p2)
    {
        return p1.points.data()->x < p2.points.data()->x;
    }
    void handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void ros_to_pcl(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void Voxel();
    void set_roi();
    void seg_cone_with_ransac();
    void seg_cone();
    void cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::PointIndices> &indices);
    visualization_msgs::Marker publish_marker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, std::string ns, int cluster_id, float r, float g, float b);
    void publish_pcl();
    void reset_pcl();
};

#endif // LIDAR_CONE_DETECTOR