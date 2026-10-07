#ifndef ANGULAR_LIDAR_CONE_DETECTOR
#define ANGULAR_LIDAR_CONE_DETECTOR

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include "fs_msgs/ControlCommand.h"
#include "lidar_cone_detector/stop_cone_detect.h"

#define DEBUG 0

class AngularConeDetector
{
private:
#if DEBUG == 0
    ros::Publisher pub_cluster_point;
    std::string pub_cluster_topic;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_rgb;
    sensor_msgs::PointCloud2::Ptr output_cluster;
#endif

    ros::NodeHandle nh;

    ros::Subscriber sub_pcl;
    ros::Subscriber sub_cone_box;

    ros::ServiceClient srv_stop_cone_detect;

    ros::Publisher pub_left_cones;
    ros::Publisher pub_right_cones;
    ros::Publisher pub_mid_cloud;
    ros::Publisher pub_mid_point;
    ros::Publisher pub_motor_cmd;

    std_msgs::Header cloud_header;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_cones;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_cones;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_rgb;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mid_rgb;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_rgb;

    sensor_msgs::PointCloud2::Ptr output_left;
    sensor_msgs::PointCloud2::Ptr output_right;
    sensor_msgs::PointCloud2::Ptr output_mid;

    geometry_msgs::PoseArray::Ptr pose_arr;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_;
    pcl::SACSegmentation<pcl::PointXYZ> groundremover_;
    pcl::ExtractIndices<pcl::PointXYZ> neg_extract;
    pcl::ExtractIndices<pcl::PointXYZ> pos_extract;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    std::string sub_lidar_topic;
    std::string pub_mid_cloud_topic;
    std::string pub_mid_point_topic;
    std::string pub_left_cone_topic;
    std::string pub_right_cone_topic;

    float rate;

    float distance;
    float cone_distance;
    float cone_distance_threshold;
    float cone_box_height_threshold;

    bool GroundRemover;
    int GR_iter;
    float gr_threshold;

    bool Voxelization;
    float vc_size;

    float ec_distance;
    int ec_MinCluster;
    int ec_MaxCluster;

    float min_x;
    float max_x;
    float min_z;
    float max_z;

    float past_angle;

    int cone_sum;
    int cone_diff;

    bool e_stop;
    bool stop_cone_detect_state;
    float stop_cone_detect_height;

    // === 추가: 좌/우 추적 모드 상태 및 파라미터 ===
    bool side_follow_mode;
    int k_left, k_right;
    double theta_gate_deg;
    double step_min, step_max;
    int max_miss;
    double lane_width_est;

    bool initialized;
    pcl::PointXYZ last_left, last_right, last_mid;
    int miss_left, miss_right;

public:
    AngularConeDetector(const ros::NodeHandle &nh_, const float r);

    void bboxHandler(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void ros_to_pcl(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void ground_remover();
    void Voxel();
    void set_roi();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void stopConeDetect(const lidar_cone_detector::stop_cone_detect::Request &req,
                        const lidar_cone_detector::stop_cone_detect::Response &res);
    void cone_search();
    bool radius_search(pcl::PointXYZ *p);
    void publish_pcl();
    void reset_pcl();

    geometry_msgs::Point toGeoPoint(const pcl::PointXYZ pcl_point);
    pcl::PointXYZ toPclXYZ(const geometry_msgs::Point geo_point);
    pcl::PointXYZRGB toPclXYZRGB(const geometry_msgs::Point geo_point, int r, int g, int b);
    pcl::PointXYZRGB toPclXYZRGB(const pcl::PointXYZ pcl_xyz, int r, int g, int b);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr toPclXYZRGB(const pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud, int r, int g, int b);

    float calculate_distance(const pcl::PointXYZ p1, const pcl::PointXYZ p2);
    float calculate_angle(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
    float calculate_angle(const pcl::PointXYZ p1, const pcl::PointXYZ p2);
    float calculate_angle(const pcl::PointXYZRGB p1, const pcl::PointXYZRGB p2);
    float calculate_angle(const pcl::PointXYZ mid_point, const pcl::PointXYZ target_point, float theta);
    float calculate_angle_y(const pcl::PointXYZ &p1, const pcl::PointXYZRGB &p2);

private:
    bool follow_sides_step();
    inline pcl::PointXYZ rgb_to_xyz(const pcl::PointXYZRGB &p) { return pcl::PointXYZ(p.x, p.y, p.z); }
};

#endif // ANGULAR_LIDAR_CONE_DETECTOR
