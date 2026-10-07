#include "lidar_cone_detector/lidar_wall_detector.hpp"

WallDetector::WallDetector(const ros::NodeHandle &nh_) : nh(nh_)
{
    nh.param<std::string>("/wall_detector_node/sub_lidar_topic", sub_lidar_topic, "/velodyne_points");
    nh.param<std::string>("/wall_detector_node/pub_wall_info_topic", pub_wall_info_topic, "/wall_info");

    nh.param<float>("/wall_detector_node/min_x", min_x, 0.0);
    nh.param<float>("/wall_detector_node/max_x", max_x, 10.0);
    nh.param<float>("/wall_detector_node/min_x", min_x, 0.0);
    nh.param<float>("/wall_detector_node/max_x", max_x, 10.0);
    nh.param<float>("/wall_detector_node/min_z", min_z, -0.38);
    nh.param<float>("/wall_detector_node/max_z", max_z, 1.0);

    nh.param<float>("/wall_detector_node/cone_distance", cone_distance, 3.5);
    nh.param<float>("/wall_detector_node/cone_distance_threshold", cone_distance_threshold, 0.5);
    nh.param<float>("/wall_detector_node/distance", distance, 5.0);

    nh.param<bool>("/wall_detector_node/ground_remover", GroundRemover, false);
    nh.param<int>("/wall_detector_node/ground_remover_iter", GR_iter, 500);
    nh.param<float>("/wall_detector_node/ground_remover_threshold", gr_threshold, 0.1);

    nh.param<bool>("/wall_detector_node/vocelization", Voxelization, false);
    nh.param<float>("/wall_detector_node/vocelization_threshold", vc_size, 0.02);
}