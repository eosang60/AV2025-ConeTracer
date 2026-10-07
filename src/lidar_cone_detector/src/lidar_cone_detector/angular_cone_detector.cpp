#include "lidar_cone_detector/angular_cone_detector.hpp"

AngularConeDetector::AngularConeDetector(const ros::NodeHandle &nh_, const float r) : nh(nh_), rate(r)
{
#if DEBUG == 0
    nh.param<std::string>("/angular_cone_detector_node/pub_cluster_point", pub_cluster_topic, "/cluster");
    pub_cluster_point = nh.advertise<sensor_msgs::PointCloud2>(pub_cluster_topic, 1);
    cluster_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    output_cluster.reset(new sensor_msgs::PointCloud2);
#endif

    nh.param<std::string>("/angular_cone_detector_node/sub_lidar_topic", sub_lidar_topic, "/velodyne_points");
    nh.param<std::string>("/angular_cone_detector_node/pub_left_cone", pub_left_cone_topic, "/left_cone");
    nh.param<std::string>("/angular_cone_detector_node/pub_right_cone", pub_right_cone_topic, "/right_cone");
    nh.param<std::string>("/angular_cone_detector_node/pub_mid_cloud", pub_mid_cloud_topic, "/mid_cloud");
    nh.param<std::string>("/angular_cone_detector_node/pub_mid_point", pub_mid_point_topic, "/mid_point");

    nh.param<float>("/angular_cone_detector_node/min_x", min_x, 0.0);
    nh.param<float>("/angular_cone_detector_node/max_x", max_x, 10.0);
    nh.param<float>("/angular_cone_detector_node/min_z", min_z, -0.38);
    nh.param<float>("/angular_cone_detector_node/max_z", max_z, 1.0);

    nh.param<float>("/angular_cone_detector_node/cone_box_height_threshold", cone_box_height_threshold, 0.5);
    nh.param<float>("/angular_cone_detector_node/cone_distance", cone_distance, 3.5);
    nh.param<float>("/angular_cone_detector_node/cone_distance_threshold", cone_distance_threshold, 0.5);
    nh.param<float>("/angular_cone_detector_node/distance", distance, 5.0);

    nh.param<bool>("/angular_cone_detector_node/ground_remover", GroundRemover, false);
    nh.param<int>("/angular_cone_detector_node/ground_remover_iter", GR_iter, 500);
    nh.param<float>("/angular_cone_detector_node/ground_remover_threshold", gr_threshold, 0.1);

    nh.param<bool>("/angular_cone_detector_node/vocelization", Voxelization, false);
    nh.param<float>("/angular_cone_detector_node/vocelization_threshold", vc_size, 0.02);

    nh.param<float>("/angular_cone_detector_node/eudclidean_distance_threshold", ec_distance, 1.0);
    nh.param<int>("/angular_cone_detector_node/eudclidian_min_cluster", ec_MinCluster, 3);
    nh.param<int>("/angular_cone_detector_node/eudclidian_max_cluster", ec_MaxCluster, 1000);

    nh.param<bool>("/angular_cone_detector_node/stop_cone_detect", e_stop, true);
    nh.param<float>("/angular_cone_detector_node/stop_cone_detect_height", stop_cone_detect_height, 280);

    // 추가 파라미터
    nh.param<bool>("/angular_cone_detector_node/side_follow_mode", side_follow_mode, true);
    nh.param<int>("/angular_cone_detector_node/k_left", k_left, 8);
    nh.param<int>("/angular_cone_detector_node/k_right", k_right, 8);
    nh.param<double>("/angular_cone_detector_node/theta_gate_deg", theta_gate_deg, 60.0);
    nh.param<double>("/angular_cone_detector_node/step_min", step_min, 0.3);
    nh.param<double>("/angular_cone_detector_node/step_max", step_max, 3.0);
    nh.param<int>("/angular_cone_detector_node/max_miss", max_miss, 3);
    nh.param<double>("/angular_cone_detector_node/lane_width_est", lane_width_est, 3.0);

    sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>(sub_lidar_topic, 1, &AngularConeDetector::handler, this);

    if (e_stop)
    {
        sub_cone_box = nh.subscribe<std_msgs::Float32MultiArray>("/cone_bboxes", 1, &AngularConeDetector::bboxHandler, this);
        srv_stop_cone_detect = nh.serviceClient<lidar_cone_detector::stop_cone_detect>("/e_stop");
        srv_stop_cone_detect.waitForExistence();
    }
    pub_motor_cmd = nh.advertise<fs_msgs::ControlCommand>("/fsds/control_command", 1);
    pub_left_cones = nh.advertise<sensor_msgs::PointCloud2>(pub_left_cone_topic, 1);
    pub_right_cones = nh.advertise<sensor_msgs::PointCloud2>(pub_right_cone_topic, 1);
    pub_mid_cloud = nh.advertise<sensor_msgs::PointCloud2>(pub_mid_cloud_topic, 1);
    pub_mid_point = nh.advertise<geometry_msgs::PoseArray>(pub_mid_point_topic, 1);

    pcl_input.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_left_cones.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_right_cones.reset(new pcl::PointCloud<pcl::PointXYZ>());

    pose_arr.reset(new geometry_msgs::PoseArray());
    left_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    right_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    mid_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    output_left.reset(new sensor_msgs::PointCloud2);
    output_right.reset(new sensor_msgs::PointCloud2);
    output_mid.reset(new sensor_msgs::PointCloud2);

    if (Voxelization)
    {
        voxel_.setLeafSize(vc_size, vc_size, vc_size);
    }

    if (GroundRemover)
    {
        groundremover_.setOptimizeCoefficients(true);
        groundremover_.setModelType(pcl::SACMODEL_PLANE);
        groundremover_.setMethodType(pcl::SAC_RANSAC);
        groundremover_.setMaxIterations(GR_iter);
        groundremover_.setDistanceThreshold(gr_threshold);
        neg_extract.setNegative(true);
    }

    past_angle = 0;
    stop_cone_detect_state = false;

    cone_sum = 0;
    cone_diff = 0;

    initialized = false;
    miss_left = miss_right = 0;
}

void AngularConeDetector::bboxHandler(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    int stop_cone_count = 0;
    if (msg->data.empty())
    {
        ROS_WARN("Received invalid bounding box data");
        return;
    }

    for (size_t i = 0; i + 5 < msg->data.size(); i += 6)
    {
        if (int(msg->data[i + 5]) == 2)
        {
            if (msg->data[i + 3] > stop_cone_detect_height)
            {
                stop_cone_count++;
            }
        }
    }
    if (stop_cone_count >= 4)
    {
        stop_cone_detect_state = true;
        lidar_cone_detector::stop_cone_detect srv;
        srv.request.detected = true;
        if (srv_stop_cone_detect.call(srv))
        {
            if (!srv.response.success)
            {
                ROS_WARN("E-STOP service returned failure");
            }
        }
        else
        {
            ROS_ERROR("Failed to call /e_stop service");
        }
    }
}

void AngularConeDetector::handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    ros_to_pcl(cloud_msg);
    set_roi();
    if (Voxelization)
    {
        Voxel();
    }
    if (GroundRemover)
    {
        ground_remover();
    }
    cone_search();
    publish_pcl();
    reset_pcl();
}

void AngularConeDetector::ros_to_pcl(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    cloud_header = cloud_msg->header;
    pcl::fromROSMsg(*cloud_msg, *pcl_input);
}

void AngularConeDetector::set_roi()
{
    if (pcl_input->empty())
    {
        return;
    }

    pcl::PassThrough<pcl::PointXYZ> p_z;
    p_z.setInputCloud(pcl_input);
    p_z.setFilterFieldName("z");
    p_z.setFilterLimits(min_z, max_z);
    p_z.setFilterLimitsNegative(false);
    p_z.filter(*pcl_input);

    pcl::PassThrough<pcl::PointXYZ> p_x;
    p_x.setInputCloud(pcl_input);
    p_x.setFilterFieldName("x");
    p_x.setFilterLimits(min_x, max_x);
    p_x.setFilterLimitsNegative(false);
    p_x.filter(*pcl_input);
}

void AngularConeDetector::Voxel()
{
    voxel_.setInputCloud(pcl_input);
    voxel_.filter(*pcl_input);
}

void AngularConeDetector::ground_remover()
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    groundremover_.setInputCloud(pcl_input);
    groundremover_.segment(*inliers, *coefficients);
    neg_extract.setInputCloud(pcl_input);
    neg_extract.setIndices(inliers);
    neg_extract.filter(*pcl_input);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr AngularConeDetector::cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    static pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> indices_vec;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point;
    Eigen::Vector4f centroid;

    tree->setInputCloud(cloud);
    euclidean_cluster.setClusterTolerance(ec_distance);
    euclidean_cluster.setMinClusterSize(ec_MinCluster);
    euclidean_cluster.setMaxClusterSize(ec_MaxCluster);
    euclidean_cluster.setSearchMethod(tree);
    euclidean_cluster.setInputCloud(cloud);
    euclidean_cluster.extract(indices_vec);

    for (const auto &indices : indices_vec)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cone(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : indices.indices)
        {
            point = cloud->points[idx];
            cluster_cone->push_back(point);
        }
        Eigen::Vector4f c;
        pcl::compute3DCentroid(*cluster_cone, c);
        point.x = c[0];
        point.y = c[1];
        point.z = 0.0;
        cluster_cloud->push_back(point);
    }
    return cluster_cloud;
}

void AngularConeDetector::cone_search()
{
    if (!pcl_input || pcl_input->empty())
    {
        ROS_WARN("pcl_input is null or empty");
        return;
    }

    pcl::PointXYZ point(0, 0, 0);
    pcl::PointXYZRGB mid_point;
    geometry_msgs::Pose pose;
    float theta;

    pcl_input = cluster(pcl_input);

#if DEBUG == 0
    cluster_rgb = toPclXYZRGB(pcl_input, 0, 0, 255);
#endif

    kdtree.setInputCloud(pcl_input);

    pose.position = toGeoPoint(point);
    pose_arr->poses.push_back(pose);

    if (side_follow_mode)
    {
        if (!initialized)
        {
            if (!radius_search(&point))
            {
                return;
            }
            mid_point = toPclXYZRGB(point, 0, 0, 255);
            mid_rgb->push_back(mid_point);
            if (!left_rgb->empty() && !right_rgb->empty())
            {
                last_left = rgb_to_xyz(left_rgb->back());
                last_right = rgb_to_xyz(right_rgb->back());
                last_mid = pcl::PointXYZ((last_left.x + last_right.x) * 0.5f, (last_left.y + last_right.y) * 0.5f, 0.0f);
                initialized = true;
            }
            pose.position = toGeoPoint(point);
            pose_arr->poses.push_back(pose);
        }

        while (follow_sides_step())
        {
            const auto &p = mid_rgb->back();
            pose.position = toGeoPoint(rgb_to_xyz(p));
            pose_arr->poses.push_back(pose);
        }
    }
    else
    {
        while (radius_search(&point))
        {
            mid_point = toPclXYZRGB(point, 0, 0, 255);
            mid_rgb->push_back(mid_point);
            pose.position = toGeoPoint(point);
            pose_arr->poses.push_back(pose);
        }
    }

    if (mid_rgb->empty())
    {
        return;
    }

    for (size_t i = 0; i + 1 < pose_arr->poses.size(); i++)
    {
        theta = calculate_angle(pose_arr->poses[i + 1].position, pose_arr->poses[i].position);
        pose_arr->poses[i].orientation = tf::createQuaternionMsgFromYaw(theta);
    }

    past_angle = calculate_angle(left_rgb->points.front(), right_rgb->points.front());
}

bool AngularConeDetector::radius_search(pcl::PointXYZ *p)
{
    static pcl::PointXYZ point;
    static pcl::PointXYZ right_point;
    static pcl::PointXYZ left_point;

    std::vector<int> idxes;
    std::vector<float> sqr_dist;
    float angle;
    float dis;
    float left_angle = 0.0;
    float right_angle = 0.0;
    bool foundLeft = false;
    bool foundRight = false;

    kdtree.nearestKSearch(*p, 8, idxes, sqr_dist);

    for (const auto &idx : idxes)
    {
        point = pcl_input->points[idx];
        dis = calculate_distance(point, *p);
        if (dis < distance)
        {
            if (left_rgb->empty() || right_rgb->empty())
            {
                angle = calculate_angle(point, *p);
            }
            else
            {
                auto &prev = mid_rgb->points[mid_rgb->points.size() - 2];
                angle = calculate_angle_y(*p, prev);
                angle = calculate_angle(*p, point, angle);
            }

            if (std::fabs(angle) < 90)
            {
                if (angle < left_angle)
                {
                    foundLeft = true;
                    left_point = point;
                    left_angle = angle;
                }
                else if (angle > right_angle)
                {
                    foundRight = true;
                    right_point = point;
                    right_angle = angle;
                }
            }
        }
    }

    if (!foundLeft || !foundRight)
    {
        return false;
    }

    p->x = (left_point.x + right_point.x) / 2;
    p->y = (left_point.y + right_point.y) / 2;

    auto left_rgb_point = toPclXYZRGB(left_point, 255, 0, 0);
    auto right_rgb_point = toPclXYZRGB(right_point, 0, 255, 0);

    left_rgb->push_back(left_rgb_point);
    right_rgb->push_back(right_rgb_point);

    return true;
}

bool AngularConeDetector::follow_sides_step()
{
    if (!initialized || pcl_input->empty())
        return false;

    std::vector<int> idxL, idxR;
    std::vector<float> d2L, d2R;
    kdtree.nearestKSearch(last_left, k_left, idxL, d2L);
    kdtree.nearestKSearch(last_right, k_right, idxR, d2R);

    pcl::PointXYZ heading_mid;
    if (mid_rgb->size() >= 2)
    {
        auto prev = rgb_to_xyz(mid_rgb->points[mid_rgb->size() - 2]);
        auto cur = rgb_to_xyz(mid_rgb->points.back());
        heading_mid.x = cur.x - prev.x;
        heading_mid.y = cur.y - prev.y;
        heading_mid.z = 0.0f;
    }
    else
    {
        heading_mid = pcl::PointXYZ(1.0f, 0.0f, 0.0f);
    }

    auto norm2d = [](const pcl::PointXYZ &a)
    { return std::sqrt(a.x * a.x + a.y * a.y); };
    auto sub = [](const pcl::PointXYZ &a, const pcl::PointXYZ &b)
    { return pcl::PointXYZ(a.x - b.x, a.y - b.y, 0.0f); };
    auto angleDeg = [&](const pcl::PointXYZ &a, const pcl::PointXYZ &b)
    {
        double dn = norm2d(a) * norm2d(b);
        if (dn <= 1e-9)
            return 0.0;
        double c = std::max(-1.0, std::min(1.0, (a.x * b.x + a.y * b.y) / dn));
        return std::acos(c) * 180.0 / M_PI;
    };

    auto gate = [&](const pcl::PointXYZ &prev, const pcl::PointXYZ &cand) -> bool
    {
        pcl::PointXYZ step = sub(cand, prev);
        double d = norm2d(step);
        if (d < step_min || d > step_max)
            return false;
        if (angleDeg(heading_mid, step) > theta_gate_deg)
            return false;
        return true;
    };

    bool okL = false, okR = false;
    pcl::PointXYZ candL, candR;
    double bestL = 1e9, bestR = 1e9;

    for (int i : idxL)
    {
        const auto &c = pcl_input->points[i];
        if (!gate(last_left, c))
            continue;
        double cost = std::hypot(c.x - last_left.x, c.y - last_left.y);
        if (cost < bestL)
        {
            bestL = cost;
            candL = c;
            okL = true;
        }
    }

    for (int i : idxR)
    {
        const auto &c = pcl_input->points[i];
        if (!gate(last_right, c))
            continue;
        double cost = std::hypot(c.x - last_right.x, c.y - last_right.y);
        if (cost < bestR)
        {
            bestR = cost;
            candR = c;
            okR = true;
        }
    }

    if (!okL)
    {
        miss_left++;
    }
    if (!okR)
    {
        miss_right++;
    }
    if (!okL && okR && miss_left <= max_miss)
    {
        pcl::PointXYZ base = pcl::PointXYZ(last_right.x - last_left.x, last_right.y - last_left.y, 0.0f);
        candL = pcl::PointXYZ(candR.x - base.x, candR.y - base.y, 0.0f);
        okL = true;
    }
    if (!okR && okL && miss_right <= max_miss)
    {
        pcl::PointXYZ base = pcl::PointXYZ(last_right.x - last_left.x, last_right.y - last_left.y, 0.0f);
        candR = pcl::PointXYZ(candL.x + base.x, candL.y + base.y, 0.0f);
        okR = true;
    }

    if (!(okL && okR))
        return false;

    miss_left = miss_right = 0;

    last_left = candL;
    last_right = candR;
    last_mid = pcl::PointXYZ((last_left.x + last_right.x) * 0.5f, (last_left.y + last_right.y) * 0.5f, 0.0f);

    left_rgb->push_back(toPclXYZRGB(last_left, 255, 0, 0));
    right_rgb->push_back(toPclXYZRGB(last_right, 0, 255, 0));
    mid_rgb->push_back(toPclXYZRGB(last_mid, 0, 0, 255));

    return true;
}

void AngularConeDetector::publish_pcl()
{
#if DEBUG == 0
    pcl::toROSMsg(*cluster_rgb, *output_cluster);
    output_cluster->header = cloud_header;
    pub_cluster_point.publish(output_cluster);
#endif

    pcl::toROSMsg(*left_rgb, *output_left);
    pcl::toROSMsg(*mid_rgb, *output_mid);
    pcl::toROSMsg(*right_rgb, *output_right);

    output_left->header = cloud_header;
    output_right->header = cloud_header;
    output_mid->header = cloud_header;
    pose_arr->header = cloud_header;

    pub_left_cones.publish(output_left);
    pub_right_cones.publish(output_right);
    pub_mid_cloud.publish(output_mid);
    pub_mid_point.publish(pose_arr);
}

void AngularConeDetector::reset_pcl()
{
#if DEBUG == 0
    cluster_rgb->clear();
#endif

    pcl_input->clear();

    cloud_left_cones->clear();
    cloud_right_cones->clear();

    left_rgb->clear();
    right_rgb->clear();
    mid_rgb->clear();
    pose_arr->poses.clear();

    initialized = false;
    miss_left = miss_right = 0;
}

pcl::PointXYZ AngularConeDetector::toPclXYZ(const geometry_msgs::Point geo_point)
{
    pcl::PointXYZ pcl_point;
    pcl_point.x = geo_point.x;
    pcl_point.y = geo_point.y;
    pcl_point.z = geo_point.z;
    return pcl_point;
}

pcl::PointXYZRGB AngularConeDetector::toPclXYZRGB(const geometry_msgs::Point geo_point, int r, int g, int b)
{
    pcl::PointXYZRGB pcl_point;
    pcl_point.x = geo_point.x;
    pcl_point.y = geo_point.y;
    pcl_point.z = geo_point.z;
    pcl_point.r = r;
    pcl_point.g = g;
    pcl_point.b = b;
    return pcl_point;
}

pcl::PointXYZRGB AngularConeDetector::toPclXYZRGB(const pcl::PointXYZ pcl_xyz, int r, int g, int b)
{
    pcl::PointXYZRGB pcl_xyzrgb;
    pcl_xyzrgb.x = pcl_xyz.x;
    pcl_xyzrgb.y = pcl_xyz.y;
    pcl_xyzrgb.z = pcl_xyz.z;
    pcl_xyzrgb.r = r;
    pcl_xyzrgb.g = g;
    pcl_xyzrgb.b = b;
    return pcl_xyzrgb;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr AngularConeDetector::toPclXYZRGB(const pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud, int r, int g, int b)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgbCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB point;
    for (size_t i = 0; i < xyzCloud->size(); i++)
    {
        point = toPclXYZRGB(xyzCloud->points[i], r, g, b);
        xyzrgbCloud->push_back(point);
    }
    return xyzrgbCloud;
}

geometry_msgs::Point AngularConeDetector::toGeoPoint(const pcl::PointXYZ pcl_point)
{
    geometry_msgs::Point geo_point;
    geo_point.x = pcl_point.x;
    geo_point.y = pcl_point.y;
    geo_point.z = pcl_point.z;
    return geo_point;
}

float AngularConeDetector::calculate_distance(const pcl::PointXYZ p1, const pcl::PointXYZ p2)
{
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

float AngularConeDetector::calculate_angle(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    geometry_msgs::Point point;
    point.x = p1.x - p2.x;
    point.y = p1.y - p2.y;
    return std::atan2(point.y, point.x);
}

float AngularConeDetector::calculate_angle(const pcl::PointXYZ p1, const pcl::PointXYZ p2)
{
    geometry_msgs::Point point;
    point.x = p1.x - p2.x;
    point.y = p1.y - p2.y;
    return std::atan2(point.y, point.x);
}

float AngularConeDetector::calculate_angle(const pcl::PointXYZRGB p1, const pcl::PointXYZRGB p2)
{
    geometry_msgs::Point point;
    point.x = p1.x - p2.x;
    point.y = p1.y - p2.y;
    return std::atan2(point.x, point.y);
}

float AngularConeDetector::calculate_angle(const pcl::PointXYZ mid_point, const pcl::PointXYZ target_point, float angle)
{
    static pcl::PointXYZ zero_point(0, 0, 0);
    static pcl::PointXYZ relative_target;
    float theta;

    relative_target.x = target_point.x - mid_point.x;
    relative_target.y = target_point.y - mid_point.y;

    theta = std::atan2(relative_target.y, relative_target.x) * 180 / M_PI;
    return theta;
}

float AngularConeDetector::calculate_angle_y(const pcl::PointXYZ &p1, const pcl::PointXYZRGB &p2)
{
    const float dx = p2.x - p1.x;
    const float dy = p2.y - p1.y;

    float theta_x = std::atan2(dx, dy);
    float angle = static_cast<float>(M_PI_2) - theta_x;

    if (angle > static_cast<float>(M_PI))
        angle -= static_cast<float>(2.0 * M_PI);
    if (angle <= -static_cast<float>(M_PI))
        angle += static_cast<float>(2.0 * M_PI);

    return angle;
}
