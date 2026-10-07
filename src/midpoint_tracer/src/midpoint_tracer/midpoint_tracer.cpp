#include "midpoint_tracer/midpoint_tracer.hpp"
#include <cmath>
#include <limits>

MidpointTracer::MidpointTracer(const ros::NodeHandle &nh) : nh_(nh)
{
    sub_midpoint_topic_ = nh_.param<std::string>("/sub_midpoint_topic", "/mid_point");
    pub_cmd_topic_ = nh_.param<std::string>("/pub_cmd_topic", "/fsds/control_command");

    wheelbase_ = nh_.param<double>("/wheelbase", 1.3);
    base_lookahead_ = nh_.param<double>("/base_lookahead", 6.0);
    min_lookahead_ = nh_.param<double>("/min_lookahead", 6.0);
    max_lookahead_ = nh_.param<double>("/max_lookahead", 6.0);
    max_steer_rad_ = nh_.param<double>("/max_steer_rad", 1.0);
    sampling_ds_ = nh_.param<double>("/sampling_ds", 0.2);

    use_adaptive_ld_ = nh_.param<bool>("/use_adaptive_lookahead", true);
    kappa_gain_ = nh_.param<double>("/kappa_gain", 4.0);
    weight_min_ = nh_.param<double>("/weight_min", 0.3);
    weight_max_ = nh_.param<double>("/weight_max", 1.0);
    max_iter_ = nh_.param<int>("/max_iter", 6);
    tol_ = nh_.param<double>("/tol", 0.03);
    damping_ = nh_.param<double>("/damping", 0.6);
    conservative_samples_ = nh_.param<int>("/conservative_samples", 3);

    // throttle params
    min_throttle_ = nh_.param<double>("/min_throttle", 0.3);
    max_throttle_ = nh_.param<double>("/max_throttle", 1.0);
    kappa_sat_ = nh_.param<double>("/kappa_sat", 0.35); // 1/m

    sub_midpoint_ = nh_.subscribe<geometry_msgs::PoseArray>(sub_midpoint_topic_, 1, &MidpointTracer::MidPointCallBack, this);
    pub_cmd_ = nh_.advertise<fs_msgs::ControlCommand>(pub_cmd_topic_, 1);
}

void MidpointTracer::MidPointCallBack(const geometry_msgs::PoseArray::ConstPtr &path)
{

    mid_point_ = *path;

    try
    {
        if (use_adaptive_ld_)
        {
            last_goal_ = spline::ComputeWeightedGoalAdaptive(
                mid_point_,
                base_lookahead_, min_lookahead_, max_lookahead_,
                kappa_gain_, weight_min_, weight_max_,
                sampling_ds_,
                /*use_abs_kappa=*/true,
                max_iter_, tol_, damping_, conservative_samples_);
            ref_path_ = last_goal_.path;
            last_goal_valid_ = true;
        }
        else
        {
            ref_path_ = spline::GenerateSplinePath(mid_point_, sampling_ds_, nullptr);
            last_goal_valid_ = false;
        }
    }
    catch (const std::exception &e)
    {
        ROS_WARN_STREAM_THROTTLE(1.0, "[MidpointTracer] spline/adaptive failed: " << e.what());
        ref_path_ = *path;
        last_goal_valid_ = false;
    }

    CalculateControl();
}

bool MidpointTracer::FindLookaheadOnPath(const geometry_msgs::PoseArray &path,
                                         double Ld,
                                         geometry_msgs::Point &goal,
                                         double &goal_dist,
                                         size_t &goal_idx) const
{
    if (path.poses.empty())
        return false;

    double best_err = std::numeric_limits<double>::infinity();
    size_t best_i = 0;
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        const auto &p = path.poses[i].position;
        double d = std::hypot(p.x, p.y);
        double err = std::fabs(d - Ld);
        if (err < best_err)
        {
            best_err = err;
            best_i = i;
        }
    }
    goal_idx = best_i;
    goal = path.poses[best_i].position;
    goal_dist = std::hypot(goal.x, goal.y);
    return true;
}

double MidpointTracer::EstimateCurvatureAt(const geometry_msgs::PoseArray &path, size_t idx)
{
    if (path.poses.size() < 3)
        return 0.0;
    size_t i1 = (idx == 0) ? 0 : idx - 1;
    size_t i2 = idx;
    size_t i3 = std::min(idx + 1, path.poses.size() - 1);

    const auto &p1 = path.poses[i1].position;
    const auto &p2 = path.poses[i2].position;
    const auto &p3 = path.poses[i3].position;

    double a = std::hypot(p2.x - p1.x, p2.y - p1.y);
    double b = std::hypot(p3.x - p2.x, p3.y - p2.y);
    double c = std::hypot(p3.x - p1.x, p3.y - p1.y);
    if (a < 1e-6 || b < 1e-6 || c < 1e-6)
        return 0.0;

    double s = (a + b + c) / 2.0;
    double area_sq = std::max(0.0, s * (s - a) * (s - b) * (s - c));
    double area = std::sqrt(area_sq);
    double kappa = (4.0 * area) / (a * b * c); // >=0
    return kappa;                              // 절대값 곡률
}

void MidpointTracer::CalculateControl()
{
    if (ref_path_.poses.size() < 2)
    {
        ROS_WARN_THROTTLE(1.0, "[MidpointTracer] path too short");
        PublishCmd(0.0f, 0.0f);
        return;
    }

    // 1) 룩어헤드 및 목표점
    double Ld = clamp(base_lookahead_, min_lookahead_, max_lookahead_);
    geometry_msgs::Point target{};
    double target_dist = 0.0;
    size_t target_idx = 0;

    if (use_adaptive_ld_ && last_goal_valid_)
    {
        Ld = clamp(last_goal_.used_lookahead, min_lookahead_, max_lookahead_);
        // last_goal_.target 기준으로 가장 가까운 경로 인덱스 찾기
        double best_d2 = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < ref_path_.poses.size(); ++i)
        {
            const auto &p = ref_path_.poses[i].position;
            double dx = p.x - last_goal_.target.x;
            double dy = p.y - last_goal_.target.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2)
            {
                best_d2 = d2;
                target_idx = i;
            }
        }
        target = ref_path_.poses[target_idx].position;
        target_dist = std::hypot(target.x, target.y);
        if (target_dist < 0.5 * Ld || target_dist > 1.5 * Ld)
        {
            FindLookaheadOnPath(ref_path_, Ld, target, target_dist, target_idx);
        }
    }
    else
    {
        FindLookaheadOnPath(ref_path_, Ld, target, target_dist, target_idx);
    }

    // 2) Pure Pursuit 조향
    double alpha = std::atan2(target.y, target.x);
    Ld = std::max(1e-2, Ld);
    double delta = std::atan((2.0 * wheelbase_ * std::sin(alpha)) / Ld);
    delta = clamp(delta, -max_steer_rad_, max_steer_rad_);

    // 3) 곡률 기반 스로틀
    double kappa_abs = EstimateCurvatureAt(ref_path_, target_idx);
    double u = clamp(kappa_abs / std::max(1e-6, kappa_sat_), 0.0, 1.0);
    double throttle = max_throttle_ - (max_throttle_ - min_throttle_) * u;
    throttle = clamp(throttle, min_throttle_, max_throttle_);

    PublishCmd(static_cast<float>(throttle), static_cast<float>(delta));
    ROS_INFO("[MidpointTracer] Ld: %.2f, target_idx: %zu, target_dist: %.2f, delta(deg): %.2f, kappa: %.3f, throttle: %.2f",
             Ld, target_idx, target_dist, delta * 180.0 / M_PI, kappa_abs, throttle);
}

void MidpointTracer::PublishCmd(float throttle, float steering)
{
    fs_msgs::ControlCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.throttle = throttle;
    cmd.steering = steering;
    cmd.brake = 0.0f;
    pub_cmd_.publish(cmd);
}
