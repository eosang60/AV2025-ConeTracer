#ifndef _MIDPOINT_TRACER_HPP_
#define _MIDPOINT_TRACER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <fs_msgs/ControlCommand.h>
#include <string>
#include "midpoint_tracer/cubic_spline.hpp"

class MidpointTracer
{
public:
    explicit MidpointTracer(const ros::NodeHandle &nh);
    void MidPointCallBack(const geometry_msgs::PoseArray::ConstPtr &path);

private:
    void CalculateControl();
    void PublishCmd(float throttle, float steering);

    static inline double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    // Ld에 가장 가까운 점 + 인덱스
    bool FindLookaheadOnPath(const geometry_msgs::PoseArray &path,
                             double Ld,
                             geometry_msgs::Point &goal,
                             double &goal_dist,
                             size_t &goal_idx) const;

    // 3점으로 곡률 근사(|kappa|). 인덱스 주변에서 계산
    static double EstimateCurvatureAt(const geometry_msgs::PoseArray &path, size_t idx);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_midpoint_;
    ros::Publisher pub_cmd_;

    std::string sub_midpoint_topic_;
    std::string pub_cmd_topic_;

    geometry_msgs::PoseArray mid_point_;
    geometry_msgs::PoseArray ref_path_;

    // Pure Pursuit
    double wheelbase_;
    double base_lookahead_, min_lookahead_, max_lookahead_;
    double max_steer_rad_;
    double sampling_ds_;

    // Adaptive lookahead
    bool use_adaptive_ld_;
    double kappa_gain_, weight_min_, weight_max_;
    int max_iter_;
    double tol_;
    double damping_;
    int conservative_samples_;

    spline::WeightedGoalResult last_goal_;
    bool last_goal_valid_{false};

    // Throttle from curvature
    double min_throttle_, max_throttle_;
    double kappa_sat_; // [1/m] 이 값 이상의 곡률이면 min_throttle로 포화
};

#endif
