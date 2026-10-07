#pragma once
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <tuple>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace spline
{

    // 1D 자연 3차 스플라인
    class Spline1D
    {
    public:
        Spline1D(const std::vector<double> &s, const std::vector<double> &y);
        double calc(double t) const;
        double calcd(double t) const;
        double calcdd(double t) const;
        double s_min() const { return s_.front(); }
        double s_max() const { return s_.back(); }

    private:
        int search(double t) const;
        std::vector<double> s_, a_, b_, c_, d_;
    };

    // 2D 스플라인 x(s), y(s)
    class Spline2D
    {
    public:
        explicit Spline2D(const geometry_msgs::PoseArray &centers);
        geometry_msgs::Point position(double s) const;
        void derivatives(double s, double &dx, double &dy, double &ddx, double &ddy) const;
        double curvature(double s) const;
        double s_min() const { return sx_.s_min(); }
        double s_max() const { return sx_.s_max(); }
        geometry_msgs::PoseArray samplePath(double ds, std::vector<double> *out_kappa = nullptr) const;
        geometry_msgs::Point pointAtArc(double targetArc, int refine_iter = 0) const;

    private:
        static std::vector<double> cumulativeS(const geometry_msgs::PoseArray &centers);
        Spline1D sx_, sy_;
    };

    // 결과 묶음
    struct WeightedGoalResult
    {
        geometry_msgs::Point target;   // 최종 목표점
        double kappa_at_target;        // 목표점에서 곡률
        double used_lookahead;         // 최종 사용 Ld
        geometry_msgs::PoseArray path; // 시각화용 샘플 경로
    };

    // 곡률 기반 룩어헤드: 파라미터화 + 고정점 반복
    WeightedGoalResult
    ComputeWeightedGoalAdaptive(const geometry_msgs::PoseArray &centers,
                                double base_lookahead,
                                double min_lookahead,
                                double max_lookahead,
                                double kappa_gain,
                                double weight_min,  // ex) 0.2
                                double weight_max,  // ex) 1.0
                                double sampling_ds, // 경로 샘플 간격
                                bool use_abs_kappa = true,
                                int max_iter = 5,
                                double tol = 0.05,    // [m]
                                double damping = 0.5, // 0~1
                                int conservative_samples = 0);

    // 스플라인 경로 생성
    geometry_msgs::PoseArray GenerateSplinePath(const geometry_msgs::PoseArray &centers,
                                                double ds,
                                                std::vector<double> *out_kappa = nullptr);

    // 레거시 호환 래퍼
    WeightedGoalResult
    ComputeWeightedGoal(const geometry_msgs::PoseArray &centers,
                        double base_lookahead,
                        double min_lookahead,
                        double max_lookahead,
                        double kappa_gain,
                        double sampling_ds);

} // namespace spline
