#include "midpoint_tracer/cubic_spline.hpp"

namespace spline
{
    // --- 유틸 ---
    static inline double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    // --- Spline1D ---
    Spline1D::Spline1D(const std::vector<double> &s, const std::vector<double> &y)
        : s_(s)
    {
        if (s.size() < 2 || y.size() != s.size())
            throw std::invalid_argument("Spline1D: invalid input");
        const int n = (int)s.size();
        for (int i = 1; i < n; i++)
            if (!(s[i] > s[i - 1]))
                throw std::invalid_argument("Spline1D: s must increase");
        a_ = y;

        std::vector<double> h(n - 1);
        for (int i = 0; i < n - 1; i++)
            h[i] = s[i + 1] - s[i];

        // 삼대각(토마스 알고리즘)
        std::vector<double> A(n, 0.0), B(n, 0.0), C(n, 0.0), D(n, 0.0);
        A[0] = 1.0;
        C[0] = 0.0;
        D[0] = 0.0;
        A[n - 1] = 1.0;
        B[n - 1] = 0.0;
        D[n - 1] = 0.0;
        for (int i = 1; i <= n - 2; i++)
        {
            A[i] = 2.0 * (h[i - 1] + h[i]);
            B[i] = h[i];
            C[i] = h[i - 1];
            D[i] = 3.0 * ((a_[i + 1] - a_[i]) / h[i] - (a_[i] - a_[i - 1]) / h[i - 1]);
        }
        for (int i = 1; i < n; i++)
        {
            double w = C[i] / A[i - 1];
            A[i] -= w * B[i - 1];
            D[i] -= w * D[i - 1];
        }
        c_.assign(n, 0.0);
        c_[n - 1] = D[n - 1] / A[n - 1];
        for (int i = n - 2; i >= 0; i--)
            c_[i] = (D[i] - B[i] * c_[i + 1]) / A[i];

        b_.assign(n - 1, 0.0);
        d_.assign(n - 1, 0.0);
        for (int i = 0; i < n - 1; i++)
        {
            b_[i] = (a_[i + 1] - a_[i]) / h[i] - h[i] * (2.0 * c_[i] + c_[i + 1]) / 3.0;
            d_[i] = (c_[i + 1] - c_[i]) / (3.0 * h[i]);
        }
    }

    int Spline1D::search(double t) const
    {
        int lo = 0, hi = (int)s_.size() - 1;
        if (t <= s_.front())
            return 0;
        if (t >= s_.back())
            return (int)s_.size() - 2;
        while (hi - lo > 1)
        {
            int mid = (lo + hi) / 2;
            if (s_[mid] <= t)
                lo = mid;
            else
                hi = mid;
        }
        return lo;
    }

    double Spline1D::calc(double t) const
    {
        int i = search(t);
        double dx = t - s_[i];
        return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
    }
    double Spline1D::calcd(double t) const
    {
        int i = search(t);
        double dx = t - s_[i];
        return b_[i] + 2.0 * c_[i] * dx + 3.0 * d_[i] * dx * dx;
    }
    double Spline1D::calcdd(double t) const
    {
        int i = search(t);
        double dx = t - s_[i];
        return 2.0 * c_[i] + 6.0 * d_[i] * dx;
    }

    // --- Spline2D ---
    std::vector<double> Spline2D::cumulativeS(const geometry_msgs::PoseArray &centers)
    {
        std::vector<double> s;
        s.reserve(centers.poses.size());
        if (centers.poses.size() < 2)
            return s;
        s.push_back(0.0);
        for (size_t i = 1; i < centers.poses.size(); ++i)
        {
            const auto &p0 = centers.poses[i - 1].position;
            const auto &p1 = centers.poses[i].position;
            s.push_back(s.back() + std::hypot(p1.x - p0.x, p1.y - p0.y));
        }
        return s;
    }

    Spline2D::Spline2D(const geometry_msgs::PoseArray &centers)
        : sx_([&]
              {
    auto s = cumulativeS(centers);
    if(s.size()<2) throw std::invalid_argument("Spline2D: need >=2 points");
    std::vector<double> X(s.size());
    for(size_t i=0;i<s.size();++i) X[i]=centers.poses[i].position.x;
    return Spline1D(s,X); }()),
          sy_([&]
              {
    auto s = cumulativeS(centers);
    std::vector<double> Y(s.size());
    for(size_t i=0;i<s.size();++i) Y[i]=centers.poses[i].position.y;
    return Spline1D(s,Y); }())
    {
    }

    geometry_msgs::Point Spline2D::position(double s) const
    {
        geometry_msgs::Point p;
        double ss = clamp(s, sx_.s_min(), sx_.s_max());
        p.x = sx_.calc(ss);
        p.y = sy_.calc(ss);
        p.z = 0.0;
        return p;
    }

    void Spline2D::derivatives(double s, double &dx, double &dy, double &ddx, double &ddy) const
    {
        double ss = clamp(s, sx_.s_min(), sx_.s_max());
        dx = sx_.calcd(ss);
        dy = sy_.calcd(ss);
        ddx = sx_.calcdd(ss);
        ddy = sy_.calcdd(ss);
    }

    double Spline2D::curvature(double s) const
    {
        double dx, dy, ddx, ddy;
        derivatives(s, dx, dy, ddx, ddy);
        double denom = std::pow(dx * dx + dy * dy, 1.5);
        if (denom < 1e-9)
            return 0.0;
        return (dx * ddy - dy * ddx) / denom;
    }

    geometry_msgs::PoseArray Spline2D::samplePath(double ds, std::vector<double> *out_kappa) const
    {
        geometry_msgs::PoseArray path;
        if (ds <= 0.0)
            return path;
        const double s0 = sx_.s_min(), s1 = sx_.s_max();
        const int N = std::max(2, (int)std::ceil((s1 - s0) / ds) + 1);
        path.poses.resize(N);
        if (out_kappa)
            out_kappa->assign(N, 0.0);
        double s = s0;
        for (int i = 0; i < N; i++, s = std::min(s + ds, s1))
        {
            path.poses[i].position = position(s);
            if (out_kappa)
                (*out_kappa)[i] = curvature(s);
        }
        return path;
    }

    geometry_msgs::Point Spline2D::pointAtArc(double targetArc, int) const
    {
        return position(targetArc); // s는 누적 호길이
    }

    // --- 가중 목표 + 경로 반환 ---
    WeightedGoalResult
    ComputeWeightedGoalAdaptive(const geometry_msgs::PoseArray &centers,
                                double base_lookahead,
                                double min_lookahead,
                                double max_lookahead,
                                double kappa_gain,
                                double weight_min,
                                double weight_max,
                                double sampling_ds,
                                bool use_abs_kappa,
                                int max_iter,
                                double tol,
                                double damping,
                                int conservative_samples)
    {
        if (centers.poses.size() < 2)
            throw std::invalid_argument("ComputeWeightedGoalAdaptive: need >=2 centers");

        Spline2D sp(centers);
        const double s0 = sp.s_min();
        const double sMax = sp.s_max() - s0;

        double s = clamp(base_lookahead, min_lookahead, std::min(max_lookahead, sMax));

        auto eval_kappa = [&](double arc) -> double
        {
            double a = clamp(arc, 0.0, sMax);
            if (conservative_samples > 0)
            {
                double kmax = 0.0;
                for (int i = 0; i <= conservative_samples; i++)
                {
                    double si = (a * i) / std::max(1, conservative_samples);
                    double k = sp.curvature(s0 + si);
                    kmax = std::max(kmax, std::fabs(k));
                }
                return kmax;
            }
            else
            {
                double k = sp.curvature(s0 + a);
                return use_abs_kappa ? std::fabs(k) : k;
            }
        };

        for (int it = 0; it < max_iter; ++it)
        {
            double k = eval_kappa(s);
            double w = 1.0 - kappa_gain * k;      // 음수 가능
            w = clamp(w, weight_min, weight_max); // 파라미터화
            double s_target = clamp(base_lookahead * w, min_lookahead, std::min(max_lookahead, sMax));
            double s_new = s + damping * (s_target - s);
            if (std::fabs(s_new - s) < tol)
            {
                s = s_new;
                break;
            }
            s = s_new;
        }

        geometry_msgs::PoseArray path = sp.samplePath(sampling_ds, nullptr);
        geometry_msgs::Point target = sp.pointAtArc(s);
        double kappa_at_target = sp.curvature(s0 + s);

        return WeightedGoalResult{target, kappa_at_target, s, path};
    }

    // 레거시 호환
    WeightedGoalResult
    ComputeWeightedGoal(const geometry_msgs::PoseArray &centers,
                        double base_lookahead,
                        double min_lookahead,
                        double max_lookahead,
                        double kappa_gain,
                        double sampling_ds)
    {
        return ComputeWeightedGoalAdaptive(centers,
                                           base_lookahead,
                                           min_lookahead,
                                           max_lookahead,
                                           kappa_gain,
                                           /*weight_min=*/0.2,
                                           /*weight_max=*/1.0,
                                           /*sampling_ds=*/sampling_ds,
                                           /*use_abs_kappa=*/true,
                                           /*max_iter=*/5,
                                           /*tol=*/0.05,
                                           /*damping=*/0.5,
                                           /*conservative_samples=*/0);
    }

    // 경로 생성 헬퍼
    geometry_msgs::PoseArray GenerateSplinePath(const geometry_msgs::PoseArray &centers,
                                                double ds,
                                                std::vector<double> *out_kappa)
    {
        Spline2D sp(centers);
        return sp.samplePath(ds, out_kappa);
    }

} // namespace spline