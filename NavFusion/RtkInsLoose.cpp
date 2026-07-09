#include "RtkInsLoose.h"

#include <algorithm>
#include <cmath>

#include "InsSolver.h"

/*
 * RtkInsLoose.cpp
 * ----------------
 * RTK/INS 松组合。
 *
 * 松组合的特点是：GNSS/RTK 和 INS 各自先独立形成结果，
 * 然后在位置/速度层面进行融合。
 *
 * 当前融合方法：
 * 1. 用 IMU 做相对位移预测；
 * 2. 用 RTK fixed 轨迹作为位置观测；
 * 3. 通过一个固定增益进行预测-更新，得到更平滑的组合轨迹。
 *
 * 这样可以体现“INS 高频预测 + RTK 低频校正”的基本思路。
 */

namespace {

Vec3 add(const Vec3& a, const Vec3& b) {
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

Vec3 sub(const Vec3& a, const Vec3& b) {
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

Vec3 scale(const Vec3& v, double s) {
    return { v.x * s, v.y * s, v.z * s };
}

Vec3 interpolateInsDelta(const Trajectory& ins, double fromTime, double toTime) {
    // 从 INS 相对轨迹中插值出 fromTime 到 toTime 之间的位移增量。
    // RTK 是 1 Hz 左右，IMU 频率更高，因此需要插值对齐时间。
    if (ins.empty() || toTime <= fromTime) return {};

    auto sampleAt = [&](double t) {
        auto it = std::lower_bound(ins.begin(), ins.end(), t, [](const TrajectoryPoint& p, double value) {
            return p.time < value;
        });
        if (it == ins.begin()) return it->ecef;
        if (it == ins.end()) return ins.back().ecef;
        const auto& b = *it;
        const auto& a = *(it - 1);
        const double span = std::max(1.0e-6, b.time - a.time);
        const double raw = (t - a.time) / span;
        const double u = std::max(0.0, std::min(1.0, raw));
        return add(scale(a.ecef, 1.0 - u), scale(b.ecef, u));
    };

    return sub(sampleAt(toTime), sampleAt(fromTime));
}

} // namespace

Trajectory solveRtkInsLoose(const Trajectory& rtkTrajectory, const std::vector<ImuSample>& imu) {
    // 松组合主入口。输出点数与 RTK 轨迹一致，便于和前几步结果对比绘图。
    if (rtkTrajectory.empty()) return {};

    const Trajectory insRelative = mechanizeIns(imu);
    Trajectory out;
    out.reserve(rtkTrajectory.size());

    Vec3 state = rtkTrajectory.front().ecef;
    out.push_back({ rtkTrajectory.front().time, state, 4 });

    for (size_t i = 1; i < rtkTrajectory.size(); ++i) {
        Vec3 predicted = state;
        if (!insRelative.empty()) {
            const Vec3 delta = interpolateInsDelta(insRelative, rtkTrajectory[i - 1].time, rtkTrajectory[i].time);
            predicted = add(predicted, scale(delta, 0.15));
        } else {
            predicted = add(predicted, sub(rtkTrajectory[i].ecef, rtkTrajectory[i - 1].ecef));
        }

        const Vec3 measurement = rtkTrajectory[i].ecef;
        constexpr double updateGain = 0.42;
        state = add(scale(predicted, 1.0 - updateGain), scale(measurement, updateGain));
        out.push_back({ rtkTrajectory[i].time, state, 4 });
    }
    return out;
}
