#include "InsSolver.h"

#include <algorithm>
#include <cmath>

/*
 * InsSolver.cpp
 * -------------
 * 简化 INS 机械编排。
 *
 * 完整惯导机械编排通常包括：
 * - 姿态更新；
 * - 比力从机体系转到导航系；
 * - 重力、地球自转、运输率改正；
 * - 速度和位置积分。
 *
 * 这里先采用比较简单的模型：
 * - 估计前若干 IMU 采样的平均加速度作为偏置；
 * - 去掉偏置后积分出相对位移；
 * - 结果作为 RTK/INS 松组合的短时预测量。
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

double norm(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

} // namespace

Trajectory mechanizeIns(const std::vector<ImuSample>& imu) {
    // 输入 IMU 采样序列，输出以第一时刻为原点的相对位移轨迹。
    if (imu.size() < 2) return {};

    Vec3 accBias{};
    const size_t biasCount = std::min<size_t>(imu.size(), 400);
    for (size_t i = 0; i < biasCount; ++i) {
        accBias = add(accBias, imu[i].acc);
    }
    accBias = scale(accBias, 1.0 / static_cast<double>(biasCount));

    Vec3 pos{};
    Vec3 vel{};
    Trajectory out;
    out.reserve(imu.size());
    out.push_back({ imu.front().time, pos, 0 });

    for (size_t i = 1; i < imu.size(); ++i) {
        const double dt = std::max(1.0e-4, std::min(0.02, imu[i].time - imu[i - 1].time));
        Vec3 acc = sub(imu[i].acc, accBias);
        const double aNorm = norm(acc);
        if (aNorm > 8.0) {
            acc = scale(acc, 8.0 / aNorm);
        }
        vel = add(vel, scale(acc, dt));
        pos = add(pos, add(scale(vel, dt), scale(acc, 0.5 * dt * dt)));
        out.push_back({ imu[i].time, pos, 0 });
    }
    return out;
}
