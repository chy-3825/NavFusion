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

double axisValue(const Vec3& v, int axis) {
    if (axis == 0) return v.x;
    if (axis == 1) return v.y;
    return v.z;
}

void setAxis(Vec3& v, int axis, double value) {
    if (axis == 0) v.x = value;
    else if (axis == 1) v.y = value;
    else v.z = value;
}

double nominalSampleInterval(const std::vector<ImuSample>& imu) {
    double sum = 0.0;
    int count = 0;
    const size_t limit = std::min<size_t>(imu.size(), 2000);
    for (size_t i = 1; i < limit; ++i) {
        const double dt = imu[i].time - imu[i - 1].time;
        if (dt > 0.0 && dt < 1.0) {
            sum += dt;
            ++count;
        }
    }
    return count > 0 ? sum / static_cast<double>(count) : 0.005;
}

std::vector<ImuSample> selectAllanSamples(const std::vector<ImuSample>& imu) {
    if (imu.size() < 4) return imu;

    const double dt = nominalSampleInterval(imu);
    const size_t windowSamples = std::max<size_t>(100, static_cast<size_t>(std::round(30.0 / dt)));
    const size_t minSamples = std::max<size_t>(windowSamples, static_cast<size_t>(std::round(60.0 / dt)));
    if (imu.size() < minSamples || windowSamples == 0) return imu;

    auto windowMean = [&](size_t first, size_t last, bool gyro) {
        double sum = 0.0;
        size_t count = 0;
        last = std::min(last, imu.size());
        for (size_t i = first; i < last; ++i) {
            sum += norm(gyro ? imu[i].gyro : imu[i].acc);
            ++count;
        }
        return count > 0 ? sum / static_cast<double>(count) : 0.0;
    };

    const double baseGyro = windowMean(0, windowSamples, true);
    const double baseAcc = windowMean(0, windowSamples, false);
    const double gyroLimit = std::max(0.5, baseGyro * 4.0);
    const double accDeltaLimit = 0.05;

    size_t keepEnd = 0;
    for (size_t first = 0; first + windowSamples <= imu.size(); first += windowSamples) {
        const size_t last = first + windowSamples;
        const double meanGyro = windowMean(first, last, true);
        const double meanAcc = windowMean(first, last, false);
        if (first > 0 && (meanGyro > gyroLimit || std::abs(meanAcc - baseAcc) > accDeltaLimit)) {
            break;
        }
        keepEnd = last;
    }

    if (keepEnd >= minSamples && keepEnd < imu.size()) {
        return std::vector<ImuSample>(imu.begin(), imu.begin() + keepEnd);
    }
    return imu;
}

double allanVarianceForAxis(const std::vector<ImuSample>& imu,
                            size_t samplesPerCluster,
                            bool gyro,
                            int axis) {
    const size_t clusterCount = imu.size() / samplesPerCluster;
    if (clusterCount < 2) return 0.0;

    std::vector<double> averages(clusterCount, 0.0);
    for (size_t c = 0; c < clusterCount; ++c) {
        double sum = 0.0;
        const size_t first = c * samplesPerCluster;
        for (size_t i = 0; i < samplesPerCluster; ++i) {
            const Vec3& v = gyro ? imu[first + i].gyro : imu[first + i].acc;
            sum += axisValue(v, axis);
        }
        averages[c] = sum / static_cast<double>(samplesPerCluster);
    }

    double diffSum = 0.0;
    for (size_t c = 0; c + 1 < clusterCount; ++c) {
        const double diff = averages[c + 1] - averages[c];
        diffSum += diff * diff;
    }
    return 0.5 * diffSum / static_cast<double>(clusterCount - 1);
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
    double yaw = 0.0;
    Trajectory out;
    out.reserve(imu.size());
    out.push_back({ imu.front().time, pos, 5 });

    for (size_t i = 1; i < imu.size(); ++i) {
        const double dt = std::max(1.0e-4, std::min(0.02, imu[i].time - imu[i - 1].time));
        Vec3 acc = sub(imu[i].acc, accBias);
        const double aNorm = norm(acc);
        if (aNorm > 8.0) {
            acc = scale(acc, 8.0 / aNorm);
        }

        yaw += imu[i].gyro.z * dt;
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);
        const Vec3 navAcc{
            cy * acc.x - sy * acc.y,
            sy * acc.x + cy * acc.y,
            acc.z
        };

        pos = add(pos, add(scale(vel, dt), scale(navAcc, 0.5 * dt * dt)));
        vel = add(vel, scale(navAcc, dt));
        out.push_back({ imu[i].time, pos, 5 });
    }
    return out;
}

AllanSeries computeAllanVariance(const std::vector<ImuSample>& imu) {
    AllanSeries out;
    const std::vector<ImuSample> allanInput = selectAllanSamples(imu);
    if (allanInput.size() < 4) return out;

    const double dt = nominalSampleInterval(allanInput);
    for (size_t m = 1; m <= allanInput.size() / 4; m *= 2) {
        AllanPoint p;
        p.samplesPerCluster = static_cast<int>(m);
        p.tau = dt * static_cast<double>(m);
        for (int axis = 0; axis < 3; ++axis) {
            const double gv = allanVarianceForAxis(allanInput, m, true, axis);
            const double av = allanVarianceForAxis(allanInput, m, false, axis);
            setAxis(p.gyroVariance, axis, gv);
            setAxis(p.accVariance, axis, av);
            setAxis(p.gyroDeviation, axis, std::sqrt(std::max(0.0, gv)));
            setAxis(p.accDeviation, axis, std::sqrt(std::max(0.0, av)));
        }
        out.push_back(p);
        if (m > static_cast<size_t>(1) << 25) break;
    }
    return out;
}
