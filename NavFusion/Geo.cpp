#include "Geo.h"

#include <algorithm>
#include <cmath>

/*
 * Geo.cpp
 * -------
 * 地理坐标工具。
 *
 * GNSS 解算自然得到的是 ECEF 地心地固坐标；
 * 但画轨迹图时，直接画几百万米量级的 ECEF 不直观。
 * 因此绘图时会选第一个轨迹点作为参考点，
 * 把 ECEF 差分转换成局部 NED，再用 East/North 平面画轨迹。
 */

namespace {

// WGS-84椭球参数。
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);

} // namespace

Vec3 llhToEcef(double latRad, double lonRad, double heightM) {
    // 大地坐标(纬度、经度、高程)转 ECEF 直角坐标。
    const double s = std::sin(latRad);
    const double c = std::cos(latRad);
    const double rn = kA / std::sqrt(1.0 - kE2 * s * s);
    return {
        (rn + heightM) * c * std::cos(lonRad),
        (rn + heightM) * c * std::sin(lonRad),
        (rn * (1.0 - kE2) + heightM) * s
    };
}

void ecefToLlh(const Vec3& ecef, double& latRad, double& lonRad, double& heightM) {
    // ECEF 反算经纬高。固定迭代次数已经能满足绘图和初值计算。
    lonRad = std::atan2(ecef.y, ecef.x);
    const double p = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
    latRad = std::atan2(ecef.z, p * (1.0 - kE2));
    for (int i = 0; i < 8; ++i) {
        const double s = std::sin(latRad);
        const double rn = kA / std::sqrt(1.0 - kE2 * s * s);
        heightM = p / std::max(0.1, std::cos(latRad)) - rn;
        latRad = std::atan2(ecef.z, p * (1.0 - kE2 * rn / (rn + heightM)));
    }
}

Vec3 ecefToNed(const Vec3& ecefDiff, double refLatRad, double refLonRad) {
    // 把 ECEF 差分向量转换到局部 NED 坐标系，返回 x=North、y=East、z=Down。
    const double sl = std::sin(refLatRad);
    const double cl = std::cos(refLatRad);
    const double sb = std::sin(refLonRad);
    const double cb = std::cos(refLonRad);
    return {
        -sl * cb * ecefDiff.x - sl * sb * ecefDiff.y + cl * ecefDiff.z,
        -sb * ecefDiff.x + cb * ecefDiff.y,
        -cl * cb * ecefDiff.x - cl * sb * ecefDiff.y - sl * ecefDiff.z
    };
}

double normalGravity(double latRad, double heightM) {
    // 正常重力模型，完善惯导机械编排时会用到。
    const double s = std::sin(latRad);
    const double g0 = 9.7803253359 * (1.0 + 0.00193185265241 * s * s) /
                      std::sqrt(1.0 - kE2 * s * s);
    return g0 - 3.086e-6 * heightM;
}
