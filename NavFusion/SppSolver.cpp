#include "SppSolver.h"

#include <cmath>
#include <vector>

/*
 * SppSolver.cpp
 * -------------
 * SPP（Single Point Positioning，单点定位）核心思路：
 *
 * 对每个历元，使用多颗卫星的伪距观测建立方程：
 *   伪距 = 接收机到卫星的几何距离 + 接收机钟差 - 卫星钟差修正 + 误差项
 *
 * 未知量是接收机三维坐标改正量和接收机钟差，因此至少需要 4 颗卫星。
 * 程序逐历元迭代线性化伪距方程，用最小二乘求出接收机 ECEF 坐标。
 *
 * 当前使用 GPS C1C 单频伪距，暂未加入电离层/对流层改正。
 */

namespace {

constexpr double kC = 299792458.0;
constexpr double kMu = 3.986005e14;
constexpr double kOmegaE = 7.2921151467e-5;
constexpr double kRelF = -4.442807633e-10;
constexpr double kPiLocal = 3.1415926535897932384626433832795;

double wrapGpsDt(double dt) {
    // 处理 GPS 跨周时间差，让时间差落在半周范围内，避免星历匹配时出现 604800 秒跳变。
    while (dt > 302400.0) dt -= 604800.0;
    while (dt < -302400.0) dt += 604800.0;
    return dt;
}

double norm(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 rotateEarth(const Vec3& p, double travelTime) {
    // Sagnac 地球自转改正：信号传播期间地球会自转，卫星坐标需要旋转到接收时刻。
    const double ang = kOmegaE * travelTime;
    const double ca = std::cos(ang);
    const double sa = std::sin(ang);
    return { ca * p.x + sa * p.y, -sa * p.x + ca * p.y, p.z };
}

const GpsEphemeris* selectEph(const std::vector<GpsEphemeris>& ephs, int prn, double tow) {
    // 为指定 PRN 选择 toe 最接近当前历元的广播星历。
    const GpsEphemeris* best = nullptr;
    double bestDt = 1e99;
    for (const auto& e : ephs) {
        if (e.prn != prn) continue;
        const double dt = std::abs(wrapGpsDt(tow - e.toe));
        if (dt < bestDt) {
            bestDt = dt;
            best = &e;
        }
    }
    return bestDt <= 7200.0 ? best : nullptr;
}

bool satPosClock(const GpsEphemeris& e, double txTow, Vec3& pos, double& clk) {
    // 根据 GPS 广播星历计算卫星发射时刻的 ECEF 坐标和卫星钟差。
    const double a = e.sqrtA * e.sqrtA;
    if (a <= 0.0) return false;

    const double n0 = std::sqrt(kMu / (a * a * a));
    const double tk = wrapGpsDt(txTow - e.toe);
    double m = std::fmod(e.m0 + (n0 + e.dn) * tk, 2.0 * kPiLocal);
    double ek = m;
    for (int i = 0; i < 12; ++i) ek = m + e.e * std::sin(ek);

    const double sinE = std::sin(ek);
    const double cosE = std::cos(ek);
    const double vk = std::atan2(std::sqrt(1.0 - e.e * e.e) * sinE, cosE - e.e);
    const double phik = vk + e.omega;
    const double du = e.cus * std::sin(2.0 * phik) + e.cuc * std::cos(2.0 * phik);
    const double dr = e.crs * std::sin(2.0 * phik) + e.crc * std::cos(2.0 * phik);
    const double di = e.cis * std::sin(2.0 * phik) + e.cic * std::cos(2.0 * phik);
    const double u = phik + du;
    const double r = a * (1.0 - e.e * cosE) + dr;
    const double i = e.i0 + di + e.idot * tk;
    const double xOrb = r * std::cos(u);
    const double yOrb = r * std::sin(u);
    const double omega = e.omega0 + (e.omegaDot - kOmegaE) * tk - kOmegaE * e.toe;
    const double co = std::cos(omega);
    const double so = std::sin(omega);
    const double ci = std::cos(i);
    const double si = std::sin(i);
    pos = { xOrb * co - yOrb * ci * so, xOrb * so + yOrb * ci * co, yOrb * si };

    const double dt = wrapGpsDt(txTow - e.toc);
    clk = e.af0 + e.af1 * dt + e.af2 * dt * dt + kRelF * e.e * e.sqrtA * sinE - e.tgd;
    return true;
}

bool solve4x4(double a[4][4], double b[4], double x[4]) {
    // 使用高斯消元解 4x4 线性方程组，对应未知量 [dX,dY,dZ,dClock]。
    double m[4][5]{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) m[i][j] = a[i][j];
        m[i][4] = b[i];
    }

    for (int c = 0; c < 4; ++c) {
        int piv = c;
        for (int r = c + 1; r < 4; ++r) {
            if (std::abs(m[r][c]) > std::abs(m[piv][c])) piv = r;
        }
        if (std::abs(m[piv][c]) < 1e-12) return false;
        if (piv != c) {
            for (int j = c; j < 5; ++j) std::swap(m[piv][j], m[c][j]);
        }
        const double div = m[c][c];
        for (int j = c; j < 5; ++j) m[c][j] /= div;
        for (int r = 0; r < 4; ++r) {
            if (r == c) continue;
            const double f = m[r][c];
            for (int j = c; j < 5; ++j) m[r][j] -= f * m[c][j];
        }
    }

    for (int i = 0; i < 4; ++i) x[i] = m[i][4];
    return true;
}

bool solveEpoch(const RinexEpoch& epoch,
                const std::vector<GpsEphemeris>& ephs,
                Vec3& rec,
                double& cb) {
    // 单历元最小二乘定位。上一历元的位置作为当前初值，可以加快收敛。
    if (epoch.gps.size() < 4) return false;

    for (int iter = 0; iter < 8; ++iter) {
        double n[4][4]{};
        double u[4]{};
        int used = 0;

        for (const auto& kv : epoch.gps) {
            const int prn = kv.first;
            const double pr = kv.second.c1;
            if (pr <= 1.0e6) continue;

            const GpsEphemeris* eph = selectEph(ephs, prn, epoch.tow);
            if (!eph) continue;

            Vec3 sat{};
            double satClk = 0.0;
            const double tx = epoch.tow - pr / kC;
            if (!satPosClock(*eph, tx, sat, satClk)) continue;

            const double rho0 = norm({ sat.x - rec.x, sat.y - rec.y, sat.z - rec.z });
            sat = rotateEarth(sat, rho0 / kC);
            const Vec3 diff{ sat.x - rec.x, sat.y - rec.y, sat.z - rec.z };
            const double rho = norm(diff);
            if (rho <= 0.0) continue;

            const double h[4] = { -diff.x / rho, -diff.y / rho, -diff.z / rho, 1.0 };
            const double v = pr - (rho + cb - kC * satClk);
            for (int r = 0; r < 4; ++r) {
                u[r] += h[r] * v;
                for (int c = 0; c < 4; ++c) n[r][c] += h[r] * h[c];
            }
            ++used;
        }

        if (used < 4) return false;

        double step[4]{};
        if (!solve4x4(n, u, step)) return false;
        rec.x += step[0];
        rec.y += step[1];
        rec.z += step[2];
        cb += step[3];
        if (std::sqrt(step[0] * step[0] + step[1] * step[1] + step[2] * step[2]) < 1e-3) return true;
    }
    return true;
}

} // namespace

Trajectory solveSpp(const RinexObservationFile& roverObs, const RinexNavigationFile& nav) {
    // SPP 主入口：遍历所有观测历元，能成功解算的点组成最终轨迹。
    if (roverObs.epochs.empty() || nav.gpsEphemerides.empty()) return {};

    Vec3 rec = roverObs.approxPosition;
    double cb = 0.0;
    Trajectory out;
    out.reserve(roverObs.epochs.size());

    for (const auto& epoch : roverObs.epochs) {
        Vec3 solved = rec;
        double solvedCb = cb;
        if (!solveEpoch(epoch, nav.gpsEphemerides, solved, solvedCb)) continue;
        rec = solved;
        cb = solvedCb;
        out.push_back({ epoch.tow, solved, 1 });
    }
    return out;
}
