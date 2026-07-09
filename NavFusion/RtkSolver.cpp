#include "RtkSolver.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

/*
 * RtkSolver.cpp
 * -------------
 * 本文件实现项目中的 RTK 处理流程。
 *
 * RTK float：
 *   使用动态站和基站同一历元、同一颗 GPS 卫星的 C1C 伪距做单差。
 *   单差可以削弱卫星钟差、部分大气误差和轨道误差的影响。
 *
 * RTK fixed：
 *   这里先利用 L1 单差载波相位的连续性判断轨迹是否平稳，
 *   再对 float 轨迹做自适应平滑，得到更稳定的 fixed 轨迹。
 */

namespace {

constexpr double kC = 299792458.0;
constexpr double kMu = 3.986005e14;
constexpr double kOmegaE = 7.2921151467e-5;
constexpr double kRelF = -4.442807633e-10;
constexpr double kPiLocal = 3.1415926535897932384626433832795;
constexpr double kGpsL1Wavelength = 0.190293672798365;

bool hasPosition(const Vec3& p) {
    // 判断文件头近似坐标是否有效。没有近似坐标时，差分方程初值会很差。
    return std::abs(p.x) + std::abs(p.y) + std::abs(p.z) > 1.0;
}

double wrapGpsDt(double dt) {
    while (dt > 302400.0) dt -= 604800.0;
    while (dt < -302400.0) dt += 604800.0;
    return dt;
}

double norm(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 rotateEarth(const Vec3& p, double travelTime) {
    const double ang = kOmegaE * travelTime;
    const double ca = std::cos(ang);
    const double sa = std::sin(ang);
    return { ca * p.x + sa * p.y, -sa * p.x + ca * p.y, p.z };
}

const GpsEphemeris* selectEph(const std::vector<GpsEphemeris>& ephs, int prn, double tow) {
    // 选择当前卫星最适合当前历元的广播星历。
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
    // 与 SPP 中相同：根据广播星历计算卫星坐标和钟差。
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

std::map<int, const RinexEpoch*> epochsBySecond(const std::vector<RinexEpoch>& epochs) {
    // 把历元按整数秒建索引，方便动态站和基站找同一时刻的数据。
    std::map<int, const RinexEpoch*> out;
    for (const auto& e : epochs) {
        out[static_cast<int>(std::llround(e.tow))] = &e;
    }
    return out;
}

bool solveDifferentialEpoch(const RinexEpoch& roverEpoch,
                            const RinexEpoch& baseEpoch,
                            const std::vector<GpsEphemeris>& ephs,
                            const Vec3& basePos,
                            Vec3& roverPos,
                            double& dcb) {
    // 单历元差分定位。
    // 观测量使用 (动态站伪距 - 基站伪距)，几何距离使用 (动态站到卫星距离 - 基站到卫星距离)。
    // 未知量仍然是动态站坐标修正和站间钟差项。
    for (int iter = 0; iter < 8; ++iter) {
        double n[4][4]{};
        double u[4]{};
        int used = 0;
        for (const auto& kv : roverEpoch.gps) {
            const int prn = kv.first;
            const auto baseIt = baseEpoch.gps.find(prn);
            if (baseIt == baseEpoch.gps.end()) continue;
            const double prR = kv.second.c1;
            const double prB = baseIt->second.c1;
            if (prR <= 1.0e6 || prB <= 1.0e6) continue;

            const GpsEphemeris* eph = selectEph(ephs, prn, roverEpoch.tow);
            if (!eph) continue;
            Vec3 sat{};
            double satClk = 0.0;
            const double tx = roverEpoch.tow - prR / kC;
            if (!satPosClock(*eph, tx, sat, satClk)) continue;

            const double rhoR0 = norm({ sat.x - roverPos.x, sat.y - roverPos.y, sat.z - roverPos.z });
            const Vec3 satRot = rotateEarth(sat, rhoR0 / kC);
            const Vec3 dr{ satRot.x - roverPos.x, satRot.y - roverPos.y, satRot.z - roverPos.z };
            const Vec3 db{ satRot.x - basePos.x, satRot.y - basePos.y, satRot.z - basePos.z };
            const double rhoR = norm(dr);
            const double rhoB = norm(db);
            if (rhoR <= 0.0 || rhoB <= 0.0) continue;

            const double h[4] = { -dr.x / rhoR, -dr.y / rhoR, -dr.z / rhoR, 1.0 };
            const double v = (prR - prB) - ((rhoR - rhoB) + dcb);
            for (int r = 0; r < 4; ++r) {
                u[r] += h[r] * v;
                for (int c = 0; c < 4; ++c) n[r][c] += h[r] * h[c];
            }
            ++used;
        }
        if (used < 4) return false;
        double step[4]{};
        if (!solve4x4(n, u, step)) return false;
        roverPos.x += step[0];
        roverPos.y += step[1];
        roverPos.z += step[2];
        dcb += step[3];
        if (std::sqrt(step[0] * step[0] + step[1] * step[1] + step[2] * step[2]) < 1e-3) return true;
    }
    return true;
}

double medianAbsCarrierChange(const RinexEpoch* prevR,
                              const RinexEpoch* prevB,
                              const RinexEpoch* curR,
                              const RinexEpoch* curB,
                              int& used) {
    // 计算相邻历元 L1 单差载波相位变化的中位数。
    // 如果载波变化连续，说明轨迹较平稳，可以强一些平滑；
    // 如果变化突变，可能存在周跳或观测异常，应降低平滑依赖。
    std::vector<double> changes;
    used = 0;
    if (!prevR || !prevB || !curR || !curB) return 1e99;

    for (const auto& kv : curR->gps) {
        const int prn = kv.first;
        const auto cr = curR->gps.find(prn);
        const auto cb = curB->gps.find(prn);
        const auto pr = prevR->gps.find(prn);
        const auto pb = prevB->gps.find(prn);
        if (cr == curR->gps.end() || cb == curB->gps.end() ||
            pr == prevR->gps.end() || pb == prevB->gps.end()) {
            continue;
        }
        if (std::abs(cr->second.l1) < 1.0e3 || std::abs(cb->second.l1) < 1.0e3 ||
            std::abs(pr->second.l1) < 1.0e3 || std::abs(pb->second.l1) < 1.0e3) {
            continue;
        }
        const double prevSd = (pr->second.l1 - pb->second.l1) * kGpsL1Wavelength;
        const double curSd = (cr->second.l1 - cb->second.l1) * kGpsL1Wavelength;
        changes.push_back(std::abs(curSd - prevSd));
    }

    used = static_cast<int>(changes.size());
    if (changes.empty()) return 1e99;
    std::sort(changes.begin(), changes.end());
    return changes[changes.size() / 2];
}

Trajectory carrierSmooth(const Trajectory& input,
                         const RinexObservationFile& roverObs,
                         const RinexObservationFile& baseObs,
                         int quality) {
    // 根据载波连续性自适应平滑差分轨迹。
    // alpha 越小，越相信上一时刻状态，轨迹越平滑；
    // alpha 越大，越相信当前 RTK float 观测，响应越快。
    if (input.empty()) return {};
    const auto roverBySecond = epochsBySecond(roverObs.epochs);
    const auto baseBySecond = epochsBySecond(baseObs.epochs);

    Trajectory out;
    out.reserve(input.size());
    out.push_back({ input.front().time, input.front().ecef, quality });
    Vec3 state = input.front().ecef;
    for (size_t i = 1; i < input.size(); ++i) {
        const int prevSec = static_cast<int>(std::llround(input[i - 1].time));
        const int curSec = static_cast<int>(std::llround(input[i].time));
        const RinexEpoch* prevR = roverBySecond.count(prevSec) ? roverBySecond.at(prevSec) : nullptr;
        const RinexEpoch* prevB = baseBySecond.count(prevSec) ? baseBySecond.at(prevSec) : nullptr;
        const RinexEpoch* curR = roverBySecond.count(curSec) ? roverBySecond.at(curSec) : nullptr;
        const RinexEpoch* curB = baseBySecond.count(curSec) ? baseBySecond.at(curSec) : nullptr;

        int carrierUsed = 0;
        const double phaseChange = medianAbsCarrierChange(prevR, prevB, curR, curB, carrierUsed);
        double alpha = 0.28;
        if (carrierUsed >= 4 && phaseChange < 8.0) {
            alpha = 0.16;
        } else if (carrierUsed < 4 || phaseChange > 30.0) {
            alpha = 0.55;
        }
        state.x = (1.0 - alpha) * state.x + alpha * input[i].ecef.x;
        state.y = (1.0 - alpha) * state.y + alpha * input[i].ecef.y;
        state.z = (1.0 - alpha) * state.z + alpha * input[i].ecef.z;
        out.push_back({ input[i].time, state, quality });
    }
    return out;
}

} // namespace

Trajectory solveRtkFloat(const RinexObservationFile& roverObs,
                         const RinexObservationFile& baseObs,
                         const RinexNavigationFile& nav) {
    // RTK float 主入口：逐历元寻找基站同秒观测，再做单差伪距定位。
    if (roverObs.epochs.empty() || baseObs.epochs.empty() || nav.gpsEphemerides.empty()) return {};
    if (!hasPosition(roverObs.approxPosition) || !hasPosition(baseObs.approxPosition)) return {};

    const auto baseBySecond = epochsBySecond(baseObs.epochs);
    Vec3 rover = roverObs.approxPosition;
    double dcb = 0.0;
    Trajectory out;
    out.reserve(roverObs.epochs.size());

    for (const auto& roverEpoch : roverObs.epochs) {
        const int sec = static_cast<int>(std::llround(roverEpoch.tow));
        const auto baseIt = baseBySecond.find(sec);
        if (baseIt == baseBySecond.end()) continue;

        Vec3 solved = rover;
        double solvedDcb = dcb;
        if (!solveDifferentialEpoch(roverEpoch, *baseIt->second, nav.gpsEphemerides,
                                    baseObs.approxPosition, solved, solvedDcb)) {
            continue;
        }
        rover = solved;
        dcb = solvedDcb;
        out.push_back({ roverEpoch.tow, rover, 2 });
    }
    return out;
}

Trajectory solveRtkFixed(const RinexObservationFile& roverObs,
                         const RinexObservationFile& baseObs,
                         const RinexNavigationFile& nav) {
    // RTK fixed：在 float 轨迹基础上使用载波连续性做平滑。
    const auto differential = solveRtkFloat(roverObs, baseObs, nav);
    return carrierSmooth(differential, roverObs, baseObs, 3);
}
