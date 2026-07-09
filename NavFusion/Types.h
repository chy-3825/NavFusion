#pragma once

/*
 * Types.h
 * -------
 * 本文件集中定义整个程序会反复使用的数据结构。
 *
 * 程序的数据流可以粗略理解为：
 * RINEX/IMR 文件 -> Reader 解析成结构体 -> Solver 生成 Trajectory -> Plot 输出图和 CSV。
 *
 * 把这些基础类型集中放在一个头文件里，可以避免 SPP、RTK、INS、绘图模块之间互相重复定义。
 */

#include <map>
#include <string>
#include <vector>

// 三维向量。项目中用它统一表示 ECEF 坐标、NED 坐标、速度、加速度、角速度等三轴量。
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 四元数姿态。当前 INS 先做相对积分，后续姿态更新会用到。
struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 单个 IMU 采样点。
// time 使用 GPS 周内秒；gyro/acc 是从 IMR 原始增量换算得到的角速度和加速度。
struct ImuSample {
    double time = 0.0;
    Vec3 gyro;
    Vec3 acc;
};

// 单颗 GPS 卫星在某一历元的观测值。
// c1 是 C1C 伪距，单位米；l1 是 L1C 载波相位，单位周。
struct RinexObsValue {
    double c1 = 0.0;
    double l1 = 0.0;
};

// 一个 RINEX 观测历元。
// tow 是 GPS 周内秒；gps 的 key 是 GPS PRN，value 是该卫星的观测值。
struct RinexEpoch {
    double tow = 0.0;
    std::map<int, RinexObsValue> gps;
};

// GPS 广播星历参数。
// 字段基本对应 RINEX 3 导航文件中一组 GPS 星历块，SPP 和 RTK 都用它计算卫星位置和卫星钟差。
struct GpsEphemeris {
    int prn = 0;
    double toc = 0.0;
    double af0 = 0.0;
    double af1 = 0.0;
    double af2 = 0.0;
    double iode = 0.0;
    double crs = 0.0;
    double dn = 0.0;
    double m0 = 0.0;
    double cuc = 0.0;
    double e = 0.0;
    double cus = 0.0;
    double sqrtA = 0.0;
    double toe = 0.0;
    double cic = 0.0;
    double omega0 = 0.0;
    double cis = 0.0;
    double i0 = 0.0;
    double crc = 0.0;
    double omega = 0.0;
    double omegaDot = 0.0;
    double idot = 0.0;
    double week = 0.0;
    double tgd = 0.0;
};

// 单个 GNSS 解算结果。当前主要使用 ecef 位置，velNed 为后续速度解算预留。
struct GnssSample {
    double time = 0.0;
    Vec3 ecef;
    Vec3 velNed;
};

// RINEX 观测文件解析结果。
// approxPosition 来自文件头 APPROX POSITION XYZ，可作为最小二乘初值。
struct RinexObservationFile {
    std::string path;
    int epochCount = 0;
    Vec3 approxPosition;
    std::vector<RinexEpoch> epochs;
};

// RINEX 导航文件解析结果。当前只读取 GPS 广播星历。
struct RinexNavigationFile {
    std::string path;
    int ephemerisCount = 0;
    std::vector<GpsEphemeris> gpsEphemerides;
};

// 轨迹点。quality 用于标记解类型：1=SPP，2=RTK float，3=RTK fixed，4=RTK/INS。
struct TrajectoryPoint {
    double time = 0.0;
    Vec3 ecef;
    int quality = 0;
};

// 一条轨迹就是按时间顺序排列的一组轨迹点。
using Trajectory = std::vector<TrajectoryPoint>;
