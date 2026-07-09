#pragma once

/*
 * Types.h
 * -------
 * 本文件集中定义整个程序反复使用的基础数据结构。
 *
 * 可以把程序的数据流理解成：
 *
 *   RINEX/IMR 原始文件
 *        ↓
 *   Reader 模块解析成下面这些结构体
 *        ↓
 *   Solver 模块使用结构体进行 SPP、RTK、INS、松组合解算
 *        ↓
 *   Plot 模块把 Trajectory 输出成 CSV、SVG、BMP
 *
 * 这些结构体不是“算法类”，更像是各模块之间传递数据的统一格式。
 */

#include <map>
#include <string>
#include <vector>

// 三维向量。
// 在本项目中，Vec3 会被复用来表示：
// - ECEF 坐标：地心地固坐标，单位 m；
// - NED 坐标：局部北东地坐标，单位 m；
// - 速度、加速度、角速度等三轴物理量。
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 四元数姿态。
// 当前 INS 主要做相对位移积分，暂时还没有完整姿态更新。
// 这个结构体是给后续扩展姿态机械编排预留的：
// w 是实部，x/y/z 是虚部。
struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 单个 IMU 采样点。
// time：GPS 周内秒。
// gyro：陀螺仪角速度，来自 IMR 原始角增量换算。
// acc：加速度计加速度，来自 IMR 原始速度/加速度增量换算。
// INS 模块会按时间顺序积分这些采样点，得到相对运动轨迹。
struct ImuSample {
    double time = 0.0;
    Vec3 gyro;
    Vec3 acc;
};

// 单颗 GPS 卫星在某个历元的观测值。
// c1：C1C 伪距观测值，单位 m，是 SPP 和当前 RTK float 的主要观测量。
// l1：L1C 载波相位观测值，单位 cycle，用于当前 fixed/smoothed 轨迹平滑。
struct RinexObsValue {
    double c1 = 0.0;
    double l1 = 0.0;
};

// 一个 RINEX 观测历元。
// tow：GPS 周内秒，表示这个历元的观测时间。
// gps：该历元内每颗 GPS 卫星的观测值。
//      key 是 GPS PRN 号，例如 1 表示 G01；
//      value 是该卫星在当前历元的 C1C/L1C 观测。
struct RinexEpoch {
    double tow = 0.0;
    std::map<int, RinexObsValue> gps;
};

// GPS 广播星历参数。
// 字段基本对应 RINEX 3 导航文件中的一组 GPS 星历块。
// SPP 和 RTK 都会使用这些参数计算卫星位置、卫星钟差和相关改正。
// 这里保留原始星历变量名，是为了方便对照 RINEX 文件和 GNSS 教材公式。
struct GpsEphemeris {
    int prn = 0;         // GPS PRN 号
    double toc = 0.0;    // 卫星钟差参考时刻
    double af0 = 0.0;    // 卫星钟差二次多项式常数项
    double af1 = 0.0;    // 卫星钟差一次项
    double af2 = 0.0;    // 卫星钟差二次项
    double iode = 0.0;   // 星历数据龄期
    double crs = 0.0;    // 轨道半径正弦调和改正
    double dn = 0.0;     // 平均角速度改正
    double m0 = 0.0;     // 参考时刻平近点角
    double cuc = 0.0;    // 纬 argument 余弦调和改正
    double e = 0.0;      // 轨道偏心率
    double cus = 0.0;    // 纬 argument 正弦调和改正
    double sqrtA = 0.0;  // 轨道长半轴平方根
    double toe = 0.0;    // 星历参考时刻
    double cic = 0.0;    // 轨道倾角余弦调和改正
    double omega0 = 0.0; // 升交点赤经
    double cis = 0.0;    // 轨道倾角正弦调和改正
    double i0 = 0.0;     // 轨道倾角
    double crc = 0.0;    // 轨道半径余弦调和改正
    double omega = 0.0;  // 近地点角距
    double omegaDot = 0.0; // 升交点赤经变化率
    double idot = 0.0;   // 轨道倾角变化率
    double week = 0.0;   // GPS 周
    double tgd = 0.0;    // 群延迟改正
};

// 单个 GNSS 解算结果。
// 当前主要使用 time 和 ecef；velNed 是给后续速度解算、组合导航扩展预留。
struct GnssSample {
    double time = 0.0;
    Vec3 ecef;
    Vec3 velNed;
};

// RINEX 观测文件解析后的结果。
// path：原始文件路径，方便日志或调试时追踪来源。
// epochCount：读取到的历元数量。
// approxPosition：RINEX 文件头中的 APPROX POSITION XYZ，可作为最小二乘初值。
// epochs：按时间排列的观测历元列表。
struct RinexObservationFile {
    std::string path;
    int epochCount = 0;
    Vec3 approxPosition;
    std::vector<RinexEpoch> epochs;
};

// RINEX 导航文件解析后的结果。
// 当前只读取 GPS 广播星历，后续如果扩展 BDS/Galileo/GLONASS，
// 可以在这里继续增加对应系统的星历数组。
struct RinexNavigationFile {
    std::string path;
    int ephemerisCount = 0;
    std::vector<GpsEphemeris> gpsEphemerides;
};

// 单个轨迹点。
// time：轨迹点时间，通常来自 GNSS 历元时间。
// ecef：该时刻的 ECEF 位置。
// quality：结果类型标记，便于 CSV 和绘图时区分来源：
//          1 = SPP，2 = RTK float，3 = RTK fixed/smoothed，4 = RTK/INS。
struct TrajectoryPoint {
    double time = 0.0;
    Vec3 ecef;
    int quality = 0;
};

// 一条轨迹就是按时间顺序排列的一组轨迹点。
// SPP、RTK、INS 松组合最终都统一输出成 Trajectory，
// 这样 Plot 模块不需要关心轨迹来自哪个算法。
using Trajectory = std::vector<TrajectoryPoint>;
