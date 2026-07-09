#pragma once

/*
 * RinexReader.h
 * -------------
 * RINEX 文件读取模块。
 *
 * RINEX 是 GNSS 数据处理中常见的标准文本格式。
 * 本项目使用两类 RINEX 文件：
 *
 * 1. 观测文件 O
 *    例如 Observation.26o 或 Base_Station.rnx。
 *    程序主要读取：
 *    - 历元时间；
 *    - GPS C1C 伪距；
 *    - GPS L1C 载波相位；
 *    - 文件头里的 APPROX POSITION XYZ 近似坐标。
 *
 * 2. 导航星历文件 P/NAV
 *    例如 Navigation.26p。
 *    程序读取 GPS 广播星历参数，用于计算卫星位置和卫星钟差。
 *
 * Reader 模块只负责“把文件读成结构体”，不做定位解算。
 * SPP、RTK 等算法模块会复用这里读出来的数据。
 */

#include <string>
#include <vector>

#include "Types.h"

// 读取完整 RINEX 观测文件。
// 返回值中包含文件路径、近似坐标、历元数量和每个历元的 GPS 观测值。
RinexObservationFile readRinexObservation(const std::string& path);

// 读取完整 RINEX 导航文件。
// 返回值中包含文件路径、星历数量和 GPS 广播星历列表。
RinexNavigationFile readRinexNavigation(const std::string& path);

// 读取 GPS 观测历元。
// approxPosition 会被写入文件头中的 APPROX POSITION XYZ。
// 这个函数供 readRinexObservation 调用，也方便后续单独测试观测文件解析。
std::vector<RinexEpoch> readGpsObservations(const std::string& path, Vec3& approxPosition);

// 读取 GPS 广播星历。
// 这个函数供 readRinexNavigation 调用，后续扩展多系统时可增加 BDS/Galileo 等类似函数。
std::vector<GpsEphemeris> readGpsNavigationEphemerides(const std::string& path);
