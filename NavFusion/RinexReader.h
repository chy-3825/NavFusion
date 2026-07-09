#pragma once

/*
 * RinexReader.h
 * -------------
 * RINEX 文件读取模块。
 *
 * 本项目使用两类 RINEX 文件：
 * 1. 观测文件 O：读取 GPS C1C 伪距、L1C 载波相位和历元时间；
 * 2. 导航文件 P/NAV：读取 GPS 广播星历参数。
 *
 * Reader 的职责只做“读文件并整理成结构体”，不在这里做定位解算。
 * 这样 SPP 和 RTK 都能复用同一份解析结果。
 */

#include <string>
#include <vector>

#include "Types.h"

// 读取 RINEX 观测文件，返回包含近似坐标和历元观测值的结构体。
RinexObservationFile readRinexObservation(const std::string& path);

// 读取 RINEX 导航文件，返回包含 GPS 广播星历的结构体。
RinexNavigationFile readRinexNavigation(const std::string& path);

// 只读取 GPS 观测历元。这个函数供 readRinexObservation 调用，也方便后续单独测试。
std::vector<RinexEpoch> readGpsObservations(const std::string& path, Vec3& approxPosition);

// 只读取 GPS 广播星历。这个函数供 readRinexNavigation 调用，也方便后续扩展多系统。
std::vector<GpsEphemeris> readGpsNavigationEphemerides(const std::string& path);
