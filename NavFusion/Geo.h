#pragma once

/*
 * Geo.h
 * -----
 * 坐标转换和地球模型工具函数。
 *
 * GNSS 解算中常见几种坐标：
 *
 * - LLH：纬度 Latitude、经度 Longitude、高程 Height。
 *        适合人理解，例如地图上的经纬度。
 *
 * - ECEF：Earth-Centered Earth-Fixed，地心地固直角坐标。
 *         原点在地球质心，坐标轴随地球自转。
 *         GNSS 卫星位置和接收机位置通常用 ECEF 计算。
 *
 * - NED：North-East-Down，局部北东地坐标。
 *        以某个参考点为原点，适合画局部轨迹图和看误差。
 *
 * 本项目中，SPP/RTK 解算主要输出 ECEF，Plot 模块会把轨迹转换到局部坐标用于绘图。
 */

#include "Types.h"

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kDeg = kPi / 180.0; // 角度转弧度：degree * kDeg = radian
constexpr double kRad = 180.0 / kPi; // 弧度转角度：radian * kRad = degree

// LLH -> ECEF。
// latRad/lonRad 使用弧度，heightM 使用米。
Vec3 llhToEcef(double latRad, double lonRad, double heightM);

// ECEF -> LLH。
// 输入 ECEF 米级坐标，输出纬度弧度、经度弧度和大地高米。
void ecefToLlh(const Vec3& ecef, double& latRad, double& lonRad, double& heightM);

// ECEF 差向量 -> NED 差向量。
// ecefDiff 是“某点 ECEF - 参考点 ECEF”；
// refLatRad/refLonRad 是参考点经纬度。
Vec3 ecefToNed(const Vec3& ecefDiff, double refLatRad, double refLonRad);

// 正常重力近似值。
// INS 积分时可用它做重力补偿；当前项目保留为基础地球模型工具。
double normalGravity(double latRad, double heightM);
