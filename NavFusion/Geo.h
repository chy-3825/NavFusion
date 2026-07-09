#pragma once

/*
 * Geo.h
 * -----
 * 坐标转换和地球模型工具函数。
 *
 * 本项目主要使用两类坐标：
 * - ECEF：地心地固直角坐标，适合 GNSS 解算；
 * - NED：以某个参考点为原点的局部北东地坐标，适合画轨迹图。
 */

#include "Types.h"

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kDeg = kPi / 180.0; // 角度转弧度
constexpr double kRad = 180.0 / kPi; // 弧度转角度

Vec3 llhToEcef(double latRad, double lonRad, double heightM);
void ecefToLlh(const Vec3& ecef, double& latRad, double& lonRad, double& heightM);
Vec3 ecefToNed(const Vec3& ecefDiff, double refLatRad, double refLonRad);
double normalGravity(double latRad, double heightM);
