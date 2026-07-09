#pragma once

/*
 * SppSolver.h
 * -----------
 * SPP 单点定位模块接口。
 *
 * 输入：
 * - 动态站 RINEX 观测结果；
 * - GPS 广播星历。
 *
 * 输出：
 * - 一条 ECEF 坐标轨迹。
 *
 * 当前使用 GPS C1C 伪距做最小二乘定位。
 */

#include "Types.h"

Trajectory solveSpp(const RinexObservationFile& roverObs,
                    const RinexNavigationFile& nav);
