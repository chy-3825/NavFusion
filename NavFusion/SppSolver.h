#pragma once

/*
 * SppSolver.h
 * -----------
 * SPP 单点定位模块接口。
 *
 * SPP = Single Point Positioning，单点定位。
 * 它只使用一台接收机自己的观测值和广播星历，不依赖基站。
 *
 * 在本项目中，SPP 的输入是：
 * - 动态站 RINEX 观测文件解析结果 roverObs；
 * - GPS 广播星历解析结果 nav。
 *
 * SPP 的主要工作可以理解为：
 * 1. 从每个历元中取出 GPS C1C 伪距；
 * 2. 根据广播星历计算每颗卫星在发射时刻的位置；
 * 3. 建立“伪距 = 卫星到接收机距离 + 接收机钟差 + 误差”的方程；
 * 4. 用最小二乘逐历元求接收机 ECEF 坐标；
 * 5. 把所有历元的位置连成一条 Trajectory。
 *
 * 当前版本主要用于跑通流程，尚未加入完整电离层、对流层、多系统多频等改正。
 */

#include "Types.h"

// 使用动态站观测值和导航星历进行 SPP 解算。
// 返回值是一条 ECEF 坐标轨迹，quality 标记为 1。
Trajectory solveSpp(const RinexObservationFile& roverObs,
                    const RinexNavigationFile& nav);
