#pragma once

/*
 * RtkSolver.h
 * -----------
 * RTK 轨迹解算模块接口。
 *
 * RTK = Real Time Kinematic，通常指利用基站和流动站的同步观测来提高定位精度。
 * 本项目中的 RTK 模块分为两个步骤：
 *
 * 1. solveRtkFloat
 *    使用动态站 roverObs、基站 baseObs 和导航星历 nav。
 *    程序会在同一历元中寻找两站共同观测到的 GPS 卫星，
 *    建立单差伪距方程，得到 RTK float 轨迹。
 *
 * 2. solveRtkFixed
 *    在 float 轨迹基础上，使用 L1C 载波相位连续性对轨迹进行平滑，
 *    生成 fixed/smoothed 轨迹。
 *
 * 注意：
 * 当前 fixed 更准确地说是“基于载波连续性的平滑轨迹”，
 * 还不是严格工业 RTK 中的“双差载波相位 + LAMBDA 整周模糊度固定”。
 * README 中也按这个边界进行了说明，面试时这样讲更稳。
 */

#include "Types.h"

// RTK float：动态站 + 基站 + 星历 -> 单差伪距 float 轨迹。
// 返回轨迹的 quality 标记为 2。
Trajectory solveRtkFloat(const RinexObservationFile& roverObs,
                         const RinexObservationFile& baseObs,
                         const RinexNavigationFile& nav);

// RTK fixed/smoothed：在 float 结果基础上使用载波连续性进行平滑。
// 返回轨迹的 quality 标记为 3。
Trajectory solveRtkFixed(const RinexObservationFile& roverObs,
                         const RinexObservationFile& baseObs,
                         const RinexNavigationFile& nav);
