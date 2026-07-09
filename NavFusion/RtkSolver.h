#pragma once

/*
 * RtkSolver.h
 * -----------
 * 简化 RTK 解算模块接口。
 *
 * solveRtkFloat：
 * 使用动态站和基站的共同 GPS 卫星，建立单差伪距方程，求动态站位置。
 *
 * solveRtkFixed：
 * 在 float 轨迹基础上，引入 L1 载波相位连续性判断，对轨迹做更稳定的平滑。
 *
 * 目前的 fixed 结果以载波连续性平滑为主，整周模糊度固定留作后续完善。
 */

#include "Types.h"

Trajectory solveRtkFloat(const RinexObservationFile& roverObs,
                         const RinexObservationFile& baseObs,
                         const RinexNavigationFile& nav);

Trajectory solveRtkFixed(const RinexObservationFile& roverObs,
                         const RinexObservationFile& baseObs,
                         const RinexNavigationFile& nav);
