#pragma once

/*
 * InsSolver.h
 * -----------
 * 简化 INS 机械编排接口。
 *
 * 当前主要做：
 * 1. 对 IMU 加速度做初始偏置估计；
 * 2. 对加速度进行积分，得到相对位移轨迹；
 * 3. 把这个相对轨迹提供给 RTK/INS 松组合模块使用。
 *
 * 后续如果继续完善 INS，还可以加入姿态更新、地球自转、重力模型和坐标系转换。
 */

#include <vector>

#include "Types.h"

Trajectory mechanizeIns(const std::vector<ImuSample>& imu);
