#pragma once

/*
 * InsSolver.h
 * -----------
 * 简化 INS 机械编排接口。
 *
 * INS = Inertial Navigation System，惯性导航系统。
 * 理论上完整 INS 会包含：
 * - 姿态更新；
 * - 速度更新；
 * - 位置更新；
 * - 地球自转、运输率、重力补偿；
 * - IMU 零偏、比例因子等误差建模。
 *
 * 当前项目为了服务 RTK/INS 松组合演示，先实现一个简化版本：
 * 1. 读取 IMU 加速度；
 * 2. 估计初始加速度偏置；
 * 3. 对加速度进行两次积分；
 * 4. 得到一个相对位移轨迹；
 * 5. 把这个相对轨迹交给 RtkInsLoose 做短时间预测。
 *
 * 所以这里的 mechanizeIns 更适合作为“INS 预测量”的原型，
 * 不是完整高精度惯导机械编排。
 */

#include <vector>

#include "Types.h"

// 根据 IMU 采样序列积分得到相对位移轨迹。
// 返回的 Trajectory 不是绝对 GNSS 坐标，而是供松组合使用的相对运动信息。
Trajectory mechanizeIns(const std::vector<ImuSample>& imu);

// Compute Allan variance/deviation for gyro and accelerometer triads.
AllanSeries computeAllanVariance(const std::vector<ImuSample>& imu);
