#pragma once

/*
 * RtkInsLoose.h
 * -------------
 * RTK/INS 松组合接口。
 *
 * 松组合的特点是：
 * - RTK/GNSS 先独立算出位置轨迹；
 * - INS 也先根据 IMU 独立做短时间预测；
 * - 然后在位置层面把两者融合。
 *
 * 和“紧组合”相比，松组合更容易理解，也更适合作为组合导航入门项目。
 *
 * 当前版本的融合思路：
 * 1. 以 RTK fixed/smoothed 轨迹作为低频位置观测；
 * 2. 以 IMU 积分得到的相对位移作为两次 RTK 历元之间的预测；
 * 3. 用固定增益进行预测-更新；
 * 4. 输出更平滑的 RTK/INS loose-coupled 轨迹。
 *
 * 后续如果要升级，可以把固定增益替换为误差状态卡尔曼滤波。
 */

#include <vector>

#include "Types.h"

// rtkTrajectory：RTK fixed/smoothed 轨迹，提供低频位置约束。
// imu：IMR 文件解析后的 IMU 高频采样，提供短时预测。
// 返回 quality 标记为 4 的松组合轨迹。
Trajectory solveRtkInsLoose(const Trajectory& rtkTrajectory,
                            const std::vector<ImuSample>& imu);
