#pragma once

/*
 * RtkInsLoose.h
 * -------------
 * RTK/INS 松组合接口。
 *
 * 松组合的基本思想：
 * - INS 用 IMU 高频数据做短时间预测；
 * - RTK/GNSS 提供低频但不漂移的位置观测；
 * - 二者结合后，轨迹比单独 RTK 更平滑，也比单独 INS 不容易长期漂移。
 *
 * 当前使用简化的预测-更新模型，误差状态滤波留作后续完善。
 */

#include <vector>

#include "Types.h"

Trajectory solveRtkInsLoose(const Trajectory& rtkTrajectory,
                            const std::vector<ImuSample>& imu);
