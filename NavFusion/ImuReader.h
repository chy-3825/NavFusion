#pragma once

/*
 * ImuReader.h
 * -----------
 * IMR 原始惯导文件读取接口。
 *
 * IMR 文件保存的是 IMU 原始采样数据。
 * 当前示例数据的文件头是 "$IMURAW"，后面每条记录 32 字节：
 *
 *   double time
 *   int32 gx, gy, gz
 *   int32 ax, ay, az
 *
 * 其中 g 表示陀螺仪三轴增量，a 表示加速度计三轴增量。
 * Reader 会把这些原始整数增量换算成 ImuSample 中的 gyro/acc，
 * 供 InsSolver 做相对位移积分。
 *
 * 这个模块只负责“读 IMR 文件”，不负责 INS 解算。
 */

#include <string>
#include <vector>

#include "Types.h"

// 读取 IMR 文件，返回按时间顺序排列的 IMU 采样序列。
// 如果文件打不开或文件头不符合当前格式，会抛出异常。
std::vector<ImuSample> readImrFile(const std::string& path);
