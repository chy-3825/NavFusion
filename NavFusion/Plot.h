#pragma once

/*
 * Plot.h
 * ------
 * 轨迹输出模块接口。
 *
 * Solver 模块只负责生成 Trajectory，不直接关心文件格式。
 * Plot 模块负责把同一条 Trajectory 输出成三种形式：
 *
 * - CSV：数值结果，适合 Excel、MATLAB、Python 后续分析；
 * - SVG：矢量轨迹图，适合放 README、报告或论文插图；
 * - BMP：位图轨迹图，方便 Win32 界面直接加载到右侧预览区。
 *
 * 这样 SPP、RTK、RTK/INS 都可以复用同一套输出函数。
 */

#include <string>

#include "Types.h"

// 写 CSV 文件。
// 内容包括时间、ECEF 坐标和 quality 标记。
void writeTrajectoryCsv(const std::string& path, const Trajectory& trajectory);

// 写 SVG 轨迹图。
// title 会显示在图中，用于区分 SPP/RTK/RTKINS 等结果。
void writeTrajectorySvg(const std::string& path, const Trajectory& trajectory, const std::string& title);

// 写 BMP 轨迹图。
// Win32 界面右侧预览区主要加载这个文件。
void writeTrajectoryBmp(const std::string& path, const Trajectory& trajectory);

// Write Allan variance/deviation numeric output.
void writeAllanCsv(const std::string& path, const AllanSeries& allan);

// Write a compact Allan deviation plot for gyro and accelerometer magnitudes.
void writeAllanSvg(const std::string& path, const AllanSeries& allan, const std::string& title);
void writeAllanBmp(const std::string& path, const AllanSeries& allan);
