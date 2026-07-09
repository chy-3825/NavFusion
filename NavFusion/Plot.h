#pragma once

/*
 * Plot.h
 * ------
 * 轨迹输出模块接口。
 *
 * CSV：保存数值结果，便于 Excel、MATLAB、Python 后续分析；
 * SVG：保存高清矢量轨迹图，适合放报告；
 * BMP：保存位图轨迹图，方便 Win32 界面直接加载预览。
 */

#include <string>

#include "Types.h"

void writeTrajectoryCsv(const std::string& path, const Trajectory& trajectory);
void writeTrajectorySvg(const std::string& path, const Trajectory& trajectory, const std::string& title);
void writeTrajectoryBmp(const std::string& path, const Trajectory& trajectory);
