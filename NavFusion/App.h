#pragma once

/*
 * App.h
 * -----
 * Win32 图形界面入口。
 *
 * main.cpp 调用 runApp 后，程序会创建主窗口，里面包括：
 * - 文件路径选择框；
 * - 输出目录选择框；
 * - 1~5 步处理按钮；
 * - 日志显示框；
 * - 右侧轨迹图预览区；
 * - 保存当前图片/CSV 的按钮。
 *
 * App.cpp 只负责界面和用户操作，不直接写 SPP、RTK、INS 算法。
 * 用户点击按钮后，App.cpp 会调用 ProcessingPipeline 中对应的步骤函数。
 */

#include <windows.h>

// 启动 Win32 主窗口。
// instance/showCmd 都是 Windows 程序入口传进来的窗口参数。
int runApp(HINSTANCE instance, int showCmd);
