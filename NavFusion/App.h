#pragma once

/*
 * App.h
 * -----
 * Win32 图形界面入口。
 *
 * main.cpp 调用 runApp 后，程序会创建主窗口、文件选择框、步骤按钮、日志框和右侧预览区。
 * 具体窗口控件和消息处理逻辑在 App.cpp 中实现。
 */

#include <windows.h>

int runApp(HINSTANCE instance, int showCmd);
