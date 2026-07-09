#pragma once

/*
 * ProcessingPipeline.h
 * --------------------
 * 处理流程调度模块的接口。
 *
 * App.cpp 只负责界面交互，不直接做算法；
 * ProcessingPipeline.cpp 负责把“按钮步骤”转换成具体处理流程：
 * 1. 检查输入文件；
 * 2. 运行 SPP；
 * 3. 运行 RTK float；
 * 4. 运行 RTK fixed；
 * 5. 运行 RTK/INS 松组合。
 */

#include <string>

// 界面中选择的所有路径都收集到这个结构体里，再交给处理流程使用。
struct PipelineInputs {
    std::string roverObsPath; // 动态站观测文件，例如 Observation.26o
    std::string navPath;      // 导航电文文件，例如 Navigation.26p
    std::string imuPath;      // IMR 原始惯导文件
    std::string baseObsPath;  // 基站观测文件
    std::string outDir;       // 输出目录，保存 CSV/SVG/BMP
};

// 第 1 步：只检查文件是否存在、输出目录是否填写。
std::string checkInputFiles(const PipelineInputs& inputs);

// 第 2 步：SPP 单点定位，并输出 spp_trajectory.*
std::string runSppStep(const PipelineInputs& inputs);

// 第 3 步：简化 RTK float，并输出 rtk_float_trajectory.*
std::string runRtkFloatStep(const PipelineInputs& inputs);

// 第 4 步：载波连续性辅助的 RTK fixed 结果，并输出 rtk_fixed_trajectory.*
std::string runRtkFixedStep(const PipelineInputs& inputs);

// 第 5 步：IMU 预测 + RTK 更新的松组合结果，并输出 rtkins_loose_trajectory.*
std::string runRtkInsLooseStep(const PipelineInputs& inputs);
