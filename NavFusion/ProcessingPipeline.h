#pragma once

/*
 * ProcessingPipeline.h
 * --------------------
 * 处理流程调度模块接口。
 *
 * App.cpp 负责“界面交互”：用户点按钮、选文件、看日志。
 * Solver/Reader/Plot 负责“具体功能”：读数据、算轨迹、写结果。
 *
 * ProcessingPipeline 位于二者中间，作用是把界面上的按钮步骤翻译成完整处理流程：
 *
 *   1 Check data      -> checkInputFiles
 *   2 SPP track       -> runSppStep
 *   3 RTK float       -> runRtkFloatStep
 *   4 RTK fixed       -> runRtkFixedStep
 *   5 RTK/INS LC      -> runRtkInsLooseStep
 *
 * 这样界面代码不用直接了解 SPP/RTK/INS 的内部细节，算法模块也不用关心窗口控件。
 */

#include <string>

// 界面中选择的所有路径都会收集到这个结构体里，再交给处理流程使用。
// 你可以把它理解为“本次解算任务的输入配置”。
struct PipelineInputs {
    // 动态站观测文件，例如 Observation.26o。
    // SPP、RTK float、RTK fixed 和松组合都会用到它。
    std::string roverObsPath;

    // GPS 广播星历文件，例如 Navigation.26p。
    // 用于根据星历参数计算卫星位置和卫星钟差。
    std::string navPath;

    // IMR 原始惯导文件。
    // 只在 RTK/INS 松组合步骤中读取，用于 INS 预测。
    std::string imuPath;

    // 基站观测文件。
    // RTK float、RTK fixed 和 RTK/INS 松组合会用它和动态站做共同卫星匹配。
    std::string baseObsPath;

    // 输出目录。
    // 每一步生成的 CSV/SVG/BMP 都写到这个目录中。
    std::string outDir;
};

// 第 1 步：检查输入文件是否存在、输出目录是否填写。
// 返回值是一段给界面日志框显示的文字；如果文件缺失，会包含 missing/empty 等提示。
std::string checkInputFiles(const PipelineInputs& inputs);

// 第 2 步：SPP 单点定位。
// 输入动态站观测文件和导航星历，输出 spp_trajectory.csv/svg/bmp。
std::string runSppStep(const PipelineInputs& inputs);

// 第 3 步：RTK float。
// 输入动态站、基站和导航星历，输出 rtk_float_trajectory.csv/svg/bmp。
std::string runRtkFloatStep(const PipelineInputs& inputs);

// 第 4 步：RTK fixed/smoothed。
// 当前版本是在 RTK float 轨迹基础上利用 L1 载波连续性进行平滑，
// 输出 rtk_fixed_trajectory.csv/svg/bmp。
std::string runRtkFixedStep(const PipelineInputs& inputs);

// 第 5 步：RTK/INS 松组合。
// 先得到 RTK fixed/smoothed 轨迹，再读取 IMR 数据做 INS 预测，
// 最后输出 rtkins_loose_trajectory.csv/svg/bmp。
std::string runRtkInsLooseStep(const PipelineInputs& inputs);

// Step 6: pure INS mechanization and Allan variance analysis from IMR data.
std::string runPureInsAllanStep(const PipelineInputs& inputs);
