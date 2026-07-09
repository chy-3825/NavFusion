#include "ProcessingPipeline.h"

#include <direct.h>
#include <fstream>
#include <string>

#include "ImuReader.h"
#include "Plot.h"
#include "RinexReader.h"
#include "RtkInsLoose.h"
#include "RtkSolver.h"
#include "SppSolver.h"

/*
 * ProcessingPipeline.cpp
 * ----------------------
 * 这是“界面按钮”和“算法模块”之间的中间层。
 *
 * App.cpp 不直接调用 SPP/RTK/INS 细节，只把用户选择的路径交给这里；
 * 这里再负责：
 * 1. 检查输入文件；
 * 2. 调用 Reader 读取数据；
 * 3. 调用 Solver 计算轨迹；
 * 4. 调用 Plot 输出 CSV/SVG/BMP；
 * 5. 返回一段日志文字给界面显示。
 *
 * 这种写法能让界面逻辑和算法逻辑分开，后续如果换成 Qt/MFC 界面，算法流程也不需要大改。
 */

using namespace std;

namespace {

bool fileExists(const string& path) {
    // 只做轻量可打开检查，不在这里解析文件内容。
    if (path.empty()) return false;
    ifstream in(path, ios::binary);
    return static_cast<bool>(in);
}

string requireFile(const string& label, const string& path) {
    if (fileExists(path)) return {};
    return label + " is missing or cannot be opened: " + path + "\r\n";
}

bool hasBlockingInputError(const string& msg) {
    // 当前各步骤都复用输入检查返回的英文提示，用关键字判断是否可以继续。
    return msg.find("missing") != string::npos || msg.find("empty") != string::npos;
}

string pathJoin(const string& dir, const string& name) {
    // 统一拼输出路径，避免各步骤重复处理反斜杠。
    if (dir.empty()) return name;
    const char last = dir.back();
    if (last == '\\' || last == '/') return dir + name;
    return dir + "\\" + name;
}

} // namespace

string checkInputFiles(const PipelineInputs& inputs) {
    // 第1步：只检查文件路径是否存在，为后续按钮解锁提供依据。
    string msg;
    msg += requireFile("Rover observation file", inputs.roverObsPath);
    msg += requireFile("Navigation file", inputs.navPath);
    msg += requireFile("IMR file", inputs.imuPath);
    msg += requireFile("Base observation file", inputs.baseObsPath);
    if (inputs.outDir.empty()) msg += "Output directory is empty.\r\n";

    if (msg.empty()) {
        msg = "Input check passed.\r\n"
              "Rover observation, navigation, IMR and base observation files are ready.\r\n";
    }
    return msg;
}

string runSppStep(const PipelineInputs& inputs) {
    // 第2步：动态站观测 + 广播星历，输出单点定位轨迹。
    const string msg = checkInputFiles(inputs);
    if (hasBlockingInputError(msg)) return msg;

    const auto roverObs = readRinexObservation(inputs.roverObsPath);
    const auto nav = readRinexNavigation(inputs.navPath);
    const auto trajectory = solveSpp(roverObs, nav);
    _mkdir(inputs.outDir.c_str());
    writeTrajectoryCsv(pathJoin(inputs.outDir, "spp_trajectory.csv"), trajectory);
    writeTrajectorySvg(pathJoin(inputs.outDir, "spp_trajectory.svg"), trajectory, "SPP trajectory");
    writeTrajectoryBmp(pathJoin(inputs.outDir, "spp_trajectory.bmp"), trajectory);
    return "SPP solved with GPS C1C pseudorange observations.\r\n"
           "Generated spp_trajectory.csv, spp_trajectory.svg and spp_trajectory.bmp.\r\n";
}

string runRtkFloatStep(const PipelineInputs& inputs) {
    // 第3步：动态站 + 基站 + 星历，当前实现为基站差分改正轨迹。
    const string msg = checkInputFiles(inputs);
    if (hasBlockingInputError(msg)) return msg;

    const auto roverObs = readRinexObservation(inputs.roverObsPath);
    const auto baseObs = readRinexObservation(inputs.baseObsPath);
    const auto nav = readRinexNavigation(inputs.navPath);
    const auto trajectory = solveRtkFloat(roverObs, baseObs, nav);
    _mkdir(inputs.outDir.c_str());
    writeTrajectoryCsv(pathJoin(inputs.outDir, "rtk_float_trajectory.csv"), trajectory);
    writeTrajectorySvg(pathJoin(inputs.outDir, "rtk_float_trajectory.svg"), trajectory, "RTK float trajectory");
    writeTrajectoryBmp(pathJoin(inputs.outDir, "rtk_float_trajectory.bmp"), trajectory);
    return "RTK float trajectory files generated.\r\n"
           "Current build solves GPS common-satellite single-difference pseudorange equations with the base station.\r\n";
}

string runRtkFixedStep(const PipelineInputs& inputs) {
    // 第4步：在差分轨迹基础上结合载波连续性做平滑。
    const string msg = checkInputFiles(inputs);
    if (hasBlockingInputError(msg)) return msg;

    const auto roverObs = readRinexObservation(inputs.roverObsPath);
    const auto baseObs = readRinexObservation(inputs.baseObsPath);
    const auto nav = readRinexNavigation(inputs.navPath);
    const auto trajectory = solveRtkFixed(roverObs, baseObs, nav);
    _mkdir(inputs.outDir.c_str());
    writeTrajectoryCsv(pathJoin(inputs.outDir, "rtk_fixed_trajectory.csv"), trajectory);
    writeTrajectorySvg(pathJoin(inputs.outDir, "rtk_fixed_trajectory.svg"), trajectory, "RTK fixed trajectory");
    writeTrajectoryBmp(pathJoin(inputs.outDir, "rtk_fixed_trajectory.bmp"), trajectory);
    return "RTK fixed trajectory files generated.\r\n"
           "Current build applies carrier-style smoothing on the differential trajectory; integer ambiguity fixing can be added on this state.\r\n";
}

string runRtkInsLooseStep(const PipelineInputs& inputs) {
    // 第5步：用 IMU 预测量和 RTK 轨迹做松组合更新。
    const string msg = checkInputFiles(inputs);
    if (hasBlockingInputError(msg)) return msg;

    const auto roverObs = readRinexObservation(inputs.roverObsPath);
    const auto baseObs = readRinexObservation(inputs.baseObsPath);
    const auto nav = readRinexNavigation(inputs.navPath);
    const auto rtk = solveRtkFixed(roverObs, baseObs, nav);
    const auto imu = readImrFile(inputs.imuPath);
    const auto trajectory = solveRtkInsLoose(rtk, imu);
    _mkdir(inputs.outDir.c_str());
    writeTrajectoryCsv(pathJoin(inputs.outDir, "rtkins_loose_trajectory.csv"), trajectory);
    writeTrajectorySvg(pathJoin(inputs.outDir, "rtkins_loose_trajectory.svg"), trajectory, "RTK/INS loose-coupled trajectory");
    writeTrajectoryBmp(pathJoin(inputs.outDir, "rtkins_loose_trajectory.bmp"), trajectory);
    return "RTK/INS loose-coupled trajectory files generated.\r\n"
           "Current build reads IMR samples, runs a basic INS prediction and updates it with the RTK fixed trajectory.\r\n";
}
