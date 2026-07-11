#include <windows.h>

#include <fstream>
#include <string>

#include "App.h"
#include "ProcessingPipeline.h"

/*
 * main.cpp
 * --------
 * 程序入口文件。
 *
 * 本项目是 Win32 桌面程序，所以入口函数不是普通控制台 main，
 * 而是 Windows 子系统使用的 wWinMain。
 *
 * 这里同时保留两个自动测试参数：
 * - --run-spp-default：不打开界面，只跑默认 SPP；
 * - --run-all-default：不打开界面，按默认数据跑完整 2~5 步。
 *
 * 这样做的好处是：平时用户可以用图形界面操作；
 * 开发调试时可以用命令快速验证算法和输出文件。
 */

namespace {

bool hasArg(const wchar_t* cmd, const wchar_t* arg) {
    // 用于自动化测试入口，例如 --run-all-default 可不打开窗口直接生成默认数据结果。
    return cmd && wcsstr(cmd, arg) != nullptr;
}

std::wstring pathJoin(const std::wstring& dir, const std::wstring& name) {
    if (dir.empty()) return name;
    const wchar_t last = dir.back();
    if (last == L'\\' || last == L'/') return dir + name;
    return dir + L"\\" + name;
}

std::wstring parentDir(std::wstring path) {
    while (!path.empty() && (path.back() == L'\\' || path.back() == L'/')) {
        path.pop_back();
    }
    const size_t pos = path.find_last_of(L"\\/");
    if (pos == std::wstring::npos) return {};
    return path.substr(0, pos);
}

bool pathExists(const std::wstring& path) {
    return GetFileAttributesW(path.c_str()) != INVALID_FILE_ATTRIBUTES;
}

std::wstring modulePath() {
    wchar_t buf[MAX_PATH * 2]{};
    GetModuleFileNameW(nullptr, buf, static_cast<DWORD>(_countof(buf)));
    return buf;
}

std::wstring findProjectRoot() {
    // 命令行默认运行时也动态寻找数据目录，外层文件夹改名后不用再改源码。
    std::wstring dir = parentDir(modulePath());
    for (int i = 0; i < 8 && !dir.empty(); ++i) {
        if (pathExists(pathJoin(dir, L"sj\\Observation.26o"))) {
            return dir;
        }
        const std::wstring next = parentDir(dir);
        if (next == dir) break;
        dir = next;
    }
    return L".";
}

std::string narrow(const std::wstring& s) {
    if (s.empty()) return {};
    const int size = WideCharToMultiByte(CP_ACP, 0, s.c_str(), -1, nullptr, 0, nullptr, nullptr);
    std::string out(size > 0 ? size - 1 : 0, '\0');
    if (size > 0) WideCharToMultiByte(CP_ACP, 0, s.c_str(), -1, &out[0], size, nullptr, nullptr);
    return out;
}

} // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int showCmd) {
    // 正常运行时进入 Win32 界面；带测试参数时直接跑默认数据，方便回归验证。
    const wchar_t* cmd = GetCommandLineW();
    if (hasArg(cmd, L"--run-spp-default") || hasArg(cmd, L"--run-all-default") || hasArg(cmd, L"--run-ins-default")) {
        const std::wstring rootW = findProjectRoot();
        const std::wstring outW = pathJoin(rootW, hasArg(cmd, L"--run-ins-default") ? L"output_check" : L"output");
        CreateDirectoryW(outW.c_str(), nullptr);
        const std::string root = narrow(rootW);
        const std::string resultPath = narrow(pathJoin(
            outW,
            hasArg(cmd, L"--run-ins-default") ? L"run_ins_default_result.txt" : L"run_spp_default_result.txt"));

        PipelineInputs inputs;
        inputs.roverObsPath = root + "\\sj\\Observation.26o";
        inputs.navPath = root + "\\sj\\Navigation.26p";
        inputs.imuPath = root + "\\sj\\IMU_Raw_Data.dat.imr";
        inputs.baseObsPath = root + "\\sj\\Base_Station.rnx";
        inputs.outDir = narrow(outW);

        try {
            std::ofstream result(resultPath);
            if (hasArg(cmd, L"--run-ins-default")) {
                result << runPureInsAllanStep(inputs);
            } else {
                result << runSppStep(inputs);
            }
            if (!hasArg(cmd, L"--run-ins-default") && hasArg(cmd, L"--run-all-default")) {
                result << "\n" << runRtkFloatStep(inputs);
                result << "\n" << runRtkFixedStep(inputs);
                result << "\n" << runRtkInsLooseStep(inputs);
                result << "\n" << runPureInsAllanStep(inputs);
            }
        } catch (const std::exception& e) {
            std::ofstream result(resultPath);
            result << "ERROR: " << e.what() << "\n";
        }
        return 0;
    }

    return runApp(instance, showCmd);
}
