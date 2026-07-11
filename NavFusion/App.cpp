#include "App.h"

#include <commdlg.h>
#include <shlobj.h>

#include <stdexcept>
#include <string>

#include "ProcessingPipeline.h"

#pragma comment(lib, "Comdlg32.lib")
#pragma comment(lib, "Shell32.lib")

/*
 * App.cpp
 * -------
 * Win32 图形界面实现文件。
 *
 * 界面主要由四部分组成：
 * 1. 左上：输入文件路径选择框；
 * 2. 左中：1~5 步处理按钮；
 * 3. 左下：运行日志；
 * 4. 右侧：当前步骤轨迹图预览，以及保存图片/CSV 的按钮。
 *
 * 本文件只负责“用户交互”和“显示结果”，不在这里写具体算法。
 * 真正的解算流程在 ProcessingPipeline.cpp 中完成。
 */

using namespace std;

namespace {

constexpr int kWidth = 1420;
constexpr int kHeight = 640;

enum ControlId {
    // 所有控件ID集中管理，便于WM_COMMAND里区分按钮和输入框。
    IdRoverObsEdit = 100,
    IdNavEdit,
    IdImuEdit,
    IdBaseObsEdit,
    IdOutDirEdit,
    IdBrowseRoverObs = 200,
    IdBrowseNav,
    IdBrowseImu,
    IdBrowseBaseObs,
    IdBrowseOutDir,
    IdStep1 = 300,
    IdStep2,
    IdStep3,
    IdStep4,
    IdStep5,
    IdStep6,
    IdSavePreview,
    IdSaveCsv,
    IdSaveAllanPreview,
    IdSaveAllanCsv,
    IdLog = 400,
    IdImageTitle,
    IdImage
};

HWND gMain = nullptr;
HBITMAP gPreviewBitmap = nullptr;
wstring gCurrentPreviewPath;
wstring gCurrentCsvPath;
wstring gCurrentAllanPreviewPath;
wstring gCurrentAllanCsvPath;

wstring widen(const string& s) {
    // UI使用Unicode宽字符，处理管线内部使用std::string路径。
    if (s.empty()) return {};
    const int size = MultiByteToWideChar(CP_ACP, 0, s.c_str(), -1, nullptr, 0);
    wstring out(size > 0 ? size - 1 : 0, L'\0');
    if (size > 0) MultiByteToWideChar(CP_ACP, 0, s.c_str(), -1, &out[0], size);
    return out;
}

string narrow(const wstring& s) {
    if (s.empty()) return {};
    const int size = WideCharToMultiByte(CP_ACP, 0, s.c_str(), -1, nullptr, 0, nullptr, nullptr);
    string out(size > 0 ? size - 1 : 0, '\0');
    if (size > 0) WideCharToMultiByte(CP_ACP, 0, s.c_str(), -1, &out[0], size, nullptr, nullptr);
    return out;
}

HWND item(int id) {
    return GetDlgItem(gMain, id);
}

wstring getText(int id) {
    wchar_t buf[MAX_PATH * 2]{};
    GetWindowTextW(item(id), buf, static_cast<int>(size(buf)));
    return buf;
}

void setText(int id, const wstring& text) {
    SetWindowTextW(item(id), text.c_str());
}

HMENU controlMenu(int id) {
    return reinterpret_cast<HMENU>(static_cast<INT_PTR>(id));
}

void appendLog(const string& text) {
    // 将每一步的处理状态追加到左下角日志框。
    const wstring w = widen(text);
    HWND log = item(IdLog);
    const int len = GetWindowTextLengthW(log);
    SendMessageW(log, EM_SETSEL, len, len);
    SendMessageW(log, EM_REPLACESEL, FALSE, reinterpret_cast<LPARAM>(w.c_str()));
    SendMessageW(log, EM_REPLACESEL, FALSE, reinterpret_cast<LPARAM>(L"\r\n"));
}

wstring pathJoin(const wstring& dir, const wstring& name) {
    if (dir.empty()) return name;
    const wchar_t last = dir.back();
    if (last == L'\\' || last == L'/') return dir + name;
    return dir + L"\\" + name;
}

wstring parentDir(wstring path) {
    while (!path.empty() && (path.back() == L'\\' || path.back() == L'/')) {
        path.pop_back();
    }
    const size_t pos = path.find_last_of(L"\\/");
    if (pos == wstring::npos) return {};
    return path.substr(0, pos);
}

bool pathExists(const wstring& path) {
    return GetFileAttributesW(path.c_str()) != INVALID_FILE_ATTRIBUTES;
}

wstring modulePath() {
    wchar_t buf[MAX_PATH * 2]{};
    GetModuleFileNameW(nullptr, buf, static_cast<DWORD>(size(buf)));
    return buf;
}

wstring findProjectRoot() {
    // 从 exe 所在目录向上查找 sj 数据文件夹，避免外层文件夹改名后默认路径失效。
    wstring dir = parentDir(modulePath());
    for (int i = 0; i < 8 && !dir.empty(); ++i) {
        if (pathExists(pathJoin(dir, L"sj\\Observation.26o"))) {
            return dir;
        }
        const wstring next = parentDir(dir);
        if (next == dir) break;
        dir = next;
    }
    return L".";
}

bool stepSucceeded(const string& msg) {
    // 流程函数返回的是可读日志，这里用简单关键字判断该步骤是否成功。
    // 成功后才解锁下一步按钮，避免用户跳过必要步骤。
    return msg.find("missing") == string::npos &&
           msg.find("empty") == string::npos &&
           msg.find("ERROR") == string::npos;
}

void cleanupOutputDir(const wstring& outDir) {
    // 程序关闭时清理 output 中由本程序生成的临时结果。
    // 注意：这里只删除固定文件名的中间输出，不会删除用户另存的图片/CSV。
    if (outDir.empty()) return;

    const wchar_t* files[] = {
        L"spp_trajectory.csv",
        L"spp_trajectory.svg",
        L"spp_trajectory.bmp",
        L"rtk_float_trajectory.csv",
        L"rtk_float_trajectory.svg",
        L"rtk_float_trajectory.bmp",
        L"rtk_fixed_trajectory.csv",
        L"rtk_fixed_trajectory.svg",
        L"rtk_fixed_trajectory.bmp",
        L"rtkins_loose_trajectory.csv",
        L"rtkins_loose_trajectory.svg",
        L"rtkins_loose_trajectory.bmp",
        L"pure_ins_trajectory.csv",
        L"pure_ins_trajectory.svg",
        L"pure_ins_trajectory.bmp",
        L"allan_variance.csv",
        L"allan_variance.svg",
        L"allan_variance.bmp",
        L"run_spp_default_result.txt",
        L"run_spp_default_marker.txt"
    };

    for (const wchar_t* name : files) {
        DeleteFileW(pathJoin(outDir, name).c_str());
    }
}

void showPreview(const wstring& title, const wstring& imagePath, const wstring& csvPath) {
    // 右侧预览框加载每一步生成的BMP图；旧位图要释放，避免GDI资源泄漏。
    setText(IdImageTitle, title);
    HBITMAP next = reinterpret_cast<HBITMAP>(
        LoadImageW(nullptr, imagePath.c_str(), IMAGE_BITMAP, 520, 360,
                   LR_LOADFROMFILE | LR_CREATEDIBSECTION));
    if (!next) {
        appendLog("Preview image was not generated or could not be loaded.");
        return;
    }
    HBITMAP old = reinterpret_cast<HBITMAP>(
        SendMessageW(item(IdImage), STM_SETIMAGE, IMAGE_BITMAP, reinterpret_cast<LPARAM>(next)));
    if (old) DeleteObject(old);
    if (gPreviewBitmap && gPreviewBitmap != old) DeleteObject(gPreviewBitmap);
    gPreviewBitmap = next;
    gCurrentPreviewPath = imagePath;
    gCurrentCsvPath = csvPath;
    EnableWindow(item(IdSavePreview), TRUE);
    EnableWindow(item(IdSaveCsv), TRUE);
}

void setAllanResult(const wstring& imagePath, const wstring& csvPath) {
    gCurrentAllanPreviewPath = imagePath;
    gCurrentAllanCsvPath = csvPath;
    EnableWindow(item(IdSaveAllanPreview), TRUE);
    EnableWindow(item(IdSaveAllanCsv), TRUE);
}

bool saveExistingFile(HWND parent,
                      const wstring& sourcePath,
                      const wchar_t* title,
                      const wchar_t* filter,
                      const wchar_t* defaultExt,
                      const wchar_t* defaultName) {
    // 通用“另存为”函数。
    // 图片和 CSV 保存逻辑基本一样，只是过滤器和默认扩展名不同，所以抽成一个函数复用。
    if (sourcePath.empty()) {
        MessageBoxW(parent, L"No current result is available yet.", L"NavFusion", MB_OK | MB_ICONINFORMATION);
        return false;
    }

    wchar_t file[MAX_PATH * 2]{};
    wcscpy_s(file, defaultName);
    OPENFILENAMEW ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = parent;
    ofn.lpstrTitle = title;
    ofn.lpstrFilter = filter;
    ofn.lpstrFile = file;
    ofn.nMaxFile = static_cast<DWORD>(size(file));
    ofn.lpstrDefExt = defaultExt;
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST;

    if (!GetSaveFileNameW(&ofn)) return false;
    if (!CopyFileW(sourcePath.c_str(), file, FALSE)) {
        MessageBoxW(parent, L"Failed to save the selected result file.", L"NavFusion", MB_OK | MB_ICONERROR);
        appendLog("ERROR: failed to save selected result file.");
        return false;
    }

    appendLog("Saved current result to: " + narrow(file));
    return true;
}

void saveCurrentPreview(HWND parent) {
    // 保存当前右侧正在展示的 BMP 轨迹图。
    saveExistingFile(parent,
                     gCurrentPreviewPath,
                     L"Save current preview image",
                     L"BMP image (*.bmp)\0*.bmp\0All Files\0*.*\0",
                     L"bmp",
                     L"trajectory.bmp");
}

void saveCurrentCsv(HWND parent) {
    // 保存当前右侧轨迹图对应的 CSV 数据。
    saveExistingFile(parent,
                     gCurrentCsvPath,
                     L"Save current trajectory CSV",
                     L"CSV file (*.csv)\0*.csv\0All Files\0*.*\0",
                     L"csv",
                     L"trajectory.csv");
}

void saveCurrentAllanPreview(HWND parent) {
    saveExistingFile(parent,
                     gCurrentAllanPreviewPath,
                     L"Save Allan variance image",
                     L"BMP image (*.bmp)\0*.bmp\0All Files\0*.*\0",
                     L"bmp",
                     L"allan_variance.bmp");
}

void saveCurrentAllanCsv(HWND parent) {
    saveExistingFile(parent,
                     gCurrentAllanCsvPath,
                     L"Save Allan variance CSV",
                     L"CSV file (*.csv)\0*.csv\0All Files\0*.*\0",
                     L"csv",
                     L"allan_variance.csv");
}

PipelineInputs inputsFromUi() {
    // 从界面输入框收集路径，交给ProcessingPipeline执行具体计算。
    PipelineInputs in;
    in.roverObsPath = narrow(getText(IdRoverObsEdit));
    in.navPath = narrow(getText(IdNavEdit));
    in.imuPath = narrow(getText(IdImuEdit));
    in.baseObsPath = narrow(getText(IdBaseObsEdit));
    in.outDir = narrow(getText(IdOutDirEdit));
    return in;
}

void setStepEnabled(int maxStep);

bool chooseFile(HWND parent, const wchar_t* title, const wchar_t* filter, int editId) {
    // 通用文件选择函数：选择后回到第1步，要求用户重新检查输入。
    wchar_t file[MAX_PATH * 2]{};
    const wstring current = getText(editId);
    if (!current.empty()) wcscpy_s(file, current.c_str());
    OPENFILENAMEW ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = parent;
    ofn.lpstrTitle = title;
    ofn.lpstrFilter = filter;
    ofn.lpstrFile = file;
    ofn.nMaxFile = static_cast<DWORD>(size(file));
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
    if (!GetOpenFileNameW(&ofn)) return false;
    setText(editId, file);
    setStepEnabled(1);
    return true;
}

bool chooseFolder(HWND parent, int editId) {
    // 选择输出目录。输出目录既用于临时显示结果，也用于保存每一步自动生成的文件。
    BROWSEINFOW bi{};
    bi.hwndOwner = parent;
    bi.lpszTitle = L"Select output directory";
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    PIDLIST_ABSOLUTE pidl = SHBrowseForFolderW(&bi);
    if (!pidl) return false;
    wchar_t path[MAX_PATH]{};
    const BOOL ok = SHGetPathFromIDListW(pidl, path);
    CoTaskMemFree(pidl);
    if (!ok) return false;
    setText(editId, path);
    setStepEnabled(1);
    return true;
}

void createLabel(HWND parent, int x, int y, const wchar_t* text) {
    CreateWindowW(L"STATIC", text, WS_CHILD | WS_VISIBLE,
                  x, y + 5, 110, 22, parent, nullptr, nullptr, nullptr);
}

void createPathRow(HWND parent, int y, const wchar_t* label, int editId, int browseId) {
    createLabel(parent, 20, y, label);
    CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"", WS_CHILD | WS_VISIBLE | ES_AUTOHSCROLL,
                    135, y, 610, 26, parent, controlMenu(editId), nullptr, nullptr);
    CreateWindowW(L"BUTTON", L"Browse", WS_CHILD | WS_VISIBLE,
                  755, y, 90, 26, parent, controlMenu(browseId), nullptr, nullptr);
}

void setStepEnabled(int maxStep) {
    // GNSS processing is sequential, while pure INS only needs the IMR input check.
    EnableWindow(item(IdStep1), TRUE);
    EnableWindow(item(IdStep2), maxStep >= 2);
    EnableWindow(item(IdStep3), maxStep >= 3);
    EnableWindow(item(IdStep4), maxStep >= 4);
    EnableWindow(item(IdStep5), maxStep >= 5);
    EnableWindow(item(IdStep6), maxStep >= 2);
}

void runStep(int id) {
    // 按钮回调：每一步只执行自己的计算，并把对应BMP显示到右侧。
    try {
        const PipelineInputs in = inputsFromUi();
        if (id == IdStep1) {
            appendLog("Step 1: checking selected files...");
            const string result = checkInputFiles(in);
            appendLog(result);
            if (stepSucceeded(result)) {
                setStepEnabled(2);
                setText(IdImageTitle, L"Step 1 checked. Run SPP or pure INS/Allan next.");
            }
        } else if (id == IdStep2) {
            appendLog("Step 2: SPP trajectory...");
            const string result = runSppStep(in);
            appendLog(result);
            if (stepSucceeded(result)) {
                setStepEnabled(3);
                showPreview(L"Step 2 - SPP trajectory",
                            pathJoin(getText(IdOutDirEdit), L"spp_trajectory.bmp"),
                            pathJoin(getText(IdOutDirEdit), L"spp_trajectory.csv"));
            }
        } else if (id == IdStep3) {
            appendLog("Step 3: RTK float trajectory...");
            const string result = runRtkFloatStep(in);
            appendLog(result);
            if (stepSucceeded(result)) {
                setStepEnabled(4);
                showPreview(L"Step 3 - RTK float trajectory",
                            pathJoin(getText(IdOutDirEdit), L"rtk_float_trajectory.bmp"),
                            pathJoin(getText(IdOutDirEdit), L"rtk_float_trajectory.csv"));
            }
        } else if (id == IdStep4) {
            appendLog("Step 4: RTK fixed trajectory...");
            const string result = runRtkFixedStep(in);
            appendLog(result);
            if (stepSucceeded(result)) {
                setStepEnabled(5);
                showPreview(L"Step 4 - RTK fixed trajectory",
                            pathJoin(getText(IdOutDirEdit), L"rtk_fixed_trajectory.bmp"),
                            pathJoin(getText(IdOutDirEdit), L"rtk_fixed_trajectory.csv"));
            }
        } else if (id == IdStep5) {
            appendLog("Step 5: RTK/INS loose-coupled processing...");
            const string result = runRtkInsLooseStep(in);
            appendLog(result);
            if (stepSucceeded(result)) {
                showPreview(L"Step 5 - RTK/INS loose-coupled trajectory",
                            pathJoin(getText(IdOutDirEdit), L"rtkins_loose_trajectory.bmp"),
                            pathJoin(getText(IdOutDirEdit), L"rtkins_loose_trajectory.csv"));
            }
        } else if (id == IdStep6) {
            appendLog("Pure INS and Allan variance...");
            const string result = runPureInsAllanStep(in);
            appendLog(result);
            if (stepSucceeded(result)) {
                showPreview(L"IMU Allan variance analysis",
                            pathJoin(getText(IdOutDirEdit), L"allan_variance.bmp"),
                            pathJoin(getText(IdOutDirEdit), L"allan_variance.csv"));
                setAllanResult(pathJoin(getText(IdOutDirEdit), L"allan_variance.bmp"),
                               pathJoin(getText(IdOutDirEdit), L"allan_variance.csv"));
            }
        }
    } catch (const exception& e) {
        appendLog(string("ERROR: ") + e.what());
        MessageBoxA(gMain, e.what(), "NavFusion", MB_ICONERROR | MB_OK);
    }
}

LRESULT CALLBACK wndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    // Win32 消息处理函数。窗口创建、按钮点击、窗口关闭等事件都会进入这里。
    switch (msg) {
    case WM_CREATE:
        gMain = hwnd;
        {
        const wstring root = findProjectRoot();
        createPathRow(hwnd, 20, L"Rover O", IdRoverObsEdit, IdBrowseRoverObs);
        createPathRow(hwnd, 58, L"Nav P", IdNavEdit, IdBrowseNav);
        createPathRow(hwnd, 96, L"IMR", IdImuEdit, IdBrowseImu);
        createPathRow(hwnd, 134, L"Base O", IdBaseObsEdit, IdBrowseBaseObs);
        createPathRow(hwnd, 172, L"Output", IdOutDirEdit, IdBrowseOutDir);

        setText(IdRoverObsEdit, pathJoin(root, L"sj\\Observation.26o"));
        setText(IdNavEdit, pathJoin(root, L"sj\\Navigation.26p"));
        setText(IdImuEdit, pathJoin(root, L"sj\\IMU_Raw_Data.dat.imr"));
        setText(IdBaseObsEdit, pathJoin(root, L"sj\\Base_Station.rnx"));
        setText(IdOutDirEdit, pathJoin(root, L"output"));
        }

        CreateWindowW(L"BUTTON", L"1 Check data", WS_CHILD | WS_VISIBLE,
                      20, 222, 160, 34, hwnd, controlMenu(IdStep1), nullptr, nullptr);
        CreateWindowW(L"BUTTON", L"2 SPP track", WS_CHILD | WS_VISIBLE,
                      190, 222, 140, 34, hwnd, controlMenu(IdStep2), nullptr, nullptr);
        CreateWindowW(L"BUTTON", L"3 RTK float", WS_CHILD | WS_VISIBLE,
                      340, 222, 150, 34, hwnd, controlMenu(IdStep3), nullptr, nullptr);
        CreateWindowW(L"BUTTON", L"4 RTK fixed", WS_CHILD | WS_VISIBLE,
                      500, 222, 150, 34, hwnd, controlMenu(IdStep4), nullptr, nullptr);
        CreateWindowW(L"BUTTON", L"5 RTK/INS LC", WS_CHILD | WS_VISIBLE,
                      660, 222, 170, 34, hwnd, controlMenu(IdStep5), nullptr, nullptr);
        CreateWindowW(L"BUTTON", L"INS + Allan", WS_CHILD | WS_VISIBLE,
                      20, 264, 180, 34, hwnd, controlMenu(IdStep6), nullptr, nullptr);
        CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
                        WS_CHILD | WS_VISIBLE | WS_VSCROLL | ES_MULTILINE | ES_READONLY | ES_AUTOVSCROLL,
                        20, 310, 825, 230, hwnd, controlMenu(IdLog), nullptr, nullptr);
        CreateWindowW(L"STATIC", L"Preview", WS_CHILD | WS_VISIBLE,
                      880, 20, 520, 24, hwnd, controlMenu(IdImageTitle), nullptr, nullptr);
        CreateWindowExW(WS_EX_CLIENTEDGE, L"STATIC", L"",
                        WS_CHILD | WS_VISIBLE | SS_BITMAP | SS_CENTERIMAGE,
                        880, 54, 520, 360, hwnd, controlMenu(IdImage), nullptr, nullptr);
        CreateWindowW(L"BUTTON", L"Save current image...", WS_CHILD | WS_VISIBLE,
                      880, 424, 180, 30, hwnd, controlMenu(IdSavePreview), nullptr, nullptr);
        CreateWindowW(L"BUTTON", L"Save current CSV...", WS_CHILD | WS_VISIBLE,
                      1070, 424, 180, 30, hwnd, controlMenu(IdSaveCsv), nullptr, nullptr);
        CreateWindowW(L"BUTTON", L"Save Allan image...", WS_CHILD | WS_VISIBLE,
                      880, 462, 180, 30, hwnd, controlMenu(IdSaveAllanPreview), nullptr, nullptr);
        CreateWindowW(L"BUTTON", L"Save Allan CSV...", WS_CHILD | WS_VISIBLE,
                      1070, 462, 180, 30, hwnd, controlMenu(IdSaveAllanCsv), nullptr, nullptr);
        CreateWindowW(L"STATIC", L"Each successful calculation step writes CSV/SVG/BMP files and shows the BMP preview here.",
                      WS_CHILD | WS_VISIBLE,
                      880, 500, 520, 40, hwnd, nullptr, nullptr, nullptr);
        setStepEnabled(1);
        EnableWindow(item(IdSavePreview), FALSE);
        EnableWindow(item(IdSaveCsv), FALSE);
        EnableWindow(item(IdSaveAllanPreview), FALSE);
        EnableWindow(item(IdSaveAllanCsv), FALSE);
        appendLog("Select rover O, navigation P, IMR, base O and output directory, then run the steps in order.");
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case IdBrowseRoverObs:
            chooseFile(hwnd, L"Select rover observation file", L"RINEX O (*.o;*.26o)\0*.o;*.26o\0All Files\0*.*\0", IdRoverObsEdit);
            break;
        case IdBrowseNav:
            chooseFile(hwnd, L"Select navigation file", L"RINEX P/NAV (*.p;*.26p;*.n;*.nav)\0*.p;*.26p;*.n;*.nav\0All Files\0*.*\0", IdNavEdit);
            break;
        case IdBrowseImu:
            chooseFile(hwnd, L"Select IMR file", L"IMR (*.imr)\0*.imr\0All Files\0*.*\0", IdImuEdit);
            break;
        case IdBrowseBaseObs:
            chooseFile(hwnd, L"Select base observation file", L"RINEX O (*.o;*.26o)\0*.o;*.26o\0All Files\0*.*\0", IdBaseObsEdit);
            break;
        case IdBrowseOutDir:
            chooseFolder(hwnd, IdOutDirEdit);
            break;
        case IdStep1:
        case IdStep2:
        case IdStep3:
        case IdStep4:
        case IdStep5:
        case IdStep6:
            runStep(LOWORD(wp));
            break;
        case IdSavePreview:
            saveCurrentPreview(hwnd);
            break;
        case IdSaveCsv:
            saveCurrentCsv(hwnd);
            break;
        case IdSaveAllanPreview:
            saveCurrentAllanPreview(hwnd);
            break;
        case IdSaveAllanCsv:
            saveCurrentAllanCsv(hwnd);
            break;
        }
        return 0;
    case WM_DESTROY:
        if (gPreviewBitmap) DeleteObject(gPreviewBitmap);
        cleanupOutputDir(getText(IdOutDirEdit));
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

} // namespace

int runApp(HINSTANCE instance, int showCmd) {
    // 注册窗口类、创建主窗口、进入消息循环。
    // 这是 Win32 GUI 程序的标准启动流程。
    const wchar_t* className = L"NavFusionProcessWindow";
    WNDCLASSW wc{};
    wc.lpfnWndProc = wndProc;
    wc.hInstance = instance;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    wc.lpszClassName = className;
    RegisterClassW(&wc);

    HWND hwnd = CreateWindowW(className, L"NavFusion GNSS/INS processing tool",
                              WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
                              CW_USEDEFAULT, CW_USEDEFAULT, kWidth, kHeight,
                              nullptr, nullptr, instance, nullptr);
    if (!hwnd) return 1;
    ShowWindow(hwnd, showCmd);
    UpdateWindow(hwnd);

    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return static_cast<int>(msg.wParam);
}
