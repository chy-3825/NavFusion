#include "RinexReader.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

/*
 * RinexReader.cpp
 * ---------------
 * RINEX 是 GNSS 数据常用的文本交换格式。
 *
 * 本文件实现了项目中使用的 RINEX 读取器：
 * - 从观测文件读取 GPS C1C 伪距和 L1C 载波相位；
 * - 从观测文件头读取接收机近似坐标；
 * - 从导航文件读取 GPS 广播星历参数；
 * - 把文本文件整理成 Types.h 中定义的结构体。
 *
 * 注意：这里故意不做定位解算。Reader 只负责“把文件读成数据”，
 * SPP/RTK 的数学计算放在 Solver 文件中，这样模块边界更清楚。
 */

namespace {

int daysFromCivil(int y, unsigned m, unsigned d) {
    // 把公历年月日转换成从 Unix epoch 开始的天数。
    // 这里用它间接计算 GPS 周内秒，避免依赖额外日期库。
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<int>(doe) - 719468;
}

double gpsTow(int y, int mo, int d, int h, int mi, double sec) {
    // GPS 时间从 1980-01-06 开始。RINEX 历元给的是年月日时分秒，
    // 解算时更方便使用“GPS 周内秒 tow”与星历 toe/toc 做匹配。
    const int unixDays = daysFromCivil(y, static_cast<unsigned>(mo), static_cast<unsigned>(d));
    const int gpsDays = unixDays - daysFromCivil(1980, 1, 6);
    double tow = std::fmod(gpsDays * 86400.0 + h * 3600.0 + mi * 60.0 + sec, 604800.0);
    if (tow < 0.0) tow += 604800.0;
    return tow;
}

double parseRinexDouble(std::string s) {
    // RINEX 科学计数法可能写成 1.23D+04，C++ 默认只认识 E 指数，
    // 所以解析前统一把 D/d 替换成 E。
    std::replace(s.begin(), s.end(), 'D', 'E');
    std::replace(s.begin(), s.end(), 'd', 'E');
    std::istringstream iss(s);
    double v = 0.0;
    iss >> v;
    return v;
}

std::vector<double> parseNavValues(const std::string& line) {
    // RINEX 3 导航文件每行通常有 4 个 19 字符宽度的浮点字段。
    // 用固定宽度切片，比按空格拆分更适合 RINEX 这种对齐格式。
    std::vector<double> out;
    for (int i = 0; i < 4; ++i) {
        const int start = 4 + i * 19;
        if (start >= static_cast<int>(line.size())) break;
        out.push_back(parseRinexDouble(line.substr(start, std::min(19, static_cast<int>(line.size()) - start))));
    }
    return out;
}

std::map<char, std::vector<std::string>> readObsTypes(std::ifstream& in, Vec3& approx) {
    // 读取观测文件头。
    // SYS / # / OBS TYPES 告诉我们每个系统有哪些观测值，以及它们在数据行中的顺序。
    // 比如 GPS 行中 C1C 排第 0 个、L1C 排第 1 个，后面按索引读取。
    std::map<char, std::vector<std::string>> types;
    std::string line;
    while (std::getline(in, line)) {
        if (line.find("APPROX POSITION XYZ") != std::string::npos) {
            std::istringstream iss(line.substr(0, 60));
            iss >> approx.x >> approx.y >> approx.z;
        } else if (line.find("SYS / # / OBS TYPES") != std::string::npos && line.size() >= 7) {
            const char sys = line[0];
            int n = 0;
            std::istringstream iss(line.substr(3, 3));
            iss >> n;
            auto& v = types[sys];
            for (int i = 0; i < std::min(n, 13); ++i) {
                const int start = 7 + i * 4;
                if (start + 3 <= static_cast<int>(line.size())) v.push_back(line.substr(start, 3));
            }
            while (static_cast<int>(v.size()) < n && std::getline(in, line)) {
                for (int i = 0; i < 13 && static_cast<int>(v.size()) < n; ++i) {
                    const int start = 7 + i * 4;
                    if (start + 3 <= static_cast<int>(line.size())) v.push_back(line.substr(start, 3));
                }
                if (line.find("SYS / # / OBS TYPES") != std::string::npos) break;
            }
        } else if (line.find("END OF HEADER") != std::string::npos) {
            break;
        }
    }
    return types;
}

double obsValueAt(const std::string& line, int obsIndex) {
    // RINEX 观测数据每个观测值占 16 个字符，前 14 个字符是数值。
    // obsIndex 表示当前要读取第几个观测值。
    const int start = 3 + obsIndex * 16;
    if (obsIndex < 0 || start >= static_cast<int>(line.size())) return 0.0;
    const int len = std::min(14, static_cast<int>(line.size()) - start);
    return parseRinexDouble(line.substr(start, len));
}

} // namespace

std::vector<GpsEphemeris> readGpsNavigationEphemerides(const std::string& path) {
    // 读取 GPS 广播星历。
    // 当前只处理 G 开头的 GPS 星历块，其他系统如北斗/伽利略可以以后扩展。
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open navigation file: " + path);

    std::string line;
    while (std::getline(in, line)) {
        if (line.find("END OF HEADER") != std::string::npos) break;
    }

    std::vector<GpsEphemeris> ephs;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] != 'G') continue;

        std::vector<std::string> block(8);
        block[0] = line;
        bool ok = true;
        for (int i = 1; i < 8; ++i) {
            if (!std::getline(in, block[i])) {
                ok = false;
                break;
            }
        }
        if (!ok) break;

        GpsEphemeris e;
        e.prn = std::stoi(block[0].substr(1, 2));
        int year = 0, mon = 0, day = 0, hour = 0, minute = 0;
        double sec = 0.0;
        std::istringstream head(block[0].substr(3, 20));
        head >> year >> mon >> day >> hour >> minute >> sec;
        e.toc = gpsTow(year, mon, day, hour, minute, sec);
        e.af0 = parseRinexDouble(block[0].substr(23, 19));
        e.af1 = parseRinexDouble(block[0].substr(42, 19));
        e.af2 = parseRinexDouble(block[0].substr(61, 19));

        const auto l2 = parseNavValues(block[1]);
        const auto l3 = parseNavValues(block[2]);
        const auto l4 = parseNavValues(block[3]);
        const auto l5 = parseNavValues(block[4]);
        const auto l6 = parseNavValues(block[5]);
        const auto l7 = parseNavValues(block[6]);
        if (l2.size() < 4 || l3.size() < 4 || l4.size() < 4 ||
            l5.size() < 4 || l6.size() < 4 || l7.size() < 3) {
            continue;
        }

        e.iode = l2[0]; e.crs = l2[1]; e.dn = l2[2]; e.m0 = l2[3];
        e.cuc = l3[0]; e.e = l3[1]; e.cus = l3[2]; e.sqrtA = l3[3];
        e.toe = l4[0]; e.cic = l4[1]; e.omega0 = l4[2]; e.cis = l4[3];
        e.i0 = l5[0]; e.crc = l5[1]; e.omega = l5[2]; e.omegaDot = l5[3];
        e.idot = l6[0]; e.week = l6[2];
        e.tgd = l7[2];
        ephs.push_back(e);
    }
    return ephs;
}

std::vector<RinexEpoch> readGpsObservations(const std::string& path, Vec3& approxPosition) {
    // 读取 GPS 观测历元。
    // 每个以 '>' 开头的行表示一个历元，后面 nsat 行是该历元的卫星观测值。
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open RINEX observation file: " + path);

    const auto obsTypes = readObsTypes(in, approxPosition);
    const auto it = obsTypes.find('G');
    if (it == obsTypes.end()) return {};

    const auto c1It = std::find(it->second.begin(), it->second.end(), "C1C");
    const auto l1It = std::find(it->second.begin(), it->second.end(), "L1C");
    const int c1Index = c1It == it->second.end() ? -1 : static_cast<int>(c1It - it->second.begin());
    const int l1Index = l1It == it->second.end() ? -1 : static_cast<int>(l1It - it->second.begin());
    if (c1Index < 0 && l1Index < 0) return {};

    std::vector<RinexEpoch> epochs;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] != '>') continue;

        int y = 0, mo = 0, d = 0, h = 0, mi = 0, flag = 0, nsat = 0;
        double sec = 0.0;
        std::istringstream iss(line.substr(1));
        iss >> y >> mo >> d >> h >> mi >> sec >> flag >> nsat;

        RinexEpoch epoch;
        epoch.tow = gpsTow(y, mo, d, h, mi, sec);
        for (int i = 0; i < nsat && std::getline(in, line); ++i) {
            if (line.size() < 3 || line[0] != 'G') continue;

            int prn = 0;
            try {
                prn = std::stoi(line.substr(1, 3));
            } catch (...) {
                continue;
            }

            RinexObsValue obs;
            obs.c1 = obsValueAt(line, c1Index);
            obs.l1 = obsValueAt(line, l1Index);
            if (obs.c1 > 1.0e6 || std::abs(obs.l1) > 1.0e3) {
                epoch.gps[prn] = obs;
            }
        }
        if (!epoch.gps.empty()) epochs.push_back(epoch);
    }
    return epochs;
}

RinexObservationFile readRinexObservation(const std::string& path) {
    // 对外的观测文件读取入口：保存路径、近似坐标、历元数量和历元数据。
    RinexObservationFile file;
    file.path = path;
    file.epochs = readGpsObservations(path, file.approxPosition);
    file.epochCount = static_cast<int>(file.epochs.size());
    return file;
}

RinexNavigationFile readRinexNavigation(const std::string& path) {
    // 对外的导航文件读取入口：保存路径、星历数量和星历参数。
    RinexNavigationFile file;
    file.path = path;
    file.gpsEphemerides = readGpsNavigationEphemerides(path);
    file.ephemerisCount = static_cast<int>(file.gpsEphemerides.size());
    return file;
}
