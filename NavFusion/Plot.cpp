#include "Plot.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

#include "Geo.h"

/*
 * Plot.cpp
 * --------
 * 负责把轨迹结果保存成三种形式：
 *
 * 1. CSV：数值结果，方便后续用 Excel、MATLAB、Python 分析；
 * 2. SVG：高清矢量图，适合放报告，放大也不会糊；
 * 3. BMP：Win32 STATIC 控件可以直接加载，适合程序右侧预览。
 *
 * 绘图时不直接使用 ECEF 坐标，而是把轨迹点相对第一个点转换成局部 East/North 平面坐标。
 */

namespace {

struct EnPoint {
    // 绘图使用局部东/北坐标，横轴East，纵轴North。
    double e = 0.0;
    double n = 0.0;
};

std::vector<EnPoint> trajectoryToEn(const Trajectory& trajectory,
                                    double& minE,
                                    double& maxE,
                                    double& minN,
                                    double& maxN) {
    // 以轨迹第一个点为局部参考原点，把 ECEF 坐标差转换成 East/North。
    // 这样图上的单位就是米，便于查看轨迹相对起点的变化。
    std::vector<EnPoint> en;
    minE = std::numeric_limits<double>::max();
    maxE = -std::numeric_limits<double>::max();
    minN = std::numeric_limits<double>::max();
    maxN = -std::numeric_limits<double>::max();
    if (trajectory.empty()) return en;

    if (trajectory.front().quality == 5) {
        en.reserve(trajectory.size());
        for (const auto& p : trajectory) {
            const double e = p.ecef.x - trajectory.front().ecef.x;
            const double n = p.ecef.y - trajectory.front().ecef.y;
            en.push_back({ e, n });
            minE = std::min(minE, e);
            maxE = std::max(maxE, e);
            minN = std::min(minN, n);
            maxN = std::max(maxN, n);
        }
        return en;
    }

    double lat0 = 0.0;
    double lon0 = 0.0;
    double h0 = 0.0;
    ecefToLlh(trajectory.front().ecef, lat0, lon0, h0);
    en.reserve(trajectory.size());
    for (const auto& p : trajectory) {
        const Vec3 diff{
            p.ecef.x - trajectory.front().ecef.x,
            p.ecef.y - trajectory.front().ecef.y,
            p.ecef.z - trajectory.front().ecef.z
        };
        const Vec3 ned = ecefToNed(diff, lat0, lon0);
        en.push_back({ ned.y, ned.x });
        minE = std::min(minE, ned.y);
        maxE = std::max(maxE, ned.y);
        minN = std::min(minN, ned.x);
        maxN = std::max(maxN, ned.x);
    }
    return en;
}

void paddedRange(double& minV, double& maxV) {
    // 给图框留边距，避免轨迹贴住边框；空轨迹时给默认范围。
    if (minV == std::numeric_limits<double>::max()) {
        minV = -1.0;
        maxV = 1.0;
        return;
    }
    double span = std::max(1.0, maxV - minV);
    const double pad = span * 0.08;
    minV -= pad;
    maxV += pad;
    if (std::abs(maxV - minV) < 1e-9) {
        minV -= 1.0;
        maxV += 1.0;
    }
}

} // namespace

void writeTrajectoryCsv(const std::string& path, const Trajectory& trajectory) {
    // CSV 保存原始 ECEF 坐标，而不是绘图用的 East/North，便于后续分析。
    std::ofstream out(path);
    if (!trajectory.empty() && trajectory.front().quality == 5) {
        out << "time,relative_x_m,relative_y_m,relative_z_m,quality\n";
    } else {
        out << "time,x_ecef_m,y_ecef_m,z_ecef_m,quality\n";
    }
    for (const auto& p : trajectory) {
        out << std::fixed << std::setprecision(4)
            << p.time << ","
            << p.ecef.x << ","
            << p.ecef.y << ","
            << p.ecef.z << ","
            << p.quality << "\n";
    }
}

void writeTrajectorySvg(const std::string& path, const Trajectory& trajectory, const std::string& title) {
    // SVG 是文本形式的矢量图。这里手动写 SVG 标签，避免引入额外绘图库。
    double minE = 0.0;
    double maxE = 0.0;
    double minN = 0.0;
    double maxN = 0.0;
    const auto en = trajectoryToEn(trajectory, minE, maxE, minN, maxN);
    paddedRange(minE, maxE);
    paddedRange(minN, maxN);

    constexpr double width = 1000.0;
    constexpr double height = 760.0;
    constexpr double left = 80.0;
    constexpr double right = 40.0;
    constexpr double top = 60.0;
    constexpr double bottom = 80.0;
    constexpr double plotW = width - left - right;
    constexpr double plotH = height - top - bottom;
    const double spanE = std::max(1.0, maxE - minE);
    const double spanN = std::max(1.0, maxN - minN);

    std::ofstream out(path);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1000\" height=\"760\" viewBox=\"0 0 1000 760\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
    out << "<text x=\"80\" y=\"32\" font-size=\"20\" font-family=\"Arial\">" << title << "</text>\n";
    out << "<rect x=\"80\" y=\"60\" width=\"880\" height=\"620\" fill=\"#fafafa\" stroke=\"#333\"/>\n";
    for (int i = 1; i < 5; ++i) {
        const double x = left + plotW * i / 5.0;
        const double y = top + plotH * i / 5.0;
        out << "<line x1=\"" << std::fixed << std::setprecision(1) << x << "\" y1=\"" << top
            << "\" x2=\"" << x << "\" y2=\"" << top + plotH << "\" stroke=\"#ddd\"/>\n";
        out << "<line x1=\"" << left << "\" y1=\"" << y
            << "\" x2=\"" << left + plotW << "\" y2=\"" << y << "\" stroke=\"#ddd\"/>\n";
    }
    if (!en.empty()) {
        out << "<polyline fill=\"none\" stroke=\"#0d47a1\" stroke-width=\"2\" points=\"";
        for (const auto& p : en) {
            const double x = left + (p.e - minE) / spanE * plotW;
            const double y = top + (maxN - p.n) / spanN * plotH;
            out << std::fixed << std::setprecision(1) << x << "," << y << " ";
        }
        out << "\"/>\n";
        auto drawPoint = [&](const EnPoint& p, const char* color, const char* label) {
            const double x = left + (p.e - minE) / spanE * plotW;
            const double y = top + (maxN - p.n) / spanN * plotH;
            out << "<circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"5\" fill=\"" << color << "\"/>\n";
            out << "<text x=\"" << x + 8 << "\" y=\"" << y - 8
                << "\" font-size=\"13\" font-family=\"Arial\" fill=\"" << color << "\">" << label << "</text>\n";
        };
        drawPoint(en.front(), "#2e7d32", "start");
        drawPoint(en.back(), "#b71c1c", "end");
    } else {
        out << "<text x=\"360\" y=\"370\" font-size=\"16\" font-family=\"Arial\" fill=\"#666\">No trajectory points yet</text>\n";
    }
    out << "<text x=\"440\" y=\"730\" font-size=\"14\" font-family=\"Arial\">East / m</text>\n";
    out << "<text x=\"18\" y=\"380\" font-size=\"14\" font-family=\"Arial\" transform=\"rotate(-90 18,380)\">North / m</text>\n";
    out << "<text x=\"80\" y=\"705\" font-size=\"13\" font-family=\"Arial\">solutions: " << en.size()
        << ", E range " << std::fixed << std::setprecision(1) << minE << " to " << maxE
        << " m, N range " << minN << " to " << maxN << " m</text>\n";
    out << "</svg>\n";
}

void putPixel(std::vector<uint8_t>& img, int w, int h, int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    // BMP缓冲区采用BGR顺序。
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    const int i = (y * w + x) * 3;
    img[i + 0] = b;
    img[i + 1] = g;
    img[i + 2] = r;
}

void drawLine(std::vector<uint8_t>& img, int w, int h, int x0, int y0, int x1, int y1,
              uint8_t r, uint8_t g, uint8_t b) {
    // Bresenham折线绘制，用于右侧预览图。
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        putPixel(img, w, h, x0, y0, r, g, b);
        putPixel(img, w, h, x0 + 1, y0, r, g, b);
        putPixel(img, w, h, x0, y0 + 1, r, g, b);
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void writeLe32(std::ofstream& out, uint32_t v) {
    char b[4] = {
        static_cast<char>(v & 0xff),
        static_cast<char>((v >> 8) & 0xff),
        static_cast<char>((v >> 16) & 0xff),
        static_cast<char>((v >> 24) & 0xff)
    };
    out.write(b, sizeof(b));
}

void writeLe16(std::ofstream& out, uint16_t v) {
    char b[2] = {
        static_cast<char>(v & 0xff),
        static_cast<char>((v >> 8) & 0xff)
    };
    out.write(b, sizeof(b));
}

void writeTrajectoryBmp(const std::string& path, const Trajectory& trajectory) {
    // BMP 用于程序右侧预览。Win32 的 STATIC + SS_BITMAP 对 BMP 支持最直接。
    constexpr int w = 520;
    constexpr int h = 360;
    std::vector<uint8_t> img(w * h * 3, 255);

    for (int x = 40; x < w - 28; x += 64) {
        drawLine(img, w, h, x, 32, x, h - 46, 222, 226, 230);
    }
    for (int y = 32; y < h - 46; y += 48) {
        drawLine(img, w, h, 40, y, w - 28, y, 222, 226, 230);
    }
    drawLine(img, w, h, 40, h - 46, w - 28, h - 46, 80, 80, 80);
    drawLine(img, w, h, 40, 32, 40, h - 46, 80, 80, 80);

    double minE = 0.0;
    double maxE = 0.0;
    double minN = 0.0;
    double maxN = 0.0;
    const auto en = trajectoryToEn(trajectory, minE, maxE, minN, maxN);
    paddedRange(minE, maxE);
    paddedRange(minN, maxN);
    if (en.size() > 1) {
        const double spanE = std::max(1.0, maxE - minE);
        const double spanN = std::max(1.0, maxN - minN);
        int lastX = 0;
        int lastY = 0;
        bool haveLast = false;
        for (const auto& p : en) {
            const int x = 40 + static_cast<int>((p.e - minE) / spanE * (w - 68));
            const int y = h - 46 - static_cast<int>((p.n - minN) / spanN * (h - 78));
            if (haveLast) drawLine(img, w, h, lastX, lastY, x, y, 16, 88, 180);
            lastX = x;
            lastY = y;
            haveLast = true;
        }
        for (int dy = -3; dy <= 3; ++dy)
            for (int dx = -3; dx <= 3; ++dx)
                if (dx * dx + dy * dy <= 9) putPixel(img, w, h, lastX + dx, lastY + dy, 183, 28, 28);
    }

    const int rowStride = ((w * 3 + 3) / 4) * 4;
    const uint32_t pixelBytes = rowStride * h;
    const uint32_t fileBytes = 14 + 40 + pixelBytes;
    std::ofstream out(path, std::ios::binary);
    out.put('B');
    out.put('M');
    writeLe32(out, fileBytes);
    writeLe16(out, 0);
    writeLe16(out, 0);
    writeLe32(out, 14 + 40);
    writeLe32(out, 40);
    writeLe32(out, w);
    writeLe32(out, h);
    writeLe16(out, 1);
    writeLe16(out, 24);
    writeLe32(out, 0);
    writeLe32(out, pixelBytes);
    writeLe32(out, 2835);
    writeLe32(out, 2835);
    writeLe32(out, 0);
    writeLe32(out, 0);

    std::vector<uint8_t> row(rowStride, 255);
    for (int y = h - 1; y >= 0; --y) {
        std::fill(row.begin(), row.end(), 255);
        std::copy(img.begin() + y * w * 3, img.begin() + (y + 1) * w * 3, row.begin());
        out.write(reinterpret_cast<const char*>(row.data()), row.size());
    }
}

namespace {

double vecMagnitude(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

std::vector<EnPoint> allanToLogPoints(const AllanSeries& allan,
                                      bool gyro,
                                      double& minX,
                                      double& maxX,
                                      double& minY,
                                      double& maxY) {
    std::vector<EnPoint> points;
    minX = std::numeric_limits<double>::max();
    maxX = -std::numeric_limits<double>::max();
    minY = std::numeric_limits<double>::max();
    maxY = -std::numeric_limits<double>::max();
    for (const auto& p : allan) {
        const double y = gyro ? vecMagnitude(p.gyroDeviation) : vecMagnitude(p.accDeviation);
        if (p.tau <= 0.0 || y <= 0.0) continue;
        const double lx = std::log10(p.tau);
        const double ly = std::log10(y);
        points.push_back({ lx, ly });
        minX = std::min(minX, lx);
        maxX = std::max(maxX, lx);
        minY = std::min(minY, ly);
        maxY = std::max(maxY, ly);
    }
    return points;
}

void mergeRange(double& minA, double& maxA, double minB, double maxB) {
    if (minB == std::numeric_limits<double>::max()) return;
    minA = std::min(minA, minB);
    maxA = std::max(maxA, maxB);
}

} // namespace

void writeAllanCsv(const std::string& path, const AllanSeries& allan) {
    std::ofstream out(path);
    out << "tau_s,samples_per_cluster,"
        << "gyro_var_x,gyro_var_y,gyro_var_z,gyro_dev_x,gyro_dev_y,gyro_dev_z,"
        << "acc_var_x,acc_var_y,acc_var_z,acc_dev_x,acc_dev_y,acc_dev_z\n";
    for (const auto& p : allan) {
        out << std::scientific << std::setprecision(10)
            << p.tau << ","
            << p.samplesPerCluster << ","
            << p.gyroVariance.x << "," << p.gyroVariance.y << "," << p.gyroVariance.z << ","
            << p.gyroDeviation.x << "," << p.gyroDeviation.y << "," << p.gyroDeviation.z << ","
            << p.accVariance.x << "," << p.accVariance.y << "," << p.accVariance.z << ","
            << p.accDeviation.x << "," << p.accDeviation.y << "," << p.accDeviation.z << "\n";
    }
}

void writeAllanSvg(const std::string& path, const AllanSeries& allan, const std::string& title) {
    double minGx = 0.0, maxGx = 0.0, minGy = 0.0, maxGy = 0.0;
    double minAx = 0.0, maxAx = 0.0, minAy = 0.0, maxAy = 0.0;
    const auto gyro = allanToLogPoints(allan, true, minGx, maxGx, minGy, maxGy);
    const auto acc = allanToLogPoints(allan, false, minAx, maxAx, minAy, maxAy);

    double minX = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    mergeRange(minX, maxX, minGx, maxGx);
    mergeRange(minX, maxX, minAx, maxAx);
    mergeRange(minY, maxY, minGy, maxGy);
    mergeRange(minY, maxY, minAy, maxAy);
    paddedRange(minX, maxX);
    paddedRange(minY, maxY);

    constexpr double width = 1000.0;
    constexpr double height = 760.0;
    constexpr double left = 90.0;
    constexpr double right = 50.0;
    constexpr double top = 70.0;
    constexpr double bottom = 90.0;
    constexpr double plotW = width - left - right;
    constexpr double plotH = height - top - bottom;
    const double spanX = std::max(1.0e-9, maxX - minX);
    const double spanY = std::max(1.0e-9, maxY - minY);

    std::ofstream out(path);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1000\" height=\"760\" viewBox=\"0 0 1000 760\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
    out << "<text x=\"90\" y=\"38\" font-size=\"20\" font-family=\"Arial\">" << title << "</text>\n";
    out << "<rect x=\"90\" y=\"70\" width=\"860\" height=\"600\" fill=\"#fafafa\" stroke=\"#333\"/>\n";
    for (int i = 1; i < 5; ++i) {
        const double x = left + plotW * i / 5.0;
        const double y = top + plotH * i / 5.0;
        out << "<line x1=\"" << x << "\" y1=\"" << top << "\" x2=\"" << x
            << "\" y2=\"" << top + plotH << "\" stroke=\"#ddd\"/>\n";
        out << "<line x1=\"" << left << "\" y1=\"" << y << "\" x2=\"" << left + plotW
            << "\" y2=\"" << y << "\" stroke=\"#ddd\"/>\n";
    }
    auto writeLine = [&](const std::vector<EnPoint>& points, const char* color) {
        if (points.empty()) return;
        out << "<polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\"2\" points=\"";
        for (const auto& p : points) {
            const double x = left + (p.e - minX) / spanX * plotW;
            const double y = top + (maxY - p.n) / spanY * plotH;
            out << std::fixed << std::setprecision(1) << x << "," << y << " ";
        }
        out << "\"/>\n";
    };
    writeLine(gyro, "#0d47a1");
    writeLine(acc, "#b71c1c");
    out << "<text x=\"720\" y=\"105\" font-size=\"14\" font-family=\"Arial\" fill=\"#0d47a1\">gyro deviation norm</text>\n";
    out << "<text x=\"720\" y=\"128\" font-size=\"14\" font-family=\"Arial\" fill=\"#b71c1c\">acc deviation norm</text>\n";
    out << "<text x=\"420\" y=\"725\" font-size=\"14\" font-family=\"Arial\">log10(tau / s)</text>\n";
    out << "<text x=\"18\" y=\"430\" font-size=\"14\" font-family=\"Arial\" transform=\"rotate(-90 18,430)\">log10(Allan deviation)</text>\n";
    out << "<text x=\"90\" y=\"698\" font-size=\"13\" font-family=\"Arial\">points: " << allan.size()
        << ", tau " << std::fixed << std::setprecision(4) << std::pow(10.0, minX)
        << " to " << std::pow(10.0, maxX) << " s</text>\n";
    out << "</svg>\n";
}

void writeAllanBmp(const std::string& path, const AllanSeries& allan) {
    constexpr int w = 520;
    constexpr int h = 360;
    std::vector<uint8_t> img(w * h * 3, 255);
    for (int x = 44; x < w - 28; x += 64) {
        drawLine(img, w, h, x, 32, x, h - 46, 222, 226, 230);
    }
    for (int y = 32; y < h - 46; y += 48) {
        drawLine(img, w, h, 44, y, w - 28, y, 222, 226, 230);
    }
    drawLine(img, w, h, 44, h - 46, w - 28, h - 46, 80, 80, 80);
    drawLine(img, w, h, 44, 32, 44, h - 46, 80, 80, 80);

    double minGx = 0.0, maxGx = 0.0, minGy = 0.0, maxGy = 0.0;
    double minAx = 0.0, maxAx = 0.0, minAy = 0.0, maxAy = 0.0;
    const auto gyro = allanToLogPoints(allan, true, minGx, maxGx, minGy, maxGy);
    const auto acc = allanToLogPoints(allan, false, minAx, maxAx, minAy, maxAy);
    double minX = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    mergeRange(minX, maxX, minGx, maxGx);
    mergeRange(minX, maxX, minAx, maxAx);
    mergeRange(minY, maxY, minGy, maxGy);
    mergeRange(minY, maxY, minAy, maxAy);
    paddedRange(minX, maxX);
    paddedRange(minY, maxY);
    const double spanX = std::max(1.0e-9, maxX - minX);
    const double spanY = std::max(1.0e-9, maxY - minY);

    auto drawCurve = [&](const std::vector<EnPoint>& points, uint8_t r, uint8_t g, uint8_t b) {
        int lastX = 0;
        int lastY = 0;
        bool haveLast = false;
        for (const auto& p : points) {
            const int x = 44 + static_cast<int>((p.e - minX) / spanX * (w - 72));
            const int y = h - 46 - static_cast<int>((p.n - minY) / spanY * (h - 78));
            if (haveLast) drawLine(img, w, h, lastX, lastY, x, y, r, g, b);
            lastX = x;
            lastY = y;
            haveLast = true;
        }
    };
    drawCurve(gyro, 16, 88, 180);
    drawCurve(acc, 183, 28, 28);

    const int rowStride = ((w * 3 + 3) / 4) * 4;
    const uint32_t pixelBytes = rowStride * h;
    const uint32_t fileBytes = 14 + 40 + pixelBytes;
    std::ofstream out(path, std::ios::binary);
    out.put('B');
    out.put('M');
    writeLe32(out, fileBytes);
    writeLe16(out, 0);
    writeLe16(out, 0);
    writeLe32(out, 14 + 40);
    writeLe32(out, 40);
    writeLe32(out, w);
    writeLe32(out, h);
    writeLe16(out, 1);
    writeLe16(out, 24);
    writeLe32(out, 0);
    writeLe32(out, pixelBytes);
    writeLe32(out, 2835);
    writeLe32(out, 2835);
    writeLe32(out, 0);
    writeLe32(out, 0);

    std::vector<uint8_t> row(rowStride, 255);
    for (int y = h - 1; y >= 0; --y) {
        std::fill(row.begin(), row.end(), 255);
        std::copy(img.begin() + y * w * 3, img.begin() + (y + 1) * w * 3, row.begin());
        out.write(reinterpret_cast<const char*>(row.data()), row.size());
    }
}
