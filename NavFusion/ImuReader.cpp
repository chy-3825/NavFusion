#include "ImuReader.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>

/*
 * ImuReader.cpp
 * -------------
 * 读取 IMR 原始惯导文件。
 *
 * 本项目数据的 IMR 文件头是 "$IMURAW"，头部长度 512 字节；
 * 后面每条记录 32 字节：
 *   double time + int32 gx + int32 gy + int32 gz + int32 ax + int32 ay + int32 az
 *
 * 由于手头没有完整的设备说明书，比例因子先按数据量级做换算，
 * 用来给 INS 模块提供相对预测量。
 */

namespace {

template <typename T>
T readValue(const char* p) {
    // 从二进制字节中读取指定类型。IMR 是小端格式，Windows/x86 环境下可直接拷贝。
    T v{};
    std::copy(p, p + sizeof(T), reinterpret_cast<char*>(&v));
    return v;
}

} // namespace

std::vector<ImuSample> readImrFile(const std::string& path) {
    // IMR 读取入口：校验文件头、逐条读取记录、把原始整数增量换算成 ImuSample。
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Cannot open IMR file: " + path);
    }

    char header[512]{};
    in.read(header, sizeof(header));
    if (in.gcount() != sizeof(header) || std::string(header, header + 7) != "$IMURAW") {
        throw std::runtime_error("Unsupported IMR file header: " + path);
    }

    constexpr double gyroDeltaScale = 1.0e-9;
    constexpr double accelDeltaScale = 1.0e-11;

    std::vector<ImuSample> samples;
    samples.reserve(420000);

    char rec[32]{};
    double lastTime = 0.0;
    bool hasLast = false;
    while (in.read(rec, sizeof(rec))) {
        const double time = readValue<double>(rec);
        const int32_t gx = readValue<int32_t>(rec + 8);
        const int32_t gy = readValue<int32_t>(rec + 12);
        const int32_t gz = readValue<int32_t>(rec + 16);
        const int32_t ax = readValue<int32_t>(rec + 20);
        const int32_t ay = readValue<int32_t>(rec + 24);
        const int32_t az = readValue<int32_t>(rec + 28);
        const double dt = hasLast ? std::max(1.0e-4, time - lastTime) : 0.005;

        ImuSample s;
        s.time = time;
        s.gyro = { gx * gyroDeltaScale / dt, gy * gyroDeltaScale / dt, gz * gyroDeltaScale / dt };
        s.acc = { ax * accelDeltaScale / dt, ay * accelDeltaScale / dt, az * accelDeltaScale / dt };
        samples.push_back(s);

        lastTime = time;
        hasLast = true;
    }

    std::sort(samples.begin(), samples.end(), [](const ImuSample& a, const ImuSample& b) {
        return a.time < b.time;
    });
    samples.erase(std::unique(samples.begin(), samples.end(), [](const ImuSample& a, const ImuSample& b) {
        return std::abs(a.time - b.time) < 1.0e-7;
    }), samples.end());
    return samples;
}
