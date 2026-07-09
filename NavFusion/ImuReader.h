#pragma once

/*
 * ImuReader.h
 * -----------
 * IMR 原始惯导文件读取接口。
 *
 * 当前数据文件头是 "$IMURAW"，后面按 32 字节一条记录读取。
 * Reader 会把原始增量换算成 ImuSample，供 INS 机械编排使用。
 */

#include <string>
#include <vector>

#include "Types.h"

std::vector<ImuSample> readImrFile(const std::string& path);
