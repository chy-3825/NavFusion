# NavFusion

NavFusion 是一个基于 Win32 C++ 的 GNSS/INS 轨迹解算与结果可视化程序。项目支持读取动态站 RINEX 观测文件、GPS 导航星历文件、IMR 惯导原始数据和基站观测文件，并按步骤完成 SPP、RTK float、RTK fixed/smoothed 以及 RTK/INS 松组合轨迹处理。

项目自带示例数据和 Windows 可执行程序，可以直接运行并复现完整处理流程。每个解算步骤都会生成轨迹 CSV、SVG 和 BMP 图像结果，图形界面中也可以预览当前轨迹，并将当前轨迹图或对应 CSV 另存到用户指定位置。

## 已验证功能

- 从默认 `sj` 文件夹读取示例数据。
- 在图形界面中手动选择动态站观测文件、导航星历文件、IMR 文件、基站观测文件和输出目录。
- 检查输入文件是否存在、输出目录是否可用。
- 使用 GPS `C1C` 伪距和广播星历进行 SPP 单点定位。
- 使用动态站/基站共同 GPS 卫星进行 RTK float 单差伪距解算。
- 在 RTK float 轨迹基础上，利用 `L1C` 载波相位连续性进行 fixed/smoothed 轨迹生成。
- 读取 IMR 原始惯导数据，进行基础 INS 相对位移积分。
- 使用 IMU 预测和 RTK fixed/smoothed 轨迹进行 RTK/INS 松组合处理。
- 每一步输出 `.csv`、`.svg` 和 `.bmp` 三种轨迹结果。
- 界面右侧预览当前步骤的 BMP 轨迹图。
- 通过按钮保存当前轨迹图和当前轨迹 CSV 到指定位置。
- 程序退出时清理 `output` 中自动生成的临时结果。

## 快速运行

项目已包含 Debug 可执行程序：

```text
x64/Debug/NavFusion.exe
```

直接运行会打开 Win32 图形界面。也可以使用命令行参数运行默认示例数据：

```text
x64/Debug/NavFusion.exe --run-spp-default
x64/Debug/NavFusion.exe --run-all-default
```

其中：

- `--run-spp-default`：只运行默认数据的 SPP 步骤。
- `--run-all-default`：依次运行 SPP、RTK float、RTK fixed/smoothed 和 RTK/INS 松组合步骤。

实际验证中，`--run-all-default` 可以生成以下结果：

```text
output/spp_trajectory.csv
output/spp_trajectory.svg
output/spp_trajectory.bmp

output/rtk_float_trajectory.csv
output/rtk_float_trajectory.svg
output/rtk_float_trajectory.bmp

output/rtk_fixed_trajectory.csv
output/rtk_fixed_trajectory.svg
output/rtk_fixed_trajectory.bmp

output/rtkins_loose_trajectory.csv
output/rtkins_loose_trajectory.svg
output/rtkins_loose_trajectory.bmp
```

## 输入数据

默认数据位于 `sj` 文件夹：

- `sj/Observation.26o`：动态站 RINEX 观测文件
- `sj/Navigation.26p`：GPS 广播星历文件
- `sj/IMU_Raw_Data.dat.imr`：IMR 原始惯导数据
- `sj/Base_Station.rnx`：基站 RINEX 观测文件

程序运行时会从可执行文件所在目录向上查找 `sj` 文件夹，因此外层项目目录改名后通常不需要修改源码中的默认数据路径。

## 图形界面流程

打开程序后，可以在界面中选择：

- Rover observation：动态站观测文件
- Navigation：导航星历文件
- IMR：惯导原始数据文件
- Base observation：基站观测文件
- Output：输出目录

选择完成后，按顺序点击：

1. `1 Check data`
2. `2 SPP track`
3. `3 RTK float`
4. `4 RTK fixed`
5. `5 RTK/INS LC`

每个成功完成的步骤都会在输出目录中生成对应轨迹文件，并在界面右侧显示当前步骤的轨迹图。

界面还提供：

- `Save current image...`：保存当前预览的轨迹图
- `Save current CSV...`：保存当前轨迹对应的 CSV 文件

保存按钮会把当前结果复制到用户指定位置。程序退出时只清理 `output` 中自动生成的临时结果，不会删除用户另存的文件。

## 处理流程

1. 检查动态站观测文件、导航星历文件、IMR 文件、基站观测文件和输出目录。
2. 读取动态站 RINEX 观测文件和 GPS 广播星历。
3. 使用 GPS `C1C` 伪距进行 SPP 单点定位，输出动态站 ECEF 轨迹。
4. 读取基站 RINEX 观测文件，匹配动态站和基站共同 GPS 卫星，进行 RTK float 单差伪距解算。
5. 在 float 轨迹基础上利用 `L1C` 载波连续性进行平滑，生成 RTK fixed/smoothed 轨迹。
6. 读取 IMR 数据，完成基础 INS 相对位移积分。
7. 使用 IMU 预测和 RTK fixed/smoothed 位置更新，生成 RTK/INS 松组合轨迹。
8. 为每一步输出 CSV、SVG 和 BMP 轨迹结果。

## 输出说明

默认输出目录为 `output`：

- `spp_trajectory.*`：SPP 单点定位轨迹
- `rtk_float_trajectory.*`：RTK float 单差伪距轨迹
- `rtk_fixed_trajectory.*`：基于载波连续性平滑后的 RTK fixed/smoothed 轨迹
- `rtkins_loose_trajectory.*`：RTK/INS 松组合轨迹

其中：

- CSV 保存轨迹点时间、ECEF 坐标和质量标记。
- SVG 用于高清查看或报告插图。
- BMP 用于 Win32 界面右侧轨迹预览。

## 代码结构

- `NavFusion/main.cpp`：程序入口，支持图形界面和默认数据自动运行参数。
- `NavFusion/App.cpp`：Win32 图形界面、文件选择、按钮流程、日志显示、轨迹预览和结果保存。
- `NavFusion/ProcessingPipeline.cpp`：连接界面和算法模块，负责输入检查、调用解算步骤和输出结果。
- `NavFusion/RinexReader.cpp`：读取 RINEX 观测文件和 GPS 导航星历。
- `NavFusion/SppSolver.cpp`：SPP 单点定位。
- `NavFusion/RtkSolver.cpp`：RTK float 和 fixed/smoothed 轨迹生成。
- `NavFusion/ImuReader.cpp`：读取 IMR 原始惯导文件。
- `NavFusion/InsSolver.cpp`：基础 INS 相对位移积分。
- `NavFusion/RtkInsLoose.cpp`：RTK/INS 松组合处理。
- `NavFusion/Geo.cpp`：WGS-84、LLH/ECEF、ECEF/NED 和正常重力等工具函数。
- `NavFusion/Plot.cpp`：输出 CSV、SVG 和 BMP 轨迹结果。

## 当前实现边界

本项目主要用于展示 GNSS/INS 数据读取、解算流程组织、轨迹输出和桌面界面集成能力。当前算法实现仍有一些简化：

- SPP 当前主要使用 GPS 单频 `C1C` 伪距，尚未加入完整电离层、对流层、多系统和多频改正。
- RTK float 当前采用共同卫星单差伪距方程。
- RTK fixed 当前更准确地说是基于载波相位连续性的 fixed/smoothed 轨迹生成，尚未实现严格的双差载波相位整周模糊度固定。
- INS 当前实现基础相对位移积分，尚未加入完整姿态更新、误差状态建模和 IMU 标定参数。
- RTK/INS 松组合当前采用固定增益预测-更新，后续可以扩展为误差状态卡尔曼滤波。

## 后续改进方向

- 加入电离层、对流层、卫星高度角和多系统多频改正。
- 实现双差载波相位模型、周跳探测和 LAMBDA 整周模糊度固定。
- 完善 INS 姿态更新、导航系转换和 IMU 误差模型。
- 将 RTK/INS 松组合改为误差状态卡尔曼滤波。
- 增加 ENU 误差曲线、固定率统计、DOP、残差和与参考结果的对比报告。
