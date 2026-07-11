# NavFusion

NavFusion 是一个基于 Win32 C++ 的 GNSS/INS 轨迹解算与可视化程序。项目支持读取动态站 RINEX 观测文件、GPS 导航星历文件、IMR 惯导原始数据和基站观测文件，并按流程完成 SPP、RTK float、RTK fixed/smoothed、RTK/INS 松组合，以及 IMU Allan 方差分析。

项目自带示例数据和 Windows 可执行程序，可以直接运行图形界面复现处理流程。每个解算步骤会生成 CSV/SVG/BMP 结果，图形界面右侧可以预览当前结果，并可将当前图像或 CSV 另存到用户指定位置。

## 已验证功能

- 从默认 `sj` 文件夹读取示例数据。
- 在图形界面中手动选择动态站 O 文件、导航 P 文件、IMR 文件、基站 O 文件和输出目录。
- 检查输入文件是否存在、输出目录是否可用。
- 使用 GPS `C1C` 伪距和广播星历进行 SPP 单点定位。
- 使用动态站/基站共同 GPS 卫星进行 RTK float 单差伪距解算。
- 在 RTK float 轨迹基础上，利用 `L1C` 载波相位连续性生成 RTK fixed/smoothed 轨迹。
- 读取 IMR 原始惯导数据，生成基础纯 INS 相对位移轨迹。
- 使用 IMU 预测和 RTK fixed/smoothed 轨迹进行 RTK/INS 松组合。
- 计算 IMU Allan 方差/Allan deviation，并输出 CSV/SVG/BMP。
- `INS + Allan` 可在 `1 Check data` 通过后直接运行，不需要等 SPP/RTK 流程结束。
- `INS + Allan` 右侧默认预览 Allan 方差分析图，而不是容易误解的纯 INS 漂移轨迹。
- 每一步结果可以通过按钮保存当前图像和当前 CSV。
- 程序退出时清理 `output` 中自动生成的临时结果，不会删除用户另存的文件。

## 快速运行

项目包含 Debug 可执行程序：

```text
x64/Debug/NavFusion.exe
```

直接运行会打开 Win32 图形界面。也可以使用命令行参数运行默认示例数据：

```text
x64/Debug/NavFusion.exe --run-spp-default
x64/Debug/NavFusion.exe --run-all-default
x64/Debug/NavFusion.exe --run-ins-default
```

参数说明：

- `--run-spp-default`：只运行默认数据的 SPP 步骤。
- `--run-all-default`：依次运行 SPP、RTK float、RTK fixed/smoothed、RTK/INS 松组合和 INS/Allan 分析。
- `--run-ins-default`：只运行默认 IMR 数据的纯 INS 轨迹生成和 Allan 方差分析，输出到 `output_check`，用于回归验证。

## 输入数据

默认数据位于 `sj` 文件夹：

- `sj/Observation.26o`：动态站 RINEX 观测文件。
- `sj/Navigation.26p`：GPS 广播星历文件。
- `sj/IMU_Raw_Data.dat.imr`：IMR 原始惯导数据。
- `sj/Base_Station.rnx`：基站 RINEX 观测文件。

程序运行时会从可执行文件所在目录向上查找 `sj` 文件夹，因此外层项目目录改名后通常不需要修改源码中的默认路径。

## 图形界面流程

打开程序后，可以在界面中选择：

- `Rover O`：动态站观测文件。
- `Nav P`：导航星历文件。
- `IMR`：惯导原始数据文件。
- `Base O`：基站观测文件。
- `Output`：输出目录。

正常 GNSS/组合导航流程：

1. `1 Check data`
2. `2 SPP track`
3. `3 RTK float`
4. `4 RTK fixed`
5. `5 RTK/INS LC`

惯导噪声分析流程：

1. `1 Check data`
2. `INS + Allan`

`INS + Allan` 会生成纯 INS 相对轨迹文件和 Allan 方差分析文件。右侧预览区默认显示 `allan_variance.bmp`，保存按钮默认保存 Allan 图和 Allan CSV。

## 输出说明

默认输出目录为 `output`：

- `spp_trajectory.*`：SPP 单点定位轨迹。
- `rtk_float_trajectory.*`：RTK float 单差伪距轨迹。
- `rtk_fixed_trajectory.*`：基于载波连续性平滑后的 RTK fixed/smoothed 轨迹。
- `rtkins_loose_trajectory.*`：RTK/INS 松组合轨迹。
- `pure_ins_trajectory.*`：纯 INS 相对位移轨迹。
- `allan_variance.*`：IMU Allan 方差/Allan deviation 分析结果。

其中：

- 轨迹 CSV 保存时间、坐标和质量标记。
- 纯 INS CSV 保存相对位移，不表示可靠绝对坐标。
- Allan CSV 保存不同平均时间 `tau` 下的陀螺和加速度计三轴 variance/deviation。
- SVG 用于高清查看或报告插图。
- BMP 用于 Win32 界面右侧预览。

## Allan 方差说明

Allan 方差应使用静止或低动态 IMU 数据计算。项目会自动从 IMR 开头截取连续低动态片段进行 Allan 方差分析，避免把车辆转弯等真实运动误当作陀螺噪声。

如果整段数据没有足够低动态片段，程序会退回使用全部 IMR 数据。此时 Allan 曲线可能包含运动影响，需要谨慎解释。

## 代码结构

- `NavFusion/main.cpp`：程序入口，支持图形界面和默认数据命令行回归参数。
- `NavFusion/App.cpp`：Win32 图形界面、文件选择、按钮流程、日志显示、预览和结果保存。
- `NavFusion/ProcessingPipeline.cpp`：连接界面和算法模块，负责输入检查、调度解算步骤和输出结果。
- `NavFusion/RinexReader.cpp`：读取 RINEX 观测文件和 GPS 导航星历。
- `NavFusion/SppSolver.cpp`：SPP 单点定位。
- `NavFusion/RtkSolver.cpp`：RTK float 和 fixed/smoothed 轨迹生成。
- `NavFusion/ImuReader.cpp`：读取 IMR 原始惯导文件，并按时间排序、去除重复样本。
- `NavFusion/InsSolver.cpp`：基础 INS 相对位移积分和 Allan 方差计算。
- `NavFusion/RtkInsLoose.cpp`：RTK/INS 松组合处理。
- `NavFusion/Geo.cpp`：WGS-84、LLH/ECEF、ECEF/NED 和正常重力等工具函数。
- `NavFusion/Plot.cpp`：输出 CSV、SVG 和 BMP 结果。

## 当前实现边界

本项目主要用于展示 GNSS/INS 数据读取、解算流程组织、轨迹输出和桌面界面集成能力。当前算法仍有简化：

- SPP 当前主要使用 GPS 单频 `C1C` 伪距，尚未加入完整电离层、对流层、多系统和多频改正。
- RTK float 当前采用共同卫星单差伪距方程。
- RTK fixed 当前更准确地说是基于载波相位连续性的 fixed/smoothed 轨迹生成，尚未实现严格的双差载波相位整周模糊度固定。
- 纯 INS 当前是基础相对位移积分，不等价于可靠绝对定位；没有 GNSS/RTK 更新时会自然漂移。
- RTK/INS 松组合当前采用固定增益预测/更新，后续可扩展为误差状态卡尔曼滤波。
- Allan 方差分析依赖低动态片段，若输入数据长期处于动态运动状态，结果应谨慎解释。

## 后续改进方向

- 加入电离层、对流层、卫星高度角和多系统多频改正。
- 实现双差载波相位模型、周跳探测和 LAMBDA 整周模糊度固定。
- 完善 INS 姿态更新、导航系转换、重力补偿和 IMU 误差模型。
- 将 RTK/INS 松组合改为误差状态卡尔曼滤波。
- 增加 ENU 误差曲线、固定率统计、DOP、残差和与参考结果的对比报告。
