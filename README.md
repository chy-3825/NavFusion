# NavFusion 项目说明

`NavFusion` 是一个基于 Win32 C++ 编写的 GNSS/INS 坐标结算、轨迹绘图和结果保存程序。程序可以从默认 `sj` 数据文件夹读取示例数据，也可以在图形界面中手动选择动态站观测文件、导航星历文件、IMU 原始数据文件、基站观测文件和输出目录。

用户选择数据后，可以按界面按钮依次完成输入检查、SPP 单点定位、RTK float、RTK fixed 和 RTK/INS 松组合处理。每一步都会生成独立的轨迹结果，并在界面右侧显示当前步骤的轨迹图预览。界面还支持将当前正在显示的轨迹图片和对应 CSV 另存到用户指定位置。

## 主要功能

- 支持从默认 `sj` 文件夹读取数据。
- 支持在界面中选择自定义数据来源。
- 支持动态站 RINEX 观测文件、基站 RINEX 观测文件、GPS 导航星历文件和 IMR 原始惯导文件。
- 支持按步骤完成 SPP、RTK float、RTK fixed 和 RTK/INS 松组合轨迹结算。
- 每个结算步骤都会输出 CSV、SVG 和 BMP 三种结果文件。
- 界面右侧可以预览当前步骤的 BMP 轨迹图。
- 可以保存当前显示的轨迹图片。
- 可以保存当前显示轨迹对应的 CSV 文件。
- 支持命令行参数使用默认数据自动运行，便于测试。

## 默认输入

默认数据位于项目的 `sj` 文件夹中：

- `sj/Observation.26o`：动态站 RINEX 观测文件
- `sj/Navigation.26p`：GPS 广播星历导航文件
- `sj/IMU_Raw_Data.dat.imr`：IMR 原始惯导数据
- `sj/Base_Station.rnx`：基站 RINEX 观测文件

程序运行时会从可执行文件所在目录向上查找 `sj` 文件夹。因此即使外层项目文件夹改名，也不需要修改源码中的数据路径。

## 自定义数据来源

正常启动程序后会进入图形界面。用户可以在界面中分别选择：

- 动态站观测文件
- 导航星历文件
- IMU 原始数据文件
- 基站观测文件
- 输出目录

选择完成后，先执行输入检查，再按顺序执行后续解算步骤。这样程序既可以使用项目自带的默认数据，也可以更换为用户自己的 GNSS/INS 数据进行处理。

## 处理流程

程序的主要处理流程如下：

1. 检查输入文件是否存在，并检查输出目录是否有效。
2. 读取动态站 RINEX 观测文件和 GPS 导航星历文件。
3. 使用 GPS C1C 伪距进行 SPP 单点定位，得到动态站 ECEF 轨迹。
4. 读取基站 RINEX 观测文件，使用动态站和基站共同 GPS 卫星进行 RTK float 单差伪距解算。
5. 在 RTK float 轨迹基础上，利用 L1 载波相位连续性进行自适应平滑，得到 RTK fixed 轨迹。
6. 读取 IMR 原始惯导数据，进行基础 INS 相对位移积分。
7. 使用 IMU 预测和 RTK fixed 轨迹更新，得到 RTK/INS 松组合轨迹。
8. 为每一步生成 CSV、SVG 和 BMP 结果，并在界面中显示当前结果。

## 默认输出

默认输出目录为 `output`。每个步骤都会输出 `.csv`、`.svg` 和 `.bmp` 三种文件：

- `spp_trajectory.*`：SPP 单点定位轨迹
- `rtk_float_trajectory.*`：RTK float 单差伪距轨迹
- `rtk_fixed_trajectory.*`：利用载波连续性平滑后的 RTK fixed 轨迹
- `rtkins_loose_trajectory.*`：IMU 预测和 RTK 更新后的松组合轨迹

CSV 文件保存轨迹点的 ECEF 坐标和质量标记；SVG 文件用于高清查看或报告插图；BMP 文件用于 Win32 界面右侧预览。

程序关闭时会清理 `output` 中由程序自动生成的临时结果，但不会删除用户通过保存按钮另存到其他位置的图片或 CSV。

## 运行方式

正常运行会打开图形界面：

```text
x64/Debug/NavFusion.exe
```

也可以使用默认数据进行自动测试：

```text
x64/Debug/NavFusion.exe --run-spp-default
x64/Debug/NavFusion.exe --run-all-default
```

其中：

- `--run-spp-default`：只使用默认数据运行 SPP 步骤。
- `--run-all-default`：使用默认数据依次运行 SPP、RTK float、RTK fixed 和 RTK/INS 松组合步骤。

## 代码结构

### `NavFusion/main.cpp`

程序入口。正常运行时启动 Win32 图形界面；带自动测试参数时，不打开窗口，直接使用默认数据运行处理流程。

### `NavFusion/App.cpp`

负责 Win32 界面，包括文件选择、步骤按钮、日志显示、轨迹图预览、保存当前图片和保存当前 CSV。

### `NavFusion/ProcessingPipeline.cpp`

负责连接界面和算法模块。它接收界面选择的文件路径，检查输入，调用 SPP、RTK float、RTK fixed 和 RTK/INS 松组合模块，并调用绘图模块输出结果。

### `NavFusion/RinexReader.cpp`

负责解析 RINEX 观测文件和导航文件。当前主要读取 GPS `C1C` 伪距、`L1C` 载波相位、历元时间、近似坐标和 GPS 广播星历参数。

### `NavFusion/SppSolver.cpp`

负责 SPP 单点定位。当前使用 GPS `C1C` 伪距和广播星历，计算卫星位置、卫星钟差和地球自转改正，并通过逐历元最小二乘求解接收机 ECEF 坐标。

### `NavFusion/RtkSolver.cpp`

负责 RTK float 和 RTK fixed 解算。RTK float 使用动态站和基站同历元共同 GPS 卫星建立单差伪距方程；RTK fixed 在 float 轨迹基础上利用 `L1C` 载波连续性进行平滑。

### `NavFusion/ImuReader.cpp`

负责读取 IMR 原始惯导文件。当前识别 `$IMURAW` 文件头，读取时间和六轴增量数据，并转换为 `ImuSample`。

### `NavFusion/InsSolver.cpp`

负责基础 INS 机械编排。当前对 IMU 加速度进行初始偏置估计，并积分得到相对位移轨迹。

### `NavFusion/RtkInsLoose.cpp`

负责 RTK/INS 松组合。当前先使用 IMU 得到相对预测，再使用 RTK fixed 轨迹作为位置观测进行更新。

### `NavFusion/Geo.cpp`

负责 WGS-84 坐标和地球模型工具，包括 LLH/ECEF 转换、ECEF 到 NED 转换和正常重力计算。

### `NavFusion/Plot.cpp`

负责输出轨迹结果。CSV 保存轨迹坐标，SVG 用于高清查看，BMP 用于界面预览。

## 后续改进方向

本项目已经完成数据读取、坐标解算、轨迹绘图、界面预览和结果保存流程。后面如果继续完善算法，可以从下面几个方向入手：

- SPP 当前主要使用 GPS 单频 `C1C` 伪距，尚未加入完整电离层、对流层、多系统和多频改正。
- RTK float 当前采用共同卫星单差伪距方程。
- RTK fixed 当前主要利用载波连续性对 float 轨迹进行平滑，后面可以加入双差载波相位和整周模糊度固定。
- INS 当前先做相对积分，后面可以补充姿态更新和导航系转换。
- RTK/INS 松组合当前使用固定增益预测-更新，后面可以改成误差状态卡尔曼滤波。

此外还可以继续加入周跳探测、IMU 标定参数、完整姿态更新和更细的质量控制。
